"""Strict S4 nominal-to-execution safety evaluation utilities."""

from __future__ import annotations

import hashlib
import pickle
from pathlib import Path
from time import perf_counter

import numpy as np

from tr_smd.control.geometric_controller import ControllerConfig, GeometricController
from tr_smd.dynamics.quadrotor import QuadrotorDynamics, QuadrotorParameters
from tr_smd.tracking.smd_reference import (
    PositionConsistentHermiteReference,
    SMD_TRAJECTORY_DURATION_S,
)

DYNAMICS_DT_S = 0.002
CONTROL_DT_S = 0.01
ROBOT_RADIUS_M = 0.05


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def load_instance_geometry(repo_root: Path, map_name: str, instance_idx: int, num_agents: int):
    records = pickle.loads((repo_root / "instances_data" / f"{map_name}.pkl").read_bytes())
    matches = [record for record in records[int(instance_idx)] if len(record[1]) == num_agents]
    if len(matches) != 1:
        raise ValueError(f"expected one {num_agents}-agent record, found {len(matches)}")
    obstacles, robot_data = matches[0]
    return obstacles, robot_data


def sample_reference(reference: PositionConsistentHermiteReference):
    times = np.arange(0.0, SMD_TRAJECTORY_DURATION_S + 0.5 * DYNAMICS_DT_S, DYNAMICS_DT_S)
    evaluated = [reference.evaluate(float(t)) for t in times]
    return times, *(np.asarray([row[i] for row in evaluated]).transpose(1, 0, 2) for i in range(3))


def clearance_traces(position: np.ndarray, obstacles) -> dict:
    """Return strict XY clearances and the identity of each closest event."""
    xy = np.asarray(position, dtype=float)[..., :2]
    n_agents, n_times, _ = xy.shape
    pair_trace = np.full(n_times, np.inf)
    pair_ids = np.full((n_times, 2), -1, dtype=int)
    for i in range(n_agents):
        for j in range(i + 1, n_agents):
            clearance = np.linalg.norm(xy[i] - xy[j], axis=1) - 2.0 * ROBOT_RADIUS_M
            update = clearance < pair_trace
            pair_trace[update] = clearance[update]
            pair_ids[update] = (i, j)
    obstacle_trace = np.full(n_times, np.inf)
    obstacle_ids = np.full((n_times, 2), -1, dtype=int)
    for obstacle_idx, (center, radius) in enumerate(obstacles):
        clearance = np.linalg.norm(xy - np.asarray(center, dtype=float), axis=-1) - ROBOT_RADIUS_M - float(radius)
        agent_idx = np.argmin(clearance, axis=0)
        minimum = clearance[agent_idx, np.arange(n_times)]
        update = minimum < obstacle_trace
        obstacle_trace[update] = minimum[update]
        obstacle_ids[update, 0] = agent_idx[update]
        obstacle_ids[update, 1] = obstacle_idx
    combined = np.minimum(pair_trace, obstacle_trace)
    event_idx = int(np.argmin(combined))
    event_type = "agent-agent" if pair_trace[event_idx] <= obstacle_trace[event_idx] else "agent-obstacle"
    return {
        "pair": pair_trace,
        "obstacle": obstacle_trace,
        "combined": combined,
        "minimum": float(combined[event_idx]),
        "minimum_index": event_idx,
        "minimum_type": event_type,
        "minimum_pair": pair_ids[event_idx].tolist(),
        "minimum_obstacle": obstacle_ids[event_idx].tolist(),
        "agent_agent_collision": bool(np.any(pair_trace < 0.0)),
        "agent_obstacle_collision": bool(np.any(obstacle_trace < 0.0)),
    }


def dynamic_demand(velocity: np.ndarray, acceleration: np.ndarray, params: QuadrotorParameters) -> dict:
    acceleration_norm = np.linalg.norm(acceleration, axis=-1)
    force = params.mass * (acceleration - np.array([0.0, 0.0, -params.gravity]))
    thrust = np.linalg.norm(force, axis=-1)
    tilt = np.degrees(np.arccos(np.clip(force[..., 2] / np.maximum(thrust, 1e-15), -1.0, 1.0)))
    ratio = thrust / params.total_max_thrust
    return {
        "max_reference_speed_mps": float(np.max(np.linalg.norm(velocity, axis=-1))),
        "max_reference_acceleration_mps2": float(np.max(acceleration_norm)),
        "p99_reference_acceleration_mps2": float(np.percentile(acceleration_norm, 99.0)),
        "max_required_total_thrust_n": float(np.max(thrust)),
        "max_required_thrust_ratio": float(np.max(ratio)),
        "max_required_tilt_deg": float(np.max(tilt)),
        "reference_dynamic_feasible": bool(np.max(ratio) <= 1.0),
    }


def simulate(reference: PositionConsistentHermiteReference) -> tuple[dict, float]:
    params = QuadrotorParameters()
    controller = GeometricController(params, ControllerConfig())
    dynamics = QuadrotorDynamics(params)
    n = reference.num_agents
    n_steps = int(round(SMD_TRAJECTORY_DURATION_S / DYNAMICS_DT_S))
    stride = int(round(CONTROL_DT_S / DYNAMICS_DT_S))
    time = np.arange(n_steps + 1, dtype=float) * DYNAMICS_DT_S
    p0, v0, _ = reference.evaluate(0.0)
    states = np.column_stack((p0, v0, np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)), np.zeros((n, 3))))
    nominal_position = np.empty((n, n_steps + 1, 3))
    nominal_velocity = np.empty_like(nominal_position)
    nominal_acceleration = np.empty_like(nominal_position)
    actual_position = np.empty_like(nominal_position)
    actual_velocity = np.empty_like(nominal_position)
    actual_quaternion = np.empty((n, n_steps + 1, 4))
    actual_body_rate = np.empty_like(nominal_position)
    rotor_thrust = np.empty((n, n_steps + 1, 4))
    held_thrust = np.zeros((n, 4))
    started = perf_counter()
    for k, t in enumerate(time):
        p_ref, v_ref, a_ref = reference.evaluate(float(t))
        nominal_position[:, k], nominal_velocity[:, k], nominal_acceleration[:, k] = p_ref, v_ref, a_ref
        actual_position[:, k] = states[:, :3]
        actual_velocity[:, k] = states[:, 3:6]
        actual_quaternion[:, k] = states[:, 6:10]
        actual_body_rate[:, k] = states[:, 10:13]
        rotor_thrust[:, k] = held_thrust
        if k == n_steps:
            break
        if k % stride == 0:
            for i in range(n):
                held_thrust[i] = controller.command(states[i], p_ref[i], v_ref[i], a_ref[i], yaw_ref=0.0)["thrust"]
        for i in range(n):
            states[i] = dynamics.rk4_step(states[i], controller.allocator.matrix @ held_thrust[i], DYNAMICS_DT_S)
    runtime = perf_counter() - started
    return {
        "time": time,
        "nominal_position": nominal_position,
        "nominal_velocity": nominal_velocity,
        "nominal_acceleration": nominal_acceleration,
        "actual_position": actual_position,
        "actual_velocity": actual_velocity,
        "actual_quaternion": actual_quaternion,
        "actual_body_rate": actual_body_rate,
        "rotor_thrust": rotor_thrust,
    }, runtime


def execution_diagnostics(data: dict, obstacles, rotor_max_thrust: float) -> dict:
    error = data["actual_position"] - data["nominal_position"]
    xy_norm = np.linalg.norm(error[..., :2], axis=-1)
    per_agent_rmse = np.sqrt(np.mean(xy_norm * xy_norm, axis=1))
    actual_clearance = clearance_traces(data["actual_position"], obstacles)
    max_tilt = 0.0
    for q in data["actual_quaternion"].reshape(-1, 4):
        q = q / np.linalg.norm(q)
        max_tilt = max(max_tilt, float(np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2), -1.0, 1.0)))))
    finite = all(np.isfinite(data[key]).all() for key in ("actual_position", "actual_velocity", "actual_quaternion"))
    return {
        "tracking_position_rmse_m": float(np.sqrt(np.mean(xy_norm * xy_norm))),
        "mean_per_agent_tracking_rmse_m": float(np.mean(per_agent_rmse)),
        "tracking_max_error_m": float(np.max(xy_norm)),
        "actual_min_clearance_m": actual_clearance["minimum"],
        "actual_closest_approach_time_s": float(data["time"][actual_clearance["minimum_index"]]),
        "actual_collision_type": actual_clearance["minimum_type"] if actual_clearance["minimum"] < 0.0 else "none",
        "agent_agent_collision": actual_clearance["agent_agent_collision"],
        "agent_obstacle_collision": actual_clearance["agent_obstacle_collision"],
        "max_actual_tilt_deg": max_tilt,
        "rotor_saturation_fraction": float(np.mean(data["rotor_thrust"] >= rotor_max_thrust - 1e-12)),
        "finite_execution": finite,
        "execution_safe": bool(finite and actual_clearance["minimum"] >= 0.0),
        "clearance_trace": actual_clearance,
    }
