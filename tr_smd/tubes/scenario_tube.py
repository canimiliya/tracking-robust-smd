"""Reference-conditioned nonlinear scenario tracking tubes.

The tube is computed before execution from the frozen reference, controller,
dynamics, and a bounded disturbance set.  It never reads obstacle clearance or
an evaluation rollout.  A split-calibrated inflation accounts for the finite
disturbance scenario bank; this is an empirical conditional tube, not a formal
reachable-set guarantee.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from time import perf_counter
from typing import Iterable

import numpy as np

from tr_smd.control.geometric_controller import ControllerConfig, GeometricController
from tr_smd.dynamics.quadrotor import QuadrotorDynamics, QuadrotorParameters
from tr_smd.tracking.smd_reference import (
    PositionConsistentHermiteReference,
    SMD_REFERENCE_DT_S,
    SMD_REFERENCE_SPAN_S,
    SMD_SUPPORT_POINTS,
    SMD_TRAJECTORY_DURATION_S,
)

DYNAMICS_DT_S = 0.002
CONTROL_DT_S = 0.01
CONTROL_STEPS = int(round(SMD_TRAJECTORY_DURATION_S / CONTROL_DT_S))
DYNAMICS_STEPS = int(round(SMD_TRAJECTORY_DURATION_S / DYNAMICS_DT_S))


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    mode: str
    angle_rad: float = 0.0
    seed: int = 0
    switch_ticks: int = 25


@dataclass
class TrackingTube:
    time: np.ndarray
    rho_dense: np.ndarray
    rho_support: np.ndarray
    rho_segment: np.ndarray
    metadata: dict


def default_design_scenarios() -> list[ScenarioSpec]:
    scenarios = [ScenarioSpec("zero", "zero")]
    scenarios.extend(
        ScenarioSpec(f"common_fixed_{degrees:03d}", "common_fixed", np.deg2rad(degrees))
        for degrees in range(0, 360, 45)
    )
    scenarios.extend(
        ScenarioSpec(f"independent_fixed_{phase}", "independent_fixed", np.deg2rad(22.5 * phase))
        for phase in range(4)
    )
    scenarios.extend(ScenarioSpec(f"common_piecewise_{seed}", "common_piecewise", seed=seed) for seed in (701, 702, 703, 704))
    scenarios.extend(ScenarioSpec(f"independent_piecewise_{seed}", "independent_piecewise", seed=seed) for seed in (801, 802, 803, 804))
    return scenarios


def disturbance_schedule(spec: ScenarioSpec, num_agents: int, bound_n: float) -> np.ndarray:
    """Build a deterministic control-tick schedule satisfying ||F_xy|| <= bound."""
    if num_agents < 1 or bound_n < 0.0:
        raise ValueError("num_agents must be positive and bound_n nonnegative")
    out = np.zeros((num_agents, CONTROL_STEPS, 3), dtype=np.float64)
    if spec.mode == "zero" or bound_n == 0.0:
        return out
    if spec.mode == "common_fixed":
        direction = np.array([np.cos(spec.angle_rad), np.sin(spec.angle_rad)])
        out[:, :, :2] = bound_n * direction
    elif spec.mode == "independent_fixed":
        golden = np.pi * (3.0 - np.sqrt(5.0))
        for agent in range(num_agents):
            angle = spec.angle_rad + agent * golden
            out[agent, :, :2] = bound_n * np.array([np.cos(angle), np.sin(angle)])
    elif spec.mode in ("common_piecewise", "independent_piecewise"):
        if spec.switch_ticks < 1:
            raise ValueError("switch_ticks must be positive")
        rng = np.random.default_rng(spec.seed)
        blocks = int(np.ceil(CONTROL_STEPS / spec.switch_ticks))
        shape = (1 if spec.mode == "common_piecewise" else num_agents, blocks)
        angles = rng.uniform(-np.pi, np.pi, size=shape)
        for block in range(blocks):
            start = block * spec.switch_ticks
            stop = min((block + 1) * spec.switch_ticks, CONTROL_STEPS)
            for agent in range(num_agents):
                angle = angles[0 if shape[0] == 1 else agent, block]
                out[agent, start:stop, :2] = bound_n * np.array([np.cos(angle), np.sin(angle)])
    else:
        raise ValueError(f"unknown disturbance mode: {spec.mode}")
    max_norm = float(np.max(np.linalg.norm(out[..., :2], axis=-1)))
    if max_norm > bound_n + 1e-12 or np.any(out[..., 2] != 0.0):
        raise AssertionError("generated disturbance violates the frozen envelope")
    return out


def simulate_scheduled(
    reference: PositionConsistentHermiteReference,
    force_schedule: np.ndarray,
) -> tuple[dict, float]:
    """Execute the frozen S4 stack with a per-agent control-tick force schedule."""
    params = QuadrotorParameters()
    controller = GeometricController(params, ControllerConfig())
    dynamics = QuadrotorDynamics(params)
    n = reference.num_agents
    schedule = np.asarray(force_schedule, dtype=float)
    if schedule.shape != (n, CONTROL_STEPS, 3):
        raise ValueError(f"expected force schedule {(n, CONTROL_STEPS, 3)}, got {schedule.shape}")
    stride = int(round(CONTROL_DT_S / DYNAMICS_DT_S))
    time = np.arange(DYNAMICS_STEPS + 1, dtype=float) * DYNAMICS_DT_S
    p0, v0, _ = reference.evaluate(0.0)
    states = np.column_stack((p0, v0, np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)), np.zeros((n, 3))))
    actual_position = np.empty((n, DYNAMICS_STEPS + 1, 3))
    actual_velocity = np.empty_like(actual_position)
    actual_quaternion = np.empty((n, DYNAMICS_STEPS + 1, 4))
    actual_body_rate = np.empty_like(actual_position)
    nominal_position = np.empty_like(actual_position)
    nominal_velocity = np.empty_like(actual_position)
    nominal_acceleration = np.empty_like(actual_position)
    rotor_thrust = np.empty((n, DYNAMICS_STEPS + 1, 4))
    held_thrust = np.zeros((n, 4))
    started = perf_counter()
    for k, t in enumerate(time):
        p_ref, v_ref, a_ref = reference.evaluate(float(t))
        nominal_position[:, k], nominal_velocity[:, k], nominal_acceleration[:, k] = p_ref, v_ref, a_ref
        actual_position[:, k], actual_velocity[:, k] = states[:, :3], states[:, 3:6]
        actual_quaternion[:, k], actual_body_rate[:, k] = states[:, 6:10], states[:, 10:13]
        rotor_thrust[:, k] = held_thrust
        if k == DYNAMICS_STEPS:
            break
        if k % stride == 0:
            for agent in range(n):
                held_thrust[agent] = controller.command(states[agent], p_ref[agent], v_ref[agent], a_ref[agent], yaw_ref=0.0)["thrust"]
        tick = min(k // stride, CONTROL_STEPS - 1)
        for agent in range(n):
            states[agent] = dynamics.rk4_step(
                states[agent],
                controller.allocator.matrix @ held_thrust[agent],
                DYNAMICS_DT_S,
                external_force_world=schedule[agent, tick],
            )
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
    }, perf_counter() - started


def _scenario_error(payload: tuple[np.ndarray, np.ndarray, float, ScenarioSpec]) -> tuple[str, np.ndarray, float]:
    position_xy, raw_velocity_xy, bound_n, spec = payload
    reference = PositionConsistentHermiteReference(position_xy, raw_velocity_xy)
    schedule = disturbance_schedule(spec, reference.num_agents, bound_n)
    data, runtime = simulate_scheduled(reference, schedule)
    error = np.linalg.norm(data["actual_position"] - data["nominal_position"], axis=-1)
    return spec.name, error, runtime


def _conservative_support_mapping(time: np.ndarray, rho_dense: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    support_time = np.arange(SMD_SUPPORT_POINTS) * SMD_REFERENCE_DT_S
    boundaries = np.concatenate(([-np.inf], 0.5 * (support_time[:-1] + support_time[1:]), [np.inf]))
    support = np.empty((rho_dense.shape[0], SMD_SUPPORT_POINTS))
    for k in range(SMD_SUPPORT_POINTS):
        mask = (time >= boundaries[k]) & (time < boundaries[k + 1])
        support[:, k] = np.max(rho_dense[:, mask], axis=1)
    segment = np.empty((rho_dense.shape[0], SMD_SUPPORT_POINTS - 1))
    for k in range(SMD_SUPPORT_POINTS - 1):
        mask = (time >= support_time[k] - 1e-12) & (time <= support_time[k + 1] + 1e-12)
        segment[:, k] = np.max(rho_dense[:, mask], axis=1)
    return support, segment


def compute_tracking_tube(
    reference: PositionConsistentHermiteReference,
    disturbance_bound: float,
    controller_config: ControllerConfig | None = None,
    dynamics_config: QuadrotorParameters | None = None,
    *,
    inflation_factor: float = 1.0,
    additive_inflation_m: float = 1e-9,
    scenarios: Iterable[ScenarioSpec] | None = None,
    workers: int = 1,
) -> TrackingTube:
    """Compute a pre-execution nonlinear scenario tube and SMD-safe mappings."""
    if controller_config not in (None, ControllerConfig()):
        raise ValueError("S5 only accepts the frozen ControllerConfig")
    if dynamics_config not in (None, QuadrotorParameters()):
        raise ValueError("S5 only accepts the frozen QuadrotorParameters")
    if inflation_factor < 1.0 or additive_inflation_m < 0.0:
        raise ValueError("tube inflation must be conservative")
    scenario_list = list(default_design_scenarios() if scenarios is None else scenarios)
    payloads = [(reference.position_xy, reference.raw_velocity_xy, disturbance_bound, spec) for spec in scenario_list]
    started = perf_counter()
    if workers > 1 and len(payloads) > 1:
        with ProcessPoolExecutor(max_workers=min(workers, len(payloads))) as pool:
            results = list(pool.map(_scenario_error, payloads))
    else:
        results = [_scenario_error(payload) for payload in payloads]
    base = np.max(np.stack([result[1] for result in results]), axis=0)
    rho_dense = inflation_factor * base + additive_inflation_m
    rho_dense[:, 0] = max(additive_inflation_m, 0.0)
    time = np.arange(DYNAMICS_STEPS + 1, dtype=float) * DYNAMICS_DT_S
    rho_support, rho_segment = _conservative_support_mapping(time, rho_dense)
    elapsed = perf_counter() - started
    return TrackingTube(
        time=time,
        rho_dense=rho_dense,
        rho_support=rho_support,
        rho_segment=rho_segment,
        metadata={
            "method": "nonlinear_reference_conditioned_scenario_envelope",
            "primary_disturbance_bound_n": float(disturbance_bound),
            "scenario_count": len(scenario_list),
            "scenario_names": [spec.name for spec in scenario_list],
            "inflation_factor": float(inflation_factor),
            "additive_inflation_m": float(additive_inflation_m),
            "control_dt_s": CONTROL_DT_S,
            "dynamics_dt_s": DYNAMICS_DT_S,
            "tube_computation_time_s": elapsed,
            "scenario_simulation_time_s": float(sum(result[2] for result in results)),
            "reference_span_s": SMD_REFERENCE_SPAN_S,
            "formal_guarantee": False,
        },
    )
