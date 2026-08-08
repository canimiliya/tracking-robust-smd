"""Run the frozen 3-agent SMD reference through actual closed-loop dynamics."""

from __future__ import annotations

import argparse
import hashlib
import json
import pickle
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tr_smd.control.geometric_controller import ControllerConfig, GeometricController
from tr_smd.dynamics.quadrotor import QuadrotorDynamics, QuadrotorParameters, WrenchAllocator
from tr_smd.evaluation.closed_loop_metrics import compute_execution_metrics
from tr_smd.tracking.smd_reference import (
    HermiteReference,
    SMD_REFERENCE_DT_S,
    SMD_REFERENCE_SPAN_S,
    SMD_SUPPORT_POINTS,
    SMD_TRAJECTORY_DURATION_S,
    finite_difference_velocity,
    load_primary_reference,
)

DYNAMICS_DT = 0.002
CONTROL_DT = 0.01
ROBOT_RADIUS = 0.05


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--paths-file", type=Path, default=REPO_ROOT / "scripts/inference/results_s1_r0_3agent/2026-08-08-20-09-42/instance_name___EnvEmptyNoWait2DRobotCompositeThreePlanarDiskRandom/num_agents___3/planner___SMDComposite/single_agent_planner___SMDEnsemble/0/paths.npy")
    p.add_argument("--map-info-file", type=Path, default=REPO_ROOT / "scripts/inference/results_s1_r0_3agent/2026-08-08-20-09-42/instance_name___EnvEmptyNoWait2DRobotCompositeThreePlanarDiskRandom/num_agents___3/planner___SMDComposite/single_agent_planner___SMDEnsemble/0/map_info.pkl")
    p.add_argument("--output-root", type=Path, default=REPO_ROOT / "experiments")
    return p.parse_args()


def load_obstacles(map_info_file: Path):
    map_info = pickle.loads(map_info_file.read_bytes())
    map_records = pickle.loads((REPO_ROOT / "instances_data" / f"{map_info['map_name']}.pkl").read_bytes())
    records = map_records[int(map_info["instance_idx"])]
    return next(record for record in records if len(record[1]) == 3)


def simulate(reference, controller, dynamics, robot_data):
    n = reference.num_agents
    n_steps = int(round(SMD_TRAJECTORY_DURATION_S / DYNAMICS_DT))
    control_stride = int(round(CONTROL_DT / DYNAMICS_DT))
    time = np.arange(n_steps + 1, dtype=float) * DYNAMICS_DT
    nominal_position = np.empty((n_steps + 1, n, 3))
    nominal_velocity = np.empty_like(nominal_position)
    nominal_acceleration = np.empty_like(nominal_position)
    actual_position = np.empty_like(nominal_position)
    actual_velocity = np.empty_like(nominal_position)
    actual_quaternion = np.empty((n_steps + 1, n, 4))
    actual_body_rate = np.empty_like(nominal_velocity)
    rotor_thrust = np.empty((n_steps + 1, n, 4))
    states = []
    for robot in robot_data:
        xy = np.asarray(robot[0], dtype=float)
        states.append(np.concatenate([[xy[0], xy[1], 1.0], np.zeros(3), [1.0, 0.0, 0.0, 0.0], np.zeros(3)]))
    held_thrust = np.zeros((n, 4))
    for k, t in enumerate(time):
        p_ref, v_ref, a_ref = reference.evaluate(t)
        nominal_position[k], nominal_velocity[k], nominal_acceleration[k] = p_ref, v_ref, a_ref
        for i, state in enumerate(states):
            actual_position[k, i], actual_velocity[k, i] = state[:3], state[3:6]
            actual_quaternion[k, i], actual_body_rate[k, i] = state[6:10], state[10:13]
            rotor_thrust[k, i] = held_thrust[i]
        if k == n_steps:
            break
        if k % control_stride == 0:
            for i in range(n):
                held_thrust[i] = controller.command(states[i], p_ref[i], v_ref[i], a_ref[i], yaw_ref=0.0)["thrust"]
        for i, state in enumerate(states):
            states[i] = dynamics.rk4_step(state, controller.allocator.matrix @ held_thrust[i], DYNAMICS_DT)
    return {
        "time": time,
        "nominal_position": nominal_position.transpose(1, 0, 2),
        "nominal_velocity": nominal_velocity.transpose(1, 0, 2),
        "nominal_acceleration": nominal_acceleration.transpose(1, 0, 2),
        "actual_position": actual_position.transpose(1, 0, 2),
        "actual_velocity": actual_velocity.transpose(1, 0, 2),
        "actual_quaternion": actual_quaternion.transpose(1, 0, 2),
        "actual_body_rate": actual_body_rate.transpose(1, 0, 2),
        "rotor_thrust": rotor_thrust.transpose(1, 0, 2),
    }


def hash_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def make_figures(data, obstacles, out_dir: Path, params):
    out_dir.mkdir(parents=True, exist_ok=True)
    colors = ["tab:blue", "tab:orange", "tab:green"]
    fig, ax = plt.subplots(figsize=(7, 6))
    for center, radius in obstacles:
        ax.add_patch(plt.Circle(center, radius, color="gray", alpha=0.35))
    for i, color in enumerate(colors):
        ax.plot(data["nominal_position"][i, :, 0], data["nominal_position"][i, :, 1], "--", color=color, label=f"agent {i+1} SMD nominal")
        ax.plot(data["actual_position"][i, :, 0], data["actual_position"][i, :, 1], color=color, label=f"agent {i+1} actual")
        ax.scatter(data["nominal_position"][i, 0, 0], data["nominal_position"][i, 0, 1], color=color, marker="o")
        ax.scatter(data["nominal_position"][i, -1, 0], data["nominal_position"][i, -1, 1], color=color, marker="x")
    ax.set(xlabel="x [m]", ylabel="y [m]", title="SMD nominal vs closed-loop actual trajectories", aspect="equal")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r0_nominal_vs_actual_3agent.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for i, color in enumerate(colors):
        error = np.linalg.norm(data["actual_position"][i, :, :2] - data["nominal_position"][i, :, :2], axis=1)
        ax.plot(data["time"], error, color=color, label=f"agent {i+1}")
    ax.set(xlabel="physical time [s]", ylabel="XY tracking error [m]", title="Closed-loop tracking error")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r0_tracking_error_vs_time.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for i, color in enumerate(colors):
        for rotor in range(4):
            ax.plot(data["time"], data["rotor_thrust"][i, :, rotor], color=color, alpha=0.35 + rotor * 0.12, label=f"agent {i+1} rotor {rotor}")
    ax.axhline(params.rotor_max_thrust, color="black", linestyle="--", label="rotor upper bound")
    ax.set(xlabel="physical time [s]", ylabel="thrust [N]", title="Closed-loop rotor thrust")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r0_control_saturation.png", dpi=180)
    plt.close(fig)


def run_synthetic_test(target_xy):
    target_xy = np.asarray(target_xy, dtype=float)
    position = np.tile(target_xy, (1, SMD_SUPPORT_POINTS, 1))
    velocity = np.zeros_like(position)
    reference = HermiteReference(position, velocity)
    params = QuadrotorParameters()
    controller = GeometricController(params, ControllerConfig())
    data = simulate(reference, controller, QuadrotorDynamics(params), [([0.0, 0.0], [0.0, 0.0], 0.05)])
    metrics = compute_execution_metrics(
        data["time"], data["nominal_position"], data["actual_position"], data["actual_velocity"],
        data["actual_quaternion"], data["rotor_thrust"], [], ROBOT_RADIUS,
    )
    error = np.linalg.norm(data["actual_position"][0, :, :2] - data["nominal_position"][0, :, :2], axis=1)
    settled = np.where(np.maximum.accumulate(error[::-1])[::-1] <= 0.02)[0]
    metrics["settling_error_m"] = float(error[-1])
    metrics["settling_time_s"] = float(data["time"][settled[0]]) if len(settled) else None
    return metrics


def main():
    args = parse_args()
    output_root = args.output_root
    raw_dir, summary_dir, figure_dir = output_root / "raw", output_root / "summaries", output_root / "figures"
    source = load_primary_reference(args.paths_file, args.map_info_file, expected_agents=3)
    fd_velocity = finite_difference_velocity(source["position_xy"], SMD_REFERENCE_DT_S)
    velocity_error = source["velocity_xy"] - fd_velocity
    reference = HermiteReference(source["position_xy"], source["velocity_xy"])
    parity = reference.knot_parity()
    obstacles, robot_data = load_obstacles(args.map_info_file)
    params = QuadrotorParameters()
    allocator = WrenchAllocator(params)
    controller = GeometricController(params, ControllerConfig())
    dynamics = QuadrotorDynamics(params)
    data = simulate(reference, controller, dynamics, robot_data)
    metrics = compute_execution_metrics(data["time"], data["nominal_position"], data["actual_position"], data["actual_velocity"], data["actual_quaternion"], data["rotor_thrust"], obstacles, ROBOT_RADIUS)
    hover_metrics = run_synthetic_test([0.0, 0.0])
    setpoint_x_metrics = run_synthetic_test([0.10, 0.0])
    setpoint_y_metrics = run_synthetic_test([0.0, 0.10])
    raw_dir.mkdir(parents=True, exist_ok=True)
    summary_dir.mkdir(parents=True, exist_ok=True)
    raw_file = raw_dir / "s3_r0_closed_loop_3agent.npz"
    np.savez_compressed(raw_file, **data, position_error=data["actual_position"] - data["nominal_position"])
    make_figures(data, obstacles, figure_dir, params)
    s3_gate = {
        "finite_state": metrics["finite_state"],
        "all_agents_completed": True,
        "mean_xy_rmse_pass": metrics["mean_xy_rmse_m"] <= 0.02,
        "each_agent_xy_rmse_pass": bool(max(metrics["per_agent_xy_rmse_m"]) <= 0.03),
        "max_xy_error_pass": metrics["max_xy_error_all_agents_m"] <= 0.05,
        "each_agent_final_xy_error_pass": bool(max(metrics["per_agent_final_xy_error_m"]) <= 0.03),
        "z_rmse_pass": metrics["z_rmse_m"] <= 0.01,
        "max_tilt_pass": metrics["max_tilt_deg"] <= 45.0,
        "rotor_saturation_pass": metrics["rotor_saturation_fraction"] <= 0.10,
        "execution_geometric_collision_free": metrics["execution_geometric_collision_free"],
    }
    s3_gate["tracking_gate_pass"] = all(s3_gate.values())
    disk_before_gb = 189.808
    disk_after_gb = round(shutil.disk_usage(REPO_ROOT).free / (1024 ** 3), 3)
    summary = {
        "task_id": "S3-R0-CLOSED-LOOP-QUADROTOR-DYNAMICS-BRIDGE-R1",
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip(),
        "source_smd_run": "simple_3_idx0_s1r0",
        "source_paths_sha256": source["paths_sha256"],
        "reference_dt": SMD_REFERENCE_DT_S,
        "reference_span": SMD_REFERENCE_SPAN_S,
        "trajectory_duration": SMD_TRAJECTORY_DURATION_S,
        "support_points": SMD_SUPPORT_POINTS,
        "velocity_consistency_rmse": float(np.sqrt(np.mean(velocity_error * velocity_error))),
        "velocity_consistency_max_error": float(np.max(np.abs(velocity_error))),
        "interpolation": parity,
        "model_parameters": {"mass": params.mass, "inertia": params.inertia, "arm": params.arm, "kf": params.kf, "km": params.km, "thrust_to_weight": params.thrust_to_weight, "rotor_max_thrust": params.rotor_max_thrust, "rotor_positions": params.rotor_positions, "yaw_signs": params.yaw_signs, "source": params.model_source, "source_commit": params.model_source_commit},
        "controller_parameters": controller.config.as_dict(),
        "dynamics_dt": DYNAMICS_DT,
        "control_dt": CONTROL_DT,
        "integrator": "fixed_step_RK4",
        "state_dim": 13,
        "rotor_allocation_rank": allocator.rank,
        "s3_gate": s3_gate,
        "status": "SUBMITTED_FOR_REVIEW" if s3_gate["tracking_gate_pass"] else "BLOCKED",
        "final_label": "S3_R0_CLOSED_LOOP_QUADROTOR_BRIDGE_READY" if s3_gate["tracking_gate_pass"] else "BLOCKED_S3_R0_CLOSED_LOOP_QUADROTOR_BRIDGE",
        "disk_before_gb": disk_before_gb,
        "disk_after_gb": disk_after_gb,
        "disk_delta_gb": round(disk_after_gb - disk_before_gb, 3),
        "hover_test": hover_metrics,
        "setpoint_tests": {"x_plus_0.10_m": setpoint_x_metrics, "y_plus_0.10_m": setpoint_y_metrics},
        "tracking_metrics": metrics,
        "execution_collision_free": metrics["execution_geometric_collision_free"],
        "raw_artifact": str(raw_file),
        "raw_artifact_sha256": hash_file(raw_file),
        "figures": [str(figure_dir / f) for f in ["s3_r0_nominal_vs_actual_3agent.png", "s3_r0_tracking_error_vs_time.png", "s3_r0_control_saturation.png"]],
    }
    (summary_dir / "s3_r0_closed_loop_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
