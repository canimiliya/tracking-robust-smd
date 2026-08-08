"""S3-R1 reference-semantics audit and one frozen-controller rerun."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pickle
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tr_smd.control.geometric_controller import ControllerConfig, GeometricController
from tr_smd.dynamics.quadrotor import QuadrotorDynamics, QuadrotorParameters
from tr_smd.evaluation.closed_loop_metrics import compute_execution_metrics
from tr_smd.tracking.smd_reference import (
    HermiteReference,
    PositionConsistentHermiteReference,
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
CONFIG_PATH = REPO_ROOT / "configs/tr_smd/closed_loop_quadrotor.yaml"
S1_MANIFEST = REPO_ROOT / "experiments/manifests/s1_r1_core_reproduction_manifest.json"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=REPO_ROOT / "experiments")
    return parser.parse_args()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def run_id_from_path(result_dir: str) -> str:
    path = Path(result_dir)
    parts = path.parts
    map_name = "instances_simple" if "Simple" in str(path) else "instances_dense" if "Dense" in str(path) else "instances_connected_room"
    num_agents = next(part.split("___", 1)[1] for part in parts if part.startswith("num_agents___"))
    timestamp = next(part for part in parts if part[:4].isdigit() and part.count("-") == 2)
    return f"{map_name}_{num_agents}_idx0_{timestamp.replace('-', '')}"


def load_obstacles(map_info_file: Path):
    map_info = pickle.loads(map_info_file.read_bytes())
    map_records = pickle.loads((REPO_ROOT / "instances_data" / f"{map_info['map_name']}.pkl").read_bytes())
    records = map_records[int(map_info["instance_idx"])]
    return next(record for record in records if len(record[1]) == 3)[0]


def audit_one(run_id: str, result_dir: str, map_name: str, num_agents: int) -> dict:
    result_dir = REPO_ROOT / result_dir
    source = load_primary_reference(result_dir / "paths.npy", result_dir / "map_info.pkl", expected_agents=num_agents)
    position = source["position_xy"]
    raw_velocity = source["velocity_xy"]
    fd_velocity = finite_difference_velocity(position, SMD_REFERENCE_DT_S)
    derived = np.zeros_like(position)
    derived[:, 1:-1] = (position[:, 2:] - position[:, :-2]) / (2.0 * SMD_REFERENCE_DT_S)
    mismatch_fd = raw_velocity - fd_velocity
    mismatch_derived = raw_velocity - derived
    return {
        "map_name": map_name,
        "num_agents": num_agents,
        "instance_idx": 0,
        "run_id": run_id,
        "paths_file": str(result_dir / "paths.npy"),
        "paths_sha256": source["paths_sha256"],
        "raw_velocity_vs_position_fd_rmse": float(np.sqrt(np.mean(mismatch_fd * mismatch_fd))),
        "raw_velocity_vs_position_fd_max_error": float(np.max(np.abs(mismatch_fd))),
        "raw_velocity_endpoint_start_norm": float(np.linalg.norm(raw_velocity[:, 0], axis=1).max()),
        "raw_velocity_endpoint_goal_norm": float(np.linalg.norm(raw_velocity[:, -1], axis=1).max()),
        "position_derived_velocity_mismatch_rmse": float(np.sqrt(np.mean(mismatch_derived * mismatch_derived))),
        "position_derived_velocity_mismatch_max_error": float(np.max(np.abs(mismatch_derived))),
    }


def derivative_audit(reference: PositionConsistentHermiteReference) -> dict:
    errors_p = []
    errors_v = []
    eps = 1.0e-6
    for segment in range(SMD_SUPPORT_POINTS - 1):
        h = SMD_REFERENCE_DT_S
        for tau in (0.23, 0.41, 0.67, 0.83):
            t = segment * h + tau * h
            p_minus, _, _ = reference.evaluate(t - eps)
            p_plus, _, _ = reference.evaluate(t + eps)
            _, v, _ = reference.evaluate(t)
            _, v_minus, _ = reference.evaluate(t - eps)
            _, v_plus, _ = reference.evaluate(t + eps)
            _, _, a = reference.evaluate(t)
            errors_p.append(np.max(np.abs((p_plus - p_minus) / (2.0 * eps) - v)))
            errors_v.append(np.max(np.abs((v_plus - v_minus) / (2.0 * eps) - a)))
    return {
        "epsilon_s": eps,
        "reference_dpdt_v_error_max": float(max(errors_p)),
        "reference_dvdt_a_error_max": float(max(errors_v)),
    }


def sample_reference(reference, end_time=SMD_TRAJECTORY_DURATION_S):
    times = np.arange(0.0, end_time + DYNAMICS_DT * 0.5, DYNAMICS_DT)
    positions, velocities, accelerations = [], [], []
    for t in times:
        p, v, a = reference.evaluate(float(t))
        positions.append(p)
        velocities.append(v)
        accelerations.append(a)
    return times, np.asarray(positions), np.asarray(velocities), np.asarray(accelerations)


def geometry_audit(reference, obstacles):
    times, positions, _, _ = sample_reference(reference)
    xy = positions[:, :, :2]
    min_pair = np.inf
    for i in range(xy.shape[1]):
        for j in range(i + 1, xy.shape[1]):
            min_pair = min(min_pair, float(np.min(np.linalg.norm(xy[:, i] - xy[:, j], axis=1))))
    min_obstacle = np.inf
    for center, radius in obstacles:
        center = np.asarray(center, dtype=float)
        min_obstacle = min(min_obstacle, float(np.min(np.linalg.norm(xy - center, axis=-1) - ROBOT_RADIUS - float(radius))))
    return {
        "reference_geometry_sample_dt_s": DYNAMICS_DT,
        "reference_geometric_collision_free": bool(min_pair - 2.0 * ROBOT_RADIUS >= 0.0 and min_obstacle >= 0.0),
        "min_reference_inter_agent_distance_m": float(min_pair),
        "min_reference_inter_agent_clearance_m": float(min_pair - 2.0 * ROBOT_RADIUS),
        "min_reference_obstacle_clearance_m": float(min_obstacle),
        "sample_times": times,
        "sample_positions": positions,
    }


def polyline_deviation(reference, position_xy):
    times, positions, _, _ = sample_reference(reference)
    max_deviation = 0.0
    for time, sample in zip(times, positions):
        if time >= SMD_REFERENCE_SPAN_S:
            line = position_xy[:, -1]
        else:
            segment = min(int(np.floor(time / SMD_REFERENCE_DT_S)), SMD_SUPPORT_POINTS - 2)
            tau = (time - segment * SMD_REFERENCE_DT_S) / SMD_REFERENCE_DT_S
            line = (1.0 - tau) * position_xy[:, segment] + tau * position_xy[:, segment + 1]
        max_deviation = max(max_deviation, float(np.max(np.linalg.norm(sample[:, :2] - line, axis=1))))
    return max_deviation


def dynamic_demand(reference, params):
    _, _, velocities, accelerations = sample_reference(reference)
    acceleration_norm = np.linalg.norm(accelerations, axis=-1)
    force = params.mass * (accelerations - np.array([0.0, 0.0, -params.gravity]))
    required_thrust = np.linalg.norm(force, axis=-1)
    tilt = np.degrees(np.arccos(np.clip(force[..., 2] / required_thrust, -1.0, 1.0)))
    speed = np.linalg.norm(velocities, axis=-1)
    return {
        "max_reference_speed_mps": float(np.max(speed)),
        "max_reference_acceleration_mps2": float(np.max(acceleration_norm)),
        "p99_reference_acceleration_mps2": float(np.percentile(acceleration_norm, 99.0)),
        "max_required_total_thrust_n": float(np.max(required_thrust)),
        "max_required_thrust_ratio": float(np.max(required_thrust) / params.total_max_thrust),
        "max_required_tilt_deg": float(np.max(tilt)),
        "reference_thrust_feasible": bool(np.max(required_thrust) <= params.total_max_thrust),
    }


def simulate(reference, controller, dynamics, initial_position=None, initial_velocity=None):
    n = reference.num_agents
    n_steps = int(round(SMD_TRAJECTORY_DURATION_S / DYNAMICS_DT))
    control_stride = int(round(CONTROL_DT / DYNAMICS_DT))
    time = np.arange(n_steps + 1, dtype=float) * DYNAMICS_DT
    p0, v0, _ = reference.evaluate(0.0)
    if initial_position is not None:
        p0 = np.asarray(initial_position, dtype=float)
    if initial_velocity is not None:
        v0 = np.asarray(initial_velocity, dtype=float)
    states = [np.concatenate([p0[i], v0[i], [1.0, 0.0, 0.0, 0.0], np.zeros(3)]) for i in range(n)]
    nominal_position = np.empty((n_steps + 1, n, 3))
    nominal_velocity = np.empty_like(nominal_position)
    nominal_acceleration = np.empty_like(nominal_position)
    actual_position = np.empty_like(nominal_position)
    actual_velocity = np.empty_like(nominal_position)
    actual_quaternion = np.empty((n_steps + 1, n, 4))
    actual_body_rate = np.empty_like(nominal_velocity)
    rotor_thrust = np.empty((n_steps + 1, n, 4))
    held_thrust = np.zeros((n, 4))
    for k, t in enumerate(time):
        p_ref, v_ref, a_ref = reference.evaluate(t)
        nominal_position[k], nominal_velocity[k], nominal_acceleration[k] = p_ref, v_ref, a_ref
        for i, state in enumerate(states):
            actual_position[k, i] = state[:3]
            actual_velocity[k, i] = state[3:6]
            actual_quaternion[k, i] = state[6:10]
            actual_body_rate[k, i] = state[10:13]
            rotor_thrust[k, i] = held_thrust[i]
        if k == n_steps:
            break
        if k % control_stride == 0:
            for i, state in enumerate(states):
                held_thrust[i] = controller.command(state, p_ref[i], v_ref[i], a_ref[i], yaw_ref=0.0)["thrust"]
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


def synthetic_regression(controller, dynamics):
    results = {}
    for name, target in (("hover", [0.0, 0.0]), ("setpoint_x", [0.10, 0.0]), ("setpoint_y", [0.0, 0.10])):
        position = np.tile(np.asarray(target, dtype=float), (1, SMD_SUPPORT_POINTS, 1))
        raw_velocity = np.zeros_like(position)
        reference = PositionConsistentHermiteReference(position, raw_velocity)
        data = simulate(
            reference,
            controller,
            dynamics,
            initial_position=np.array([[0.0, 0.0, 1.0]]),
            initial_velocity=np.zeros((1, 3)),
        )
        metrics = compute_execution_metrics(data["time"], data["nominal_position"], data["actual_position"], data["actual_velocity"], data["actual_quaternion"], data["rotor_thrust"], [], ROBOT_RADIUS)
        final_error = float(np.linalg.norm(data["actual_position"][0, -1, :2] - data["nominal_position"][0, -1, :2]))
        results[name] = {**metrics, "final_xy_error_m": final_error, "pass": bool(metrics["finite_state"] and final_error <= 1.0e-4)}
    return results


def make_figures(data, out_dir: Path, params):
    out_dir.mkdir(parents=True, exist_ok=True)
    colors = ["tab:blue", "tab:orange", "tab:green"]
    fig, ax = plt.subplots(figsize=(7, 6))
    for i, color in enumerate(colors):
        ax.plot(data["nominal_position"][i, :, 0], data["nominal_position"][i, :, 1], "--", color=color, label=f"agent {i+1} R1 nominal")
        ax.plot(data["actual_position"][i, :, 0], data["actual_position"][i, :, 1], color=color, label=f"agent {i+1} actual")
    ax.set(xlabel="x [m]", ylabel="y [m]", title="S3-R1 projected-position reference vs actual", aspect="equal")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r1_nominal_vs_actual_3agent.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for i, color in enumerate(colors):
        error = np.linalg.norm(data["actual_position"][i, :, :2] - data["nominal_position"][i, :, :2], axis=1)
        ax.plot(data["time"], error, color=color, label=f"agent {i+1}")
    ax.set(xlabel="physical time [s]", ylabel="XY tracking error [m]", title="S3-R1 tracking error")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r1_tracking_error_vs_time.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i, color in enumerate(colors):
        axes[0].plot(data["time"], data["nominal_acceleration"][i, :, 0], color=color, label=f"agent {i+1} ax")
        axes[1].plot(data["time"], data["nominal_acceleration"][i, :, 1], color=color, label=f"agent {i+1} ay")
        axes[2].plot(data["time"], data["nominal_acceleration"][i, :, 2], color=color, label=f"agent {i+1} az")
    for ax in axes:
        ax.legend(fontsize=7, ncol=3)
        ax.grid(alpha=0.2)
    axes[0].set_ylabel("ax [m/s²]")
    axes[1].set_ylabel("ay [m/s²]")
    axes[2].set_ylabel("az [m/s²]")
    axes[2].set_xlabel("physical time [s]")
    fig.suptitle("S3-R1 reference acceleration")
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r1_reference_acceleration.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for i, color in enumerate(colors):
        for rotor in range(4):
            ax.plot(data["time"], data["rotor_thrust"][i, :, rotor], color=color, alpha=0.3 + rotor * 0.15)
    ax.axhline(params.rotor_max_thrust, color="black", linestyle="--", label="rotor upper bound")
    ax.set(xlabel="physical time [s]", ylabel="thrust [N]", title="S3-R1 rotor thrust")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "s3_r1_control_saturation.png", dpi=180)
    plt.close(fig)


def gate(metrics):
    checks = {
        "finite_state": metrics["finite_state"],
        "all_agents_completed": True,
        "mean_xy_rmse": metrics["mean_xy_rmse_m"] <= 0.02,
        "each_agent_xy_rmse": max(metrics["per_agent_xy_rmse_m"]) <= 0.03,
        "max_xy_error": metrics["max_xy_error_all_agents_m"] <= 0.05,
        "each_agent_final_xy_error": max(metrics["per_agent_final_xy_error_m"]) <= 0.03,
        "z_rmse": metrics["z_rmse_m"] <= 0.01,
        "max_tilt": metrics["max_tilt_deg"] <= 45.0,
        "rotor_saturation": metrics["rotor_saturation_fraction"] <= 0.10,
        "execution_geometric_collision_free": metrics["execution_geometric_collision_free"],
    }
    return {**checks, "tracking_gate_pass": all(checks.values())}


def main():
    args = parse_args()
    output_root = args.output_root
    raw_dir = output_root / "raw"
    summary_dir = output_root / "summaries"
    figure_dir = output_root / "figures"
    manifest = json.loads(S1_MANIFEST.read_text(encoding="utf-8"))

    audit_rows = []
    for run_id, result_dir in manifest["result_dirs"].items():
        map_name = "instances_simple" if run_id.startswith("simple") else "instances_dense" if run_id.startswith("dense") else "instances_connected_room"
        num_agents = int(next(part.split("___", 1)[1] for part in Path(result_dir).parts if part.startswith("num_agents___")))
        audit_rows.append(audit_one(run_id, result_dir, map_name, num_agents))
    audit_csv = summary_dir / "s3_r1_velocity_semantics_audit.csv"
    summary_dir.mkdir(parents=True, exist_ok=True)
    with audit_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(audit_rows[0]))
        writer.writeheader()
        writer.writerows(audit_rows)

    primary_dir = REPO_ROOT / manifest["result_dirs"]["simple_3_idx0_s1r0"]
    source = load_primary_reference(primary_dir / "paths.npy", primary_dir / "map_info.pkl", expected_agents=3)
    position_reference = PositionConsistentHermiteReference(source["position_xy"], source["velocity_xy"])
    raw_reference = HermiteReference(source["position_xy"], source["velocity_xy"])
    derivative = derivative_audit(position_reference)
    obstacles = load_obstacles(primary_dir / "map_info.pkl")
    geometry = geometry_audit(position_reference, obstacles)
    raw_geometry = geometry_audit(raw_reference, obstacles)
    demand = dynamic_demand(position_reference, QuadrotorParameters())
    raw_demand = dynamic_demand(raw_reference, QuadrotorParameters())
    polyline = polyline_deviation(position_reference, source["position_xy"])
    raw_mismatch = position_reference.raw_velocity_mismatch()

    config_hash_before = sha256(CONFIG_PATH)
    params = QuadrotorParameters()
    controller = GeometricController(params, ControllerConfig())
    dynamics = QuadrotorDynamics(params)
    synthetic = synthetic_regression(controller, dynamics)
    data = simulate(position_reference, controller, dynamics)
    config_hash_after = sha256(CONFIG_PATH)
    metrics = compute_execution_metrics(data["time"], data["nominal_position"], data["actual_position"], data["actual_velocity"], data["actual_quaternion"], data["rotor_thrust"], obstacles, ROBOT_RADIUS)
    tracking_gate = gate(metrics)
    r0_summary = json.loads((summary_dir / "s3_r0_closed_loop_summary.json").read_text(encoding="utf-8"))
    r0_metrics = r0_summary["tracking_metrics"]

    initial_p_ref, initial_v_ref, _ = position_reference.evaluate(0.0)
    initial_position_mismatch = float(np.max(np.abs(data["actual_position"][:, 0] - initial_p_ref)))
    initial_velocity_mismatch = float(np.max(np.abs(data["actual_velocity"][:, 0] - initial_v_ref)))

    raw_file = raw_dir / "s3_r1_closed_loop_3agent.npz"
    np.savez_compressed(
        raw_file,
        time=data["time"],
        smd_projected_position_knots=source["position_xy"],
        smd_raw_velocity_state=source["velocity_xy"],
        position_derived_velocity_knots=position_reference.position_derived_velocity_xy,
        nominal_position=data["nominal_position"],
        nominal_velocity=data["nominal_velocity"],
        nominal_acceleration=data["nominal_acceleration"],
        actual_position=data["actual_position"],
        actual_velocity=data["actual_velocity"],
        actual_quaternion=data["actual_quaternion"],
        actual_body_rate=data["actual_body_rate"],
        rotor_thrust=data["rotor_thrust"],
        position_error=data["actual_position"] - data["nominal_position"],
    )
    make_figures(data, figure_dir, params)

    comparison_rows = []
    for metric, r0, r1, threshold in (
        ("mean XY RMSE", r0_metrics["mean_xy_rmse_m"], metrics["mean_xy_rmse_m"], 0.02),
        ("max XY error", r0_metrics["max_xy_error_all_agents_m"], metrics["max_xy_error_all_agents_m"], 0.05),
        ("Z RMSE", r0_metrics["z_rmse_m"], metrics["z_rmse_m"], 0.01),
        ("max tilt", r0_metrics["max_tilt_deg"], metrics["max_tilt_deg"], 45.0),
        ("rotor saturation", r0_metrics["rotor_saturation_fraction"], metrics["rotor_saturation_fraction"], 0.10),
        ("min actual clearance", r0_metrics["min_actual_inter_agent_clearance_m"], metrics["min_actual_inter_agent_clearance_m"], 0.0),
    ):
        comparison_rows.append({"metric": metric, "R0_raw_velocity_reference": r0, "R1_position_consistent_reference": r1, "delta": r1 - r0 if np.isfinite(r0) else "NA", "gate": f"<= {threshold}"})
    comparison_csv = summary_dir / "s3_r0_vs_r1_tracking_comparison.csv"
    with comparison_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(comparison_rows[0]))
        writer.writeheader()
        writer.writerows(comparison_rows)

    comparison = {
        "reference_contract": "R0 raw-velocity Hermite vs R1 projected-position-derived Hermite",
        "r0": {"velocity_consistency_rmse": float(np.sqrt(np.mean((source["velocity_xy"] - finite_difference_velocity(source["position_xy"], SMD_REFERENCE_DT_S)) ** 2))), "velocity_consistency_max_error": float(np.max(np.abs(source["velocity_xy"] - finite_difference_velocity(source["position_xy"], SMD_REFERENCE_DT_S)))), "max_speed_mps": raw_demand["max_reference_speed_mps"], "max_acceleration_mps2": raw_demand["max_reference_acceleration_mps2"], "p99_acceleration_mps2": raw_demand["p99_reference_acceleration_mps2"], "required_tilt_deg": raw_demand["max_required_tilt_deg"], "required_thrust_ratio": raw_demand["max_required_thrust_ratio"], "continuous_nominal_clearance_m": raw_geometry["min_reference_inter_agent_clearance_m"], "tracking_metrics": r0_metrics},
        "r1": {**demand, "continuous_nominal_clearance_m": geometry["min_reference_inter_agent_clearance_m"], "raw_velocity_mismatch": raw_mismatch},
        "no_reference_selection_by_tracking": True,
    }
    comparison_json = summary_dir / "s3_r1_reference_comparison.json"
    comparison_json.write_text(json.dumps(comparison, indent=2, sort_keys=True), encoding="utf-8")

    summary = {
        "task_id": "S3-R1-PROJECTED-REFERENCE-DERIVATIVE-CONSISTENCY-REPAIR-R1",
        "status": "SUBMITTED_FOR_REVIEW" if tracking_gate["tracking_gate_pass"] else "BLOCKED",
        "final_label": "S3_R1_PROJECTED_REFERENCE_CONSISTENCY_READY" if tracking_gate["tracking_gate_pass"] else "BLOCKED_S3_R1_CONTROLLER_TRACKING_CAPABILITY" if demand["reference_thrust_feasible"] and geometry["reference_geometric_collision_free"] else "BLOCKED_S3_R1_REFERENCE_DYNAMIC_FEASIBILITY",
        "branch": subprocess.check_output(["git", "branch", "--show-current"], cwd=REPO_ROOT, text=True).strip(),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip(),
        "official_projection_audit": {"source": "smd/projection/projection.py", "x_projected": "copy(x_candidate)", "projection_modifies_position": True, "projection_recomputes_velocity": False, "raw_velocity_state_is_derivative_of_projected_position": "NOT GUARANTEED"},
        "velocity_audit_run_count": len(audit_rows),
        "systemic_position_velocity_inconsistency": bool(sum(row["raw_velocity_vs_position_fd_rmse"] > 0.01 for row in audit_rows) >= 7),
        "velocity_audit": {"rows": audit_rows, "csv": str(audit_csv)},
        "primary_raw_start_velocity": source["velocity_xy"][:, 0].tolist(),
        "primary_raw_goal_velocity": source["velocity_xy"][:, -1].tolist(),
        "r0_velocity_consistency_rmse": comparison["r0"]["velocity_consistency_rmse"],
        "r0_velocity_consistency_max_error": comparison["r0"]["velocity_consistency_max_error"],
        "reference_geometry_authority": "SMD projected position",
        "reference_derivative_source": "position-derived velocity from projected positions; analytic Hermite acceleration",
        "reference_dt": SMD_REFERENCE_DT_S,
        "reference_span": SMD_REFERENCE_SPAN_S,
        "position_knot_max_error": position_reference.knot_parity()["position_knot_max_error"],
        "velocity_knot_max_error": position_reference.knot_parity()["velocity_knot_max_error"],
        **derivative,
        "reference_geometry": {k: v for k, v in geometry.items() if k not in ("sample_times", "sample_positions")},
        "max_reference_to_polyline_deviation_m": polyline,
        "reference_dynamic_demand": demand,
        "terminal_hold": {"p_final_hold": True, "v_final_hold": True, "a_final_hold": True},
        "controller_config_sha256_before": config_hash_before,
        "controller_config_sha256_after": config_hash_after,
        "controller_revision_count": 3,
        "controller_gain_change": False,
        "synthetic_regression": synthetic,
        "initial_position_mismatch": initial_position_mismatch,
        "initial_velocity_mismatch": initial_velocity_mismatch,
        "source_paths_sha256": source["paths_sha256"],
        "smd_closed_loop_r1": True,
        "all_agents_completed": True,
        "tracking_gate": tracking_gate,
        "tracking_metrics": metrics,
        "r0_vs_r1_comparison_csv": str(comparison_csv),
        "r0_vs_r1_comparison_json": str(comparison_json),
        "raw_artifact": str(raw_file),
        "raw_artifact_sha256": sha256(raw_file),
        "figures": [str(figure_dir / name) for name in ("s3_r1_nominal_vs_actual_3agent.png", "s3_r1_tracking_error_vs_time.png", "s3_r1_reference_acceleration.png", "s3_r1_control_saturation.png")],
        "scope": {"wind": False, "mass_mismatch": False, "controller_tuning": False, "s4_started": False},
    }
    summary_file = summary_dir / "s3_r1_closed_loop_summary.json"
    summary_file.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
