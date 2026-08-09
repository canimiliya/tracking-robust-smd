"""Run the resumable S4 execution-gap campaign over accepted SMD results."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tr_smd.dynamics.quadrotor import QuadrotorParameters
from tr_smd.evaluation.execution_gap import (
    clearance_traces,
    dynamic_demand,
    execution_diagnostics,
    load_instance_geometry,
    sample_reference,
    sha256_file,
    simulate,
)
from tr_smd.tracking.smd_reference import PositionConsistentHermiteReference, load_primary_reference

TASK_ID = "S4-R0-AUTONOMOUS-EXECUTION-GAP-DISCOVERY-CAMPAIGN-R1"
S1_MANIFEST = REPO_ROOT / "experiments/manifests/s1_r1_core_reproduction_manifest.json"
CONFIG_PATH = REPO_ROOT / "configs/tr_smd/closed_loop_quadrotor.yaml"
DYNAMICS_PATH = REPO_ROOT / "tr_smd/dynamics/quadrotor.py"
CONTROLLER_PATH = REPO_ROOT / "tr_smd/control/geometric_controller.py"
REFERENCE_PATH = REPO_ROOT / "tr_smd/tracking/smd_reference.py"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("calm-existing",), default="calm-existing")
    return parser.parse_args()


def git(*args):
    return subprocess.check_output(["git", *args], cwd=REPO_ROOT, text=True).strip()


def combined_hash(paths):
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.relative_to(REPO_ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest().upper()


def identify(run_id: str):
    map_name = "instances_simple" if run_id.startswith("simple") else "instances_dense" if run_id.startswith("dense") else "instances_connected_room"
    match = re.search(r"_(3|6|9)_idx", run_id)
    if match is None:
        raise ValueError(f"cannot parse agent count from {run_id}")
    agent_count = int(match.group(1))
    return map_name, agent_count


def public_trial(row):
    return {key: value for key, value in row.items() if not key.startswith("_")}


def run_trial(run_id: str, result_dir: str, fingerprint: dict):
    result_dir = REPO_ROOT / result_dir
    map_name, num_agents = identify(run_id)
    source = load_primary_reference(result_dir / "paths.npy", result_dir / "map_info.pkl", expected_agents=num_agents)
    obstacles, _ = load_instance_geometry(REPO_ROOT, map_name, source["instance_idx"], num_agents)
    reference = PositionConsistentHermiteReference(source["position_xy"], source["velocity_xy"])
    times, positions, velocities, accelerations = sample_reference(reference)
    nominal_trace = clearance_traces(positions, obstacles)
    demand = dynamic_demand(velocities, accelerations, QuadrotorParameters())
    data, runtime = simulate(reference)
    actual = execution_diagnostics(data, obstacles, QuadrotorParameters().rotor_max_thrust)
    official_success = True
    nominal_safe = nominal_trace["minimum"] >= 0.0
    dynamic_feasible = demand["reference_dynamic_feasible"]
    eligible = bool(official_success and nominal_safe and dynamic_feasible)
    if not actual["finite_execution"]:
        failure_class = "NONFINITE_EXECUTION"
    elif not nominal_safe:
        failure_class = "NOMINAL_STRICT_GEOMETRY_FAILURE"
    elif not dynamic_feasible:
        failure_class = "REFERENCE_DYNAMIC_INFEASIBLE"
    elif actual["execution_safe"]:
        failure_class = "ELIGIBLE_EXECUTION_SAFE"
    else:
        failure_class = "ELIGIBLE_EXECUTION_UNSAFE"
    row = {
        "trial_id": f"calm_{map_name}_{num_agents}_idx{source['instance_idx']}",
        "phase": "calm_existing",
        "git_head": git("rev-parse", "HEAD"),
        "config_hash": fingerprint["config"],
        "source_paths_sha256": source["paths_sha256"],
        "map": map_name,
        "instance_idx": source["instance_idx"],
        "agent_count": num_agents,
        "disturbance_id": "D0_CALM",
        "controller_hash": fingerprint["controller"],
        "dynamics_hash": fingerprint["dynamics"],
        "reference_hash": fingerprint["reference"],
        "official_planning_success": official_success,
        "nominal_safe": nominal_safe,
        "dynamic_feasible": dynamic_feasible,
        "gap_eligible": eligible,
        "execution_safe": actual["execution_safe"] if eligible else False,
        "nominal_min_clearance_m": nominal_trace["minimum"],
        "actual_min_clearance_m": actual["actual_min_clearance_m"],
        "clearance_loss_m": nominal_trace["minimum"] - actual["actual_min_clearance_m"],
        "tracking_position_rmse_m": actual["tracking_position_rmse_m"],
        "tracking_max_error_m": actual["tracking_max_error_m"],
        "max_reference_speed_mps": demand["max_reference_speed_mps"],
        "max_reference_acceleration_mps2": demand["max_reference_acceleration_mps2"],
        "p99_reference_acceleration_mps2": demand["p99_reference_acceleration_mps2"],
        "max_required_thrust_ratio": demand["max_required_thrust_ratio"],
        "max_required_tilt_deg": demand["max_required_tilt_deg"],
        "max_actual_tilt_deg": actual["max_actual_tilt_deg"],
        "rotor_saturation_fraction": actual["rotor_saturation_fraction"],
        "nominal_closest_approach_time_s": float(times[nominal_trace["minimum_index"]]),
        "actual_closest_approach_time_s": actual["actual_closest_approach_time_s"],
        "collision_type": actual["actual_collision_type"],
        "agent_agent_collision": actual["agent_agent_collision"],
        "agent_obstacle_collision": actual["agent_obstacle_collision"],
        "runtime_s": runtime,
        "failure_class": failure_class,
        "_data": data,
        "_nominal_trace": nominal_trace,
        "_actual_trace": actual["clearance_trace"],
        "_source": source,
    }
    return row


def save_candidate_raw(row, raw_dir: Path):
    raw_dir.mkdir(parents=True, exist_ok=True)
    path = raw_dir / f"{row['trial_id']}.npz"
    data = row["_data"]
    source = row["_source"]
    np.savez_compressed(
        path,
        time=data["time"],
        smd_projected_position_knots=source["position_xy"],
        smd_raw_velocity_state=source["velocity_xy"],
        nominal_position=data["nominal_position"],
        nominal_velocity=data["nominal_velocity"],
        nominal_acceleration=data["nominal_acceleration"],
        actual_position=data["actual_position"],
        actual_velocity=data["actual_velocity"],
        actual_quaternion=data["actual_quaternion"],
        actual_body_rate=data["actual_body_rate"],
        rotor_thrust=data["rotor_thrust"],
        tracking_error=data["actual_position"] - data["nominal_position"],
        nominal_clearance_trace=row["_nominal_trace"]["combined"],
        actual_clearance_trace=row["_actual_trace"]["combined"],
        disturbance_force_world=np.zeros((len(data["time"]), 3)),
    )
    return str(path.relative_to(REPO_ROOT)), sha256_file(path)


def write_outputs(rows, fingerprint):
    manifest_dir = REPO_ROOT / "experiments/manifests"
    summary_dir = REPO_ROOT / "experiments/summaries"
    figure_dir = REPO_ROOT / "experiments/figures"
    raw_dir = REPO_ROOT / "experiments/raw/s4"
    for directory in (manifest_dir, summary_dir, figure_dir):
        directory.mkdir(parents=True, exist_ok=True)
    ranked = sorted(rows, key=lambda r: (not r["gap_eligible"], r["actual_min_clearance_m"], -r["clearance_loss_m"]))
    representative = [row for row in ranked if row["failure_class"] == "ELIGIBLE_EXECUTION_UNSAFE"][:3]
    if not representative:
        representative = ranked[:1]
    raw_artifacts = []
    for row in representative:
        path, digest = save_candidate_raw(row, raw_dir)
        raw_artifacts.append({"trial_id": row["trial_id"], "path": path, "sha256": digest})
    public_rows = [public_trial(row) for row in rows]
    csv_path = summary_dir / "s4_execution_gap_trials.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(public_rows[0]))
        writer.writeheader()
        writer.writerows(public_rows)
    ranking_fields = ["trial_id", "map", "agent_count", "instance_idx", "gap_eligible", "failure_class", "nominal_min_clearance_m", "actual_min_clearance_m", "clearance_loss_m", "tracking_position_rmse_m", "tracking_max_error_m"]
    with (summary_dir / "s4_candidate_ranking.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=ranking_fields)
        writer.writeheader()
        writer.writerows([{key: public_trial(row)[key] for key in ranking_fields} for row in ranked])
    eligible = [row for row in rows if row["gap_eligible"]]
    unsafe = [row for row in eligible if not row["execution_safe"]]
    summary = {
        "task_id": TASK_ID,
        "phase": "calm_existing",
        "git_head": git("rev-parse", "HEAD"),
        "baseline_fingerprint": fingerprint,
        "trial_count": len(rows),
        "official_planning_success_count": sum(row["official_planning_success"] for row in rows),
        "strict_nominal_safe_count": sum(row["nominal_safe"] for row in rows),
        "reference_dynamic_feasible_count": sum(row["dynamic_feasible"] for row in rows),
        "gap_eligible_count": len(eligible),
        "execution_safe_count": len(eligible) - len(unsafe),
        "execution_unsafe_count": len(unsafe),
        "execution_gap_count": len(unsafe),
        "agent_agent_collision_count": sum(row["agent_agent_collision"] for row in unsafe),
        "agent_obstacle_collision_count": sum(row["agent_obstacle_collision"] for row in unsafe),
        "max_tracking_rmse_m": max(row["tracking_position_rmse_m"] for row in rows),
        "max_tracking_error_m": max(row["tracking_max_error_m"] for row in rows),
        "min_nominal_clearance_m": min(row["nominal_min_clearance_m"] for row in rows),
        "min_actual_clearance_m": min(row["actual_min_clearance_m"] for row in rows),
        "max_clearance_loss_m": max(row["clearance_loss_m"] for row in rows),
        "top_candidate": public_trial(ranked[0]),
        "raw_artifacts": raw_artifacts,
        "failure_taxonomy": {name: sum(row["failure_class"] == name for row in rows) for name in ("ELIGIBLE_EXECUTION_SAFE", "ELIGIBLE_EXECUTION_UNSAFE", "NOMINAL_STRICT_GEOMETRY_FAILURE", "REFERENCE_DYNAMIC_INFEASIBLE", "NONFINITE_EXECUTION", "INFRASTRUCTURE_FAILURE")},
    }
    (summary_dir / "s4_execution_gap_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    manifest = {
        "task_id": TASK_ID,
        "start_head": "65fb9fc87ab0df44b2061709c41cc354ddb1d2fb",
        "branch": git("branch", "--show-current"),
        "baseline_fingerprint": fingerprint,
        "candidate_rule": 0,
        "dynamics_dt_s": 0.002,
        "control_dt_s": 0.01,
        "robot_radius_m": 0.05,
        "trials": public_rows,
        "raw_artifacts": raw_artifacts,
    }
    (manifest_dir / "s4_campaign_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    for row in rows:
        ax.scatter(row["nominal_min_clearance_m"], row["actual_min_clearance_m"], s=25 + 5 * row["agent_count"], label=f"{row['map'].replace('instances_', '')}-{row['agent_count']}")
    bounds = [min(summary["min_nominal_clearance_m"], summary["min_actual_clearance_m"]), max(max(row["nominal_min_clearance_m"] for row in rows), max(row["actual_min_clearance_m"] for row in rows))]
    ax.plot(bounds, bounds, "k--", linewidth=1, label="no clearance loss")
    ax.axhline(0.0, color="red", linewidth=1)
    ax.set(xlabel="nominal minimum clearance [m]", ylabel="actual minimum clearance [m]", title="S4 calm execution-gap screening")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(figure_dir / "s4_calm_clearance_transfer.png", dpi=180)
    plt.close(fig)
    return summary


def main():
    parse_args()
    fingerprint = {
        "config": sha256_file(CONFIG_PATH),
        "controller": sha256_file(CONTROLLER_PATH),
        "dynamics": sha256_file(DYNAMICS_PATH),
        "reference": sha256_file(REFERENCE_PATH),
        "combined": combined_hash([CONFIG_PATH, CONTROLLER_PATH, DYNAMICS_PATH, REFERENCE_PATH]),
    }
    source_manifest = json.loads(S1_MANIFEST.read_text(encoding="utf-8"))
    rows = []
    for run_id, result_dir in source_manifest["result_dirs"].items():
        print(f"RUN {run_id}", flush=True)
        rows.append(run_trial(run_id, result_dir, fingerprint))
        print(f"DONE {rows[-1]['trial_id']} {rows[-1]['failure_class']} nominal={rows[-1]['nominal_min_clearance_m']:.6f} actual={rows[-1]['actual_min_clearance_m']:.6f}", flush=True)
    print(json.dumps(write_outputs(rows, fingerprint), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
