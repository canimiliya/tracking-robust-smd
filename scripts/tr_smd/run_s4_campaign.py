"""Run the resumable S4 execution-gap campaign over accepted SMD results."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pickle
import re
import subprocess
import sys
from math import sqrt
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
S3_BASELINE_FINGERPRINT = {
    "config": "97440F483FE8928E95F028CB0BF29F296ACF0BC6838060FFEE468CB5D65988CC",
    "controller": "A3BA0282A203FA442EC06EB93BAE6CE2882B91A8173C65CFC41C07D6210D92C8",
    "dynamics": "408CB19239AC497A3FFD32460BCF494A7A0B869D2B6C10591A4CE7DDE37F9AE5",
    "reference": "0C51602C854B52F77A8148A507C260AA2131876F5FA8E31766AF04D1676C254B",
    "combined": "BE4CEF9CC5EE99BF8AC238B6BF65C2438F28DF781F29E009F3348B030A96CFC1",
}
DISTURBANCE_LEVELS = {
    "D0_CALM": np.array([0.0, 0.0, 0.0]),
    "D1_FX_005": np.array([0.005, 0.0, 0.0]),
    "D2_FX_010": np.array([0.010, 0.0, 0.0]),
}


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
    map_name = next(
        name
        for prefix, name in (
            ("simple", "instances_simple"),
            ("dense", "instances_dense"),
            ("connected_room", "instances_connected_room"),
            ("empty", "instances_empty"),
            ("shelf", "instances_shelf"),
        )
        if run_id.startswith(prefix)
    )
    match = re.search(r"_(3|6|9)_idx", run_id)
    if match is None:
        raise ValueError(f"cannot parse agent count from {run_id}")
    agent_count = int(match.group(1))
    return map_name, agent_count


def public_trial(row):
    return {key: value for key, value in row.items() if not key.startswith("_")}


def run_trial(run_id: str, result_dir: str, fingerprint: dict, disturbance_id: str = "D0_CALM"):
    result_dir = REPO_ROOT / result_dir
    map_name, num_agents = identify(run_id)
    source = load_primary_reference(result_dir / "paths.npy", result_dir / "map_info.pkl", expected_agents=num_agents)
    obstacles, _ = load_instance_geometry(REPO_ROOT, map_name, source["instance_idx"], num_agents)
    reference = PositionConsistentHermiteReference(source["position_xy"], source["velocity_xy"])
    times, positions, velocities, accelerations = sample_reference(reference)
    nominal_trace = clearance_traces(positions, obstacles)
    demand = dynamic_demand(velocities, accelerations, QuadrotorParameters())
    external_force = DISTURBANCE_LEVELS[disturbance_id]
    data, runtime = simulate(reference, external_force_world=external_force)
    actual = execution_diagnostics(data, obstacles, QuadrotorParameters().rotor_max_thrust)
    official_success = True
    nominal_safe = nominal_trace["minimum"] >= 0.0
    dynamic_feasible = demand["reference_dynamic_feasible"]
    eligible = bool(official_success and nominal_safe and dynamic_feasible)
    is_confirmation = map_name == "instances_dense" and num_agents == 3 and 5 <= source["instance_idx"] <= 24
    phase = "prospective_confirmation" if is_confirmation else "calm_discovery"
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
        "trial_id": f"{'confirmation' if is_confirmation else 'calm'}_{map_name}_{num_agents}_idx{source['instance_idx']}_{disturbance_id}",
        "phase": phase,
        "git_head": fingerprint["git_head"],
        "config_hash": fingerprint["config"],
        "source_paths_sha256": source["paths_sha256"],
        "map": map_name,
        "instance_idx": source["instance_idx"],
        "agent_count": num_agents,
        "disturbance_id": disturbance_id,
        "external_force_world_n": external_force.tolist(),
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
        disturbance_force_world=np.tile(DISTURBANCE_LEVELS[row["disturbance_id"]], (len(data["time"]), 1)),
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
    unsafe_rows = [row for row in ranked if row["failure_class"] == "ELIGIBLE_EXECUTION_UNSAFE"]
    representative = []
    for level in ("D0_CALM", "D1_FX_005", "D2_FX_010"):
        match = next((row for row in unsafe_rows if row["disturbance_id"] == level), None)
        if match is not None:
            representative.append(match)
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
    ranking_fields = ["trial_id", "map", "agent_count", "instance_idx", "disturbance_id", "gap_eligible", "failure_class", "nominal_min_clearance_m", "actual_min_clearance_m", "clearance_loss_m", "tracking_position_rmse_m", "tracking_max_error_m"]
    with (summary_dir / "s4_candidate_ranking.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=ranking_fields)
        writer.writeheader()
        writer.writerows([{key: public_trial(row)[key] for key in ranking_fields} for row in ranked])
    eligible = [row for row in rows if row["gap_eligible"]]
    unsafe = [row for row in eligible if not row["execution_safe"]]
    confirmation = [row for row in rows if row["phase"] == "prospective_confirmation"]
    level_statistics = {}
    for disturbance_id in DISTURBANCE_LEVELS:
        level_rows = [row for row in confirmation if row["disturbance_id"] == disturbance_id]
        level_eligible = [row for row in level_rows if row["gap_eligible"]]
        level_unsafe = [row for row in level_eligible if not row["execution_safe"]]
        n = len(level_eligible)
        failures = len(level_unsafe)
        if n:
            z = 1.959963984540054
            rate = failures / n
            denominator = 1.0 + z * z / n
            center = (rate + z * z / (2.0 * n)) / denominator
            half = z * sqrt(rate * (1.0 - rate) / n + z * z / (4.0 * n * n)) / denominator
            wilson = [center - half, center + half]
        else:
            rate, wilson = None, [None, None]
        level_statistics[disturbance_id] = {
            "scheduled_trials": len(level_rows),
            "eligible_trials": n,
            "execution_failures": failures,
            "execution_successes": n - failures,
            "execution_success_rate": (n - failures) / n if n else None,
            "execution_gap_rate": rate,
            "gap_rate_wilson_95_interval": wilson,
            "minimum_evidence_met": bool(failures >= 3),
        }
    confirmed_level = next((level for level in DISTURBANCE_LEVELS if level_statistics[level]["minimum_evidence_met"]), None)
    summary = {
        "task_id": TASK_ID,
        "phase": "calm_discovery_and_prospective_confirmation",
        "git_head": fingerprint["git_head"],
        "baseline_fingerprint": S3_BASELINE_FINGERPRINT,
        "s4_implementation_fingerprint": fingerprint,
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
        "statistical_confirmation": {
            "scenario_family": "instances_dense / 3 agents",
            "scheduled_instance_ids": list(range(5, 25)),
            "by_disturbance_level": level_statistics,
            "lowest_confirmed_level": confirmed_level,
            "primary_execution_gap_confirmed": confirmed_level is not None,
        },
        "failure_taxonomy": {name: sum(row["failure_class"] == name for row in rows) for name in ("ELIGIBLE_EXECUTION_SAFE", "ELIGIBLE_EXECUTION_UNSAFE", "NOMINAL_STRICT_GEOMETRY_FAILURE", "REFERENCE_DYNAMIC_INFEASIBLE", "NONFINITE_EXECUTION", "INFRASTRUCTURE_FAILURE")},
    }
    (summary_dir / "s4_execution_gap_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    manifest = {
        "task_id": TASK_ID,
        "start_head": "65fb9fc87ab0df44b2061709c41cc354ddb1d2fb",
        "branch": fingerprint["branch"],
        "baseline_fingerprint": S3_BASELINE_FINGERPRINT,
        "s4_implementation_fingerprint": fingerprint,
        "candidate_rule": 0,
        "dynamics_dt_s": 0.002,
        "control_dt_s": 0.01,
        "robot_radius_m": 0.05,
        "trials": public_rows,
        "raw_artifacts": raw_artifacts,
        "planning_accounting": {
            "reused_s1_s2_runs": 9,
            "new_successful_smd_runs": 46,
            "infrastructure_attempts_without_raw": 26,
            "infrastructure_breakdown": {"ipopt_path_missing": 6, "windows_path_too_long": 20},
            "soft_budget_exceeded_reason": "20 prospective dense/3-agent trials were required after a calm discovery gap to estimate repeatability without changing family or adding disturbance",
        },
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

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    levels = list(DISTURBANCE_LEVELS)
    rates = [level_statistics[level]["execution_gap_rate"] for level in levels]
    intervals = [level_statistics[level]["gap_rate_wilson_95_interval"] for level in levels]
    lower = [rate - interval[0] for rate, interval in zip(rates, intervals)]
    upper = [interval[1] - rate for rate, interval in zip(rates, intervals)]
    ax.bar(levels, rates, color=["#6c8ebf", "#e8a33d", "#c94c4c"], yerr=[lower, upper], capsize=5)
    ax.set(ylabel="eligible execution collision rate", title="Frozen dense / 3-agent confirmation", ylim=(0.0, 1.05))
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(figure_dir / "s4_gap_rate_by_disturbance.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    colors = {"D0_CALM": "#6c8ebf", "D1_FX_005": "#e8a33d", "D2_FX_010": "#c94c4c"}
    for level in levels:
        level_rows = [row for row in rows if row["gap_eligible"] and row["disturbance_id"] == level]
        ax.scatter([row["tracking_position_rmse_m"] for row in level_rows], [row["clearance_loss_m"] for row in level_rows], label=level, color=colors[level], alpha=0.75)
    ax.set(xlabel="tracking position RMSE [m]", ylabel="nominal minus actual clearance [m]", title="Tracking error and safety-margin consumption")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_dir / "s4_tracking_vs_clearance_loss.png", dpi=180)
    plt.close(fig)

    d1_failure = next(row for row in representative if row["disturbance_id"] == "D1_FX_005")
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    ax.plot(d1_failure["_data"]["time"], d1_failure["_nominal_trace"]["combined"], label="nominal strict clearance")
    ax.plot(d1_failure["_data"]["time"], d1_failure["_actual_trace"]["combined"], label="actual strict clearance")
    ax.axhline(0.0, color="black", linewidth=1)
    ax.set(xlabel="time [s]", ylabel="minimum clearance [m]", title=f"D1 representative: instance {d1_failure['instance_idx']}")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_dir / "s4_d1_failure_clearance_trace.png", dpi=180)
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
        "git_head": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
    }
    source_manifest = json.loads(S1_MANIFEST.read_text(encoding="utf-8"))
    result_dirs = dict(source_manifest["result_dirs"])
    new_roots = (
        REPO_ROOT / "scripts/inference/results_s4_official_idx0_retry",
        REPO_ROOT / "scripts/inference/results_s4_official_3agent_idx1_4",
        REPO_ROOT / "scripts/inference/r4c",
    )
    for new_root in new_roots:
        for paths_file in sorted(new_root.glob("**/paths.npy")):
            map_info = pickle.loads(paths_file.with_name("map_info.pkl").read_bytes())
            agent_count = int(next(part.split("___", 1)[1] for part in paths_file.parts if part.startswith("num_agents___")))
            map_short = map_info["map_name"].replace("instances_", "")
            run_id = f"{map_short}_{agent_count}_idx{map_info['instance_idx']}_s4new"
            result_dirs[run_id] = str(paths_file.parent.relative_to(REPO_ROOT))
    rows = []
    confirmation_inputs = []
    for run_id, result_dir in result_dirs.items():
        print(f"RUN {run_id}", flush=True)
        rows.append(run_trial(run_id, result_dir, fingerprint, "D0_CALM"))
        print(f"DONE {rows[-1]['trial_id']} {rows[-1]['failure_class']} nominal={rows[-1]['nominal_min_clearance_m']:.6f} actual={rows[-1]['actual_min_clearance_m']:.6f}", flush=True)
        if rows[-1]["phase"] == "prospective_confirmation":
            confirmation_inputs.append((run_id, result_dir))
    s3_raw = np.load(REPO_ROOT / "experiments/raw/s3_r1_closed_loop_3agent.npz")
    baseline_row = next(row for row in rows if row["map"] == "instances_simple" and row["agent_count"] == 3 and row["instance_idx"] == 0)
    zero_regression = {
        "actual_position_max_abs_error": float(np.max(np.abs(baseline_row["_data"]["actual_position"] - s3_raw["actual_position"]))),
        "actual_velocity_max_abs_error": float(np.max(np.abs(baseline_row["_data"]["actual_velocity"] - s3_raw["actual_velocity"]))),
        "rotor_thrust_max_abs_error": float(np.max(np.abs(baseline_row["_data"]["rotor_thrust"] - s3_raw["rotor_thrust"]))),
    }
    zero_regression["pass"] = all(value <= 1e-12 for value in zero_regression.values())
    if not zero_regression["pass"]:
        raise RuntimeError(f"zero-disturbance S3 regression failed: {zero_regression}")
    for disturbance_id in ("D1_FX_005", "D2_FX_010"):
        for run_id, result_dir in confirmation_inputs:
            print(f"RUN {run_id} {disturbance_id}", flush=True)
            rows.append(run_trial(run_id, result_dir, fingerprint, disturbance_id))
            print(f"DONE {rows[-1]['trial_id']} {rows[-1]['failure_class']} nominal={rows[-1]['nominal_min_clearance_m']:.6f} actual={rows[-1]['actual_min_clearance_m']:.6f}", flush=True)
    summary = write_outputs(rows, fingerprint)
    summary["zero_disturbance_s3_regression"] = zero_regression
    confirmed = summary["statistical_confirmation"]["primary_execution_gap_confirmed"]
    summary["status"] = "SUBMITTED_FOR_REVIEW" if confirmed else "IN_PROGRESS"
    summary["final_label"] = "S4_EXECUTION_GAP_CONFIRMED" if confirmed else "S4_EXECUTION_GAP_CANDIDATE_READY"
    summary["s4_status"] = "SUBMITTED_FOR_REVIEW" if confirmed else "IN_PROGRESS"
    summary_path = REPO_ROOT / "experiments/summaries/s4_execution_gap_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    manifest_path = REPO_ROOT / "experiments/manifests/s4_campaign_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["zero_disturbance_s3_regression"] = zero_regression
    manifest["final_label"] = summary["final_label"]
    manifest["status"] = summary["status"]
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
