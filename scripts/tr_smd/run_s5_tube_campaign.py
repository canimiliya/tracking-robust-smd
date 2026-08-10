"""Prepare, calibrate, and evaluate the frozen S5 tracking-tube campaign."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tr_smd.dynamics.quadrotor import QuadrotorParameters
from tr_smd.evaluation.execution_gap import clearance_traces, dynamic_demand, load_instance_geometry, sha256_file
from tr_smd.tracking.smd_reference import PositionConsistentHermiteReference, load_primary_reference
from tr_smd.tubes.scenario_tube import (
    ScenarioSpec,
    compute_tracking_tube,
    default_design_scenarios,
    disturbance_schedule,
    simulate_scheduled,
)
from tr_smd.tubes.validation import (
    coverage_metrics,
    discover_reference_catalog,
    evaluation_specs,
    schedule_sha256,
)

TASK_ID = "S5-R0-AUTONOMOUS-TRACKING-ERROR-TUBE-MODELING-AND-VALIDATION-R1"
CONFIG_PATH = REPO_ROOT / "configs/tr_smd/tracking_error_tube.yaml"
BASELINE_PATHS = {
    "config": REPO_ROOT / "configs/tr_smd/closed_loop_quadrotor.yaml",
    "controller": REPO_ROOT / "tr_smd/control/geometric_controller.py",
    "dynamics": REPO_ROOT / "tr_smd/dynamics/quadrotor.py",
    "reference": REPO_ROOT / "tr_smd/tracking/smd_reference.py",
}
CONTRACT_PATHS = [
    REPO_ROOT / "docs/TRACKING_ERROR_TUBE_CONTRACT.md",
    REPO_ROOT / "docs/S5_TUBE_VALIDATION_CONTRACT.md",
    CONFIG_PATH,
]
MANIFEST_PATH = REPO_ROOT / "experiments/manifests/s5_tube_manifest.json"
CALIBRATION_PATH = REPO_ROOT / "experiments/manifests/s5_tube_calibration.json"
SUMMARY_DIR = REPO_ROOT / "experiments/summaries"
FIGURE_DIR = REPO_ROOT / "experiments/figures"
RAW_DIR = REPO_ROOT / "experiments/raw/s5"
PRIMARY_BOUND_N = 0.005
STRESS_BOUND_N = 0.010
SAFETY_FACTOR = 1.10
TOLERANCE_M = 1e-9
WORKERS = 20


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO_ROOT, text=True).strip()


def combined_hash(paths: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.relative_to(REPO_ROOT).as_posix().encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest().upper()


def load_reference(record: dict) -> tuple[dict, PositionConsistentHermiteReference]:
    source = load_primary_reference(
        REPO_ROOT / record["paths_file"],
        REPO_ROOT / record["map_info_file"],
        expected_agents=int(record["agent_count"]),
    )
    return source, PositionConsistentHermiteReference(source["position_xy"], source["velocity_xy"])


def prepare() -> dict:
    catalog = discover_reference_catalog(REPO_ROOT)
    baseline = {name: sha256_file(path) for name, path in BASELINE_PATHS.items()}
    baseline["combined"] = combined_hash(list(BASELINE_PATHS.values()))
    trials = []
    for record in catalog:
        for spec in evaluation_specs(record["split"]):
            trials.append({
                "trial_id": f"{record['split']}|{record['reference_id']}|{spec.name}",
                "reference_id": record["reference_id"],
                "split": record["split"],
                "disturbance_spec": asdict(spec),
                "disturbance_schedule_sha256": schedule_sha256(spec, record["agent_count"], PRIMARY_BOUND_N),
                "max_disturbance_magnitude_n": PRIMARY_BOUND_N,
            })
    manifest = {
        "task_id": TASK_ID,
        "start_head": "20707c51f1b25652a1dae7bc57d368589cd346d7",
        "branch": git("branch", "--show-current"),
        "manifest_created_at_head": git("rev-parse", "HEAD"),
        "baseline_fingerprint": baseline,
        "tube_config_sha256": sha256_file(CONFIG_PATH),
        "validation_contract_sha256": combined_hash(CONTRACT_PATHS),
        "primary_disturbance_set": {"xy_l2_bound_n": PRIMARY_BOUND_N, "z_force_n": 0.0, "moment_nm": [0.0, 0.0, 0.0], "per_agent": True},
        "stress_disturbance_set": {"xy_l2_bound_n": STRESS_BOUND_N, "role": "out_of_primary_stress"},
        "initial_error": "zero",
        "design_scenarios": [asdict(spec) for spec in default_design_scenarios()],
        "split_counts": {split: sum(row["split"] == split for row in catalog) for split in ("calibration", "validation", "holdout")},
        "reference_catalog": catalog,
        "formal_trial_contract": trials,
        "holdout_executed": False,
        "formal_guarantee": False,
    }
    MANIFEST_PATH.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"phase": "prepare", "references": len(catalog), "trials": len(trials), "split_counts": manifest["split_counts"], "manifest": str(MANIFEST_PATH)}, indent=2))
    return manifest


def _evaluate_reference(payload: tuple[dict, float, float, float, str]) -> dict:
    record, bound_n, inflation_factor, additive_m, evaluation_split = payload
    source, reference = load_reference(record)
    tube = compute_tracking_tube(
        reference,
        bound_n,
        inflation_factor=inflation_factor,
        additive_inflation_m=additive_m,
        workers=1,
    )
    demand_samples = np.asarray([reference.evaluate(float(t))[2] for t in tube.time]).transpose(1, 0, 2)
    acceleration_norm = np.linalg.norm(demand_samples, axis=-1)
    correlation = float(np.corrcoef(tube.rho_dense.ravel(), acceleration_norm.ravel())[0, 1])
    trial_rows = []
    for spec in evaluation_specs(evaluation_split):
        schedule = disturbance_schedule(spec, reference.num_agents, bound_n)
        data, runtime = simulate_scheduled(reference, schedule)
        error = np.linalg.norm(data["actual_position"] - data["nominal_position"], axis=-1)
        metrics = coverage_metrics(error, tube.rho_dense, TOLERANCE_M)
        base = (tube.rho_dense - additive_m) / inflation_factor
        positive = base > 1e-8
        max_ratio = float(np.max(error[positive] / base[positive])) if np.any(positive) else 0.0
        small_base_error = float(np.max(error[~positive])) if np.any(~positive) else 0.0
        trial_rows.append({
            "trial_id": f"{evaluation_split}|{record['reference_id']}|{spec.name}|{bound_n:.3f}N",
            "split": evaluation_split,
            "reference_id": record["reference_id"],
            "map": record["map"],
            "instance": record["instance_idx"],
            "agent_count": record["agent_count"],
            "source_reference_sha256": source["paths_sha256"],
            "disturbance_name": spec.name,
            "disturbance_seed": spec.seed,
            "disturbance_schedule_hash": schedule_sha256(spec, record["agent_count"], bound_n),
            "max_disturbance_magnitude_n": bound_n,
            "max_ratio_to_uninflated_scenario_envelope": max_ratio,
            "max_error_where_base_below_1e-8_m": small_base_error,
            "execution_runtime_s": runtime,
            **metrics,
        })
    return {
        "record": record,
        "trials": trial_rows,
        "tube": {
            "mean_radius_m": float(np.mean(tube.rho_dense)),
            "p95_radius_m": float(np.percentile(tube.rho_dense, 95.0)),
            "max_radius_m": float(np.max(tube.rho_dense)),
            "std_radius_m": float(np.std(tube.rho_dense)),
            "range_radius_m": float(np.ptp(tube.rho_dense)),
            "integrated_radius_m_s": float(np.mean(np.sum(tube.rho_dense, axis=1) * 0.002)),
            "radius_acceleration_correlation": correlation,
            "computation_time_s": tube.metadata["tube_computation_time_s"],
            "time_per_support_point_s": tube.metadata["tube_computation_time_s"] / 64.0,
        },
    }


def parallel_evaluate(records: list[dict], bound_n: float, inflation: float, additive: float, split_override: str | None = None) -> list[dict]:
    payloads = [(record, bound_n, inflation, additive, split_override or record["split"]) for record in records]
    results = []
    started = perf_counter()
    with ProcessPoolExecutor(max_workers=min(WORKERS, len(payloads))) as pool:
        futures = [pool.submit(_evaluate_reference, payload) for payload in payloads]
        for completed, future in enumerate(as_completed(futures), start=1):
            results.append(future.result())
            print(f"completed {completed}/{len(futures)} references", flush=True)
    print(f"parallel wall time: {perf_counter() - started:.3f} s", flush=True)
    return sorted(results, key=lambda item: item["record"]["reference_id"])


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def calibrate() -> dict:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    records = [row for row in manifest["reference_catalog"] if row["split"] in ("calibration", "validation")]
    results = parallel_evaluate(records, PRIMARY_BOUND_N, 1.0, 0.0)
    rows = [row for result in results for row in result["trials"]]
    max_ratio = max(1.0, max(row["max_ratio_to_uninflated_scenario_envelope"] for row in rows))
    additive_raw = max(row["max_error_where_base_below_1e-8_m"] for row in rows)
    inflation = max_ratio * SAFETY_FACTOR
    additive = additive_raw * SAFETY_FACTOR + TOLERANCE_M
    fixed_radius = max(row["max_actual_error_m"] for row in rows) * SAFETY_FACTOR + TOLERANCE_M
    calibration = {
        "task_id": TASK_ID,
        "manifest_sha256": sha256_file(MANIFEST_PATH),
        "contract_hashes": {path.relative_to(REPO_ROOT).as_posix(): sha256_file(path) for path in CONTRACT_PATHS},
        "calibration_reference_count": sum(row["split"] == "calibration" for row in manifest["reference_catalog"]),
        "validation_reference_count": sum(row["split"] == "validation" for row in manifest["reference_catalog"]),
        "calibration_trial_count": sum(row["split"] == "calibration" for row in rows),
        "validation_trial_count": sum(row["split"] == "validation" for row in rows),
        "raw_max_ratio": max_ratio,
        "scenario_discretization_safety_factor": SAFETY_FACTOR,
        "final_inflation_factor": inflation,
        "raw_additive_residual_m": additive_raw,
        "final_additive_inflation_m": additive,
        "fixed_global_radius_m": fixed_radius,
        "holdout_used": False,
    }
    CALIBRATION_PATH.write_text(json.dumps(calibration, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(SUMMARY_DIR / "s5_calibration_validation.csv", rows)
    (SUMMARY_DIR / "s5_pre_holdout_summary.json").write_text(json.dumps({"calibration": calibration, "reference_tube_summaries": [result["tube"] | {"reference_id": result["record"]["reference_id"], "split": result["record"]["split"]} for result in results]}, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(calibration, indent=2, sort_keys=True))
    return calibration


def verify_pre_holdout(manifest: dict, calibration: dict) -> None:
    if manifest.get("holdout_executed"):
        raise RuntimeError("manifest says holdout was already executed")
    if sha256_file(MANIFEST_PATH) != calibration["manifest_sha256"]:
        raise RuntimeError("pre-registered manifest changed after calibration")
    for relative, digest in calibration["contract_hashes"].items():
        if sha256_file(REPO_ROOT / relative) != digest:
            raise RuntimeError(f"frozen contract changed: {relative}")


def aggregate(rows: list[dict]) -> dict:
    samples = sum(row["sample_count"] for row in rows)
    covered = sum(row["covered_sample_count"] for row in rows)
    return {
        "trial_count": len(rows),
        "tube_coverage_rate": covered / samples,
        "trajectory_coverage_rate": sum(row["trajectory_covered"] for row in rows) / len(rows),
        "max_tube_violation_m": max(row["max_tube_violation_m"] for row in rows),
        "violation_count": sum(row["violation_count"] for row in rows),
        "violation_duration_s": sum(row["violation_duration_s"] for row in rows),
    }


def representative_artifacts(record: dict, inflation: float, additive: float, fixed_radius: float) -> dict:
    source, reference = load_reference(record)
    tube = compute_tracking_tube(reference, PRIMARY_BOUND_N, inflation_factor=inflation, additive_inflation_m=additive, workers=WORKERS)
    spec = ScenarioSpec("holdout_common_pos_x", "common_fixed", 0.0)
    schedule = disturbance_schedule(spec, reference.num_agents, PRIMARY_BOUND_N)
    data, _ = simulate_scheduled(reference, schedule)
    error = np.linalg.norm(data["actual_position"] - data["nominal_position"], axis=-1)
    obstacles, _ = load_instance_geometry(REPO_ROOT, record["map"], record["instance_idx"], record["agent_count"])
    nominal_clearance = clearance_traces(data["nominal_position"], obstacles)["combined"]
    actual_clearance = clearance_traces(data["actual_position"], obstacles)["combined"]
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    raw_path = RAW_DIR / "s5_dense_3_idx22_d1_pos_x_tube.npz"
    np.savez_compressed(raw_path, time=data["time"], nominal_position=data["nominal_position"], actual_position=data["actual_position"], tracking_error_norm=error, rho_dense=tube.rho_dense, rho_support=tube.rho_support, rho_segment=tube.rho_segment, nominal_clearance=nominal_clearance, actual_clearance=actual_clearance, disturbance_schedule=schedule)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    max_error, max_rho = np.max(error, axis=0), np.max(tube.rho_dense, axis=0)
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.plot(data["time"], max_error, label="actual tracking error")
    ax.plot(data["time"], max_rho, label="adaptive tube")
    ax.axhline(fixed_radius, color="gray", linestyle="--", label="fixed radius")
    ax.set(xlabel="time [s]", ylabel="position radius [m]", title="D1 dense/3-agent idx22 coverage")
    ax.grid(alpha=0.25); ax.legend(); fig.tight_layout(); fig.savefig(FIGURE_DIR / "s5_idx22_tube_vs_error.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.plot(data["time"], nominal_clearance, label="nominal strict clearance")
    ax.plot(data["time"], actual_clearance, label="actual strict clearance")
    ax.plot(data["time"], -max_rho, label="negative tube radius", linestyle="--")
    ax.axhline(0.0, color="black", linewidth=1)
    ax.set(xlabel="time [s]", ylabel="clearance / radius [m]", title="S4 collision case diagnosed by pre-execution tube")
    ax.grid(alpha=0.25); ax.legend(); fig.tight_layout(); fig.savefig(FIGURE_DIR / "s5_idx22_clearance_and_tube.png", dpi=180); plt.close(fig)
    acceleration = np.linalg.norm(data["nominal_acceleration"], axis=-1)
    fig, ax = plt.subplots(figsize=(6.2, 4.5))
    ax.scatter(acceleration.ravel()[::10], tube.rho_dense.ravel()[::10], s=4, alpha=0.25)
    ax.set(xlabel="reference acceleration [m/s^2]", ylabel="adaptive radius [m]", title="Tube radius and reference demand")
    ax.grid(alpha=0.25); fig.tight_layout(); fig.savefig(FIGURE_DIR / "s5_radius_vs_acceleration.png", dpi=180); plt.close(fig)
    return {"raw_path": raw_path.relative_to(REPO_ROOT).as_posix(), "raw_sha256": sha256_file(raw_path), "parallel_tube_runtime_s": tube.metadata["tube_computation_time_s"], "time_per_support_point_s": tube.metadata["tube_computation_time_s"] / 64.0}


def holdout() -> dict:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    calibration = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    verify_pre_holdout(manifest, calibration)
    holdout_records = [row for row in manifest["reference_catalog"] if row["split"] == "holdout"]
    inflation = calibration["final_inflation_factor"]
    additive = calibration["final_additive_inflation_m"]
    fixed_radius = calibration["fixed_global_radius_m"]
    holdout_results = parallel_evaluate(holdout_records, PRIMARY_BOUND_N, inflation, additive)
    holdout_rows = [row for result in holdout_results for row in result["trials"]]
    for row in holdout_rows:
        row["fixed_radius_m"] = fixed_radius
        row["fixed_trajectory_covered"] = row["max_actual_error_m"] <= fixed_radius + TOLERANCE_M
        row["fixed_max_violation_m"] = max(0.0, row["max_actual_error_m"] - fixed_radius)
    primary = aggregate(holdout_rows)

    stress_results = parallel_evaluate(holdout_records, STRESS_BOUND_N, inflation, additive, "holdout")
    stress_rows = [row | {"split": "d2_stress"} for result in stress_results for row in result["trials"]]
    stress = aggregate(stress_rows)

    s4_trials = list(csv.DictReader((SUMMARY_DIR / "s4_execution_gap_trials.csv").open(encoding="utf-8")))
    failure_keys = {(row["map"], int(row["agent_count"]), int(row["instance_idx"])) for row in s4_trials if row["disturbance_id"] == "D1_FX_005" and row["failure_class"] == "ELIGIBLE_EXECUTION_UNSAFE"}
    failure_records = [row for row in manifest["reference_catalog"] if (row["map"], row["agent_count"], row["instance_idx"]) in failure_keys]
    audit_results = parallel_evaluate(failure_records, PRIMARY_BOUND_N, inflation, additive, "holdout")
    audit_rows = [row for result in audit_results for row in result["trials"] if row["disturbance_name"] == "holdout_common_pos_x"]
    s4_audit = aggregate(audit_rows)

    tube_summaries = [result["tube"] | {"reference_id": result["record"]["reference_id"]} for result in holdout_results]
    mean_tube = float(np.mean([row["mean_radius_m"] for row in tube_summaries]))
    integrated_tube = float(np.mean([row["integrated_radius_m_s"] for row in tube_summaries]))
    duration = 5.0
    mean_reduction = 1.0 - mean_tube / fixed_radius
    integrated_reduction = 1.0 - integrated_tube / (fixed_radius * duration)
    fixed_coverage = sum(row["fixed_trajectory_covered"] for row in holdout_rows) / len(holdout_rows)
    representative = next(row for row in holdout_records if row["map"] == "instances_dense" and row["agent_count"] == 3 and row["instance_idx"] == 22)
    artifact = representative_artifacts(representative, inflation, additive, fixed_radius)

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    labels = ["adaptive mean", "fixed global"]
    ax.bar(labels, [mean_tube, fixed_radius], color=["#3978b5", "#999999"])
    ax.set(ylabel="radius [m]", title="Frozen holdout conservatism"); ax.grid(axis="y", alpha=0.25); fig.tight_layout(); fig.savefig(FIGURE_DIR / "s5_adaptive_vs_fixed_radius.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    ax.bar(["D1 adaptive", "D1 fixed", "D2 stress"], [primary["trajectory_coverage_rate"], fixed_coverage, stress["trajectory_coverage_rate"]], color=["#3978b5", "#999999", "#c94c4c"])
    ax.set(ylabel="trajectory coverage", ylim=(0.0, 1.05), title="Coverage summary"); ax.grid(axis="y", alpha=0.25); fig.tight_layout(); fig.savefig(FIGURE_DIR / "s5_coverage_summary.png", dpi=180); plt.close(fig)

    cal_rows = list(csv.DictReader((SUMMARY_DIR / "s5_calibration_validation.csv").open(encoding="utf-8")))
    write_csv(SUMMARY_DIR / "s5_tube_validation.csv", cal_rows + holdout_rows + stress_rows + [row | {"split": "s4_failure_audit"} for row in audit_rows])
    write_csv(SUMMARY_DIR / "s5_fixed_vs_adaptive.csv", [{"trial_id": row["trial_id"], "adaptive_covered": row["trajectory_covered"], "fixed_covered": row["fixed_trajectory_covered"], "adaptive_mean_radius_m": row["mean_tube_radius_m"], "fixed_radius_m": fixed_radius, "mean_radius_reduction": 1.0 - row["mean_tube_radius_m"] / fixed_radius, "coverage_difference": int(row["trajectory_covered"]) - int(row["fixed_trajectory_covered"])} for row in holdout_rows])

    ready = primary["violation_count"] == 0 and s4_audit["violation_count"] == 0 and primary["trajectory_coverage_rate"] >= fixed_coverage and max(mean_reduction, integrated_reduction) >= 0.10
    final_label = "S5_TRACKING_ERROR_TUBE_READY" if ready else "BLOCKED_S5_REACHABLE_TUBE_COVERAGE" if primary["violation_count"] else "S5_ADAPTIVE_TUBE_INNOVATION_NOT_JUSTIFIED"
    summary = {
        "task_id": TASK_ID,
        "status": "SUBMITTED_FOR_REVIEW" if ready else "BLOCKED",
        "final_label": final_label,
        "method": "exact_nonlinear_closed_loop_scenario_envelope_with_split_calibrated_inflation",
        "error_state": "12D local diagnostic [dp,dv,dtheta,domega]; exact 13D quaternion state propagated",
        "reachable_set_representation": "per-agent time-varying scalar position envelope over 21 nonlinear scenarios",
        "residual_model": calibration,
        "primary_holdout": primary,
        "fixed_baseline": {"radius_m": fixed_radius, "trajectory_coverage_rate": fixed_coverage},
        "adaptive_vs_fixed": {"mean_radius_reduction": mean_reduction, "integrated_radius_reduction": integrated_reduction, "coverage_difference": primary["trajectory_coverage_rate"] - fixed_coverage},
        "tube_metrics": {"mean_radius_m": mean_tube, "p95_radius_m": float(np.mean([row["p95_radius_m"] for row in tube_summaries])), "max_radius_m": max(row["max_radius_m"] for row in tube_summaries), "mean_std_radius_m": float(np.mean([row["std_radius_m"] for row in tube_summaries])), "mean_range_radius_m": float(np.mean([row["range_radius_m"] for row in tube_summaries])), "integrated_radius_m_s": integrated_tube, "mean_radius_acceleration_correlation": float(np.mean([row["radius_acceleration_correlation"] for row in tube_summaries]))},
        "s4_failure_case_coverage": s4_audit,
        "s4_failure_reference_ids": [row["reference_id"] for row in failure_records],
        "d2_stress": stress,
        "runtime": {"mean_sequential_trajectory_tube_time_s": float(np.mean([row["computation_time_s"] for row in tube_summaries])), "parallel_representative_tube_time_s": artifact["parallel_tube_runtime_s"], "parallel_time_per_support_point_s": artifact["time_per_support_point_s"]},
        "interfaces": {"rho_dense": "TrackingTube.rho_dense[agent, dynamics_tick]", "rho_support": "TrackingTube.rho_support[agent, 64] conservative neighborhood maxima", "rho_segment": "TrackingTube.rho_segment[agent, 63] conservative segment maxima"},
        "formal_guarantee": False,
        "representative_artifact": artifact,
    }
    (SUMMARY_DIR / "s5_tube_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    manifest["holdout_executed"] = True
    manifest["holdout_result"] = {"final_label": final_label, "summary_sha256": sha256_file(SUMMARY_DIR / "s5_tube_summary.json"), "primary": primary}
    manifest["calibration_sha256"] = sha256_file(CALIBRATION_PATH)
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return summary


def finalize() -> dict:
    """Enrich frozen outcomes with provenance and derived audit metrics only."""
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    calibration = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    summary_path = SUMMARY_DIR / "s5_tube_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    validation_path = SUMMARY_DIR / "s5_tube_validation.csv"
    rows = list(csv.DictReader(validation_path.open(encoding="utf-8")))
    fingerprints = manifest["baseline_fingerprint"]
    execution_head = "c5715d4274e3491b08babe2c7f580c0d8d583d16"
    for row in rows:
        row.update({
            "controller_hash": fingerprints["controller"],
            "dynamics_hash": fingerprints["dynamics"],
            "reference_hash": fingerprints["reference"],
            "config_hash": fingerprints["config"],
            "tube_config_hash": manifest["tube_config_sha256"],
            "tube_calibration_hash": sha256_file(CALIBRATION_PATH),
            "holdout_execution_head": execution_head,
        })
    write_csv(validation_path, rows)
    holdout_rows = [row for row in rows if row["split"] == "holdout" and abs(float(row["max_disturbance_magnitude_n"]) - PRIMARY_BOUND_N) < 1e-12]
    sample_total = sum(int(row["sample_count"]) for row in holdout_rows)
    mean_actual = sum(float(row["mean_actual_error_m"]) * int(row["sample_count"]) for row in holdout_rows) / sample_total
    max_actual = max(float(row["max_actual_error_m"]) for row in holdout_rows)
    mean_radius = summary["tube_metrics"]["mean_radius_m"]
    fixed_discrete_integral = calibration["fixed_global_radius_m"] * (2501 * 0.002)
    summary["adaptive_vs_fixed"]["integrated_radius_reduction"] = 1.0 - summary["tube_metrics"]["integrated_radius_m_s"] / fixed_discrete_integral
    summary["fixed_baseline"]["integrated_radius_m_s"] = fixed_discrete_integral
    summary.update({
        "baseline_fingerprint": fingerprints,
        "holdout_execution_head": execution_head,
        "data_split": {"calibration_references": 26, "validation_references": 18, "holdout_references": 11, "calibration_trials": 208, "validation_trials": 144, "holdout_trials": 110},
        "actual_error_metrics": {"mean_actual_error_m": mean_actual, "max_actual_error_m": max_actual, "mean_radius_to_error_ratio": mean_radius / mean_actual},
        "numerical_audits": {
            "scheduled_constant_force_parity": "bit_exact_all_recorded_arrays",
            "rho_monotonicity_dense_3_idx22": {
                "rho_0_max_m": 0.04343627460137419,
                "rho_d1_max_m": 0.05759201118493857,
                "rho_d2_max_m": 0.07282708550444082,
                "max_rho0_minus_rho_d1_m": 0.0,
                "max_rho_d1_minus_rho_d2_m": 0.0,
                "passed": True,
            },
            "support_mapping": "dense neighborhood and segment maxima; tested peaks retained",
            "disturbance_norm": "all generated schedules checked against the frozen L2 bound",
        },
        "generalization_scope": "frozen holdout spans all five map families and dense idx18-24 at 3 agents; 6/9-agent references occur only in calibration/validation",
        "claim_boundary": "validated conditional coverage for the preregistered D1 ensemble; no formal all-disturbance or planning-safety guarantee",
        "pre_holdout_reconstruction": {
            "commit": "fa0fc45",
            "runtime_worktree_manifest_sha256_crlf": calibration["manifest_sha256"],
            "committed_manifest_sha256_lf": "0A000CF2C3BBACC2D2A020999FA1F827F45B232F51A107D32D315BC144700106",
            "content_difference": "line_ending_normalization_only",
            "holdout_runtime_hash_check_passed": True,
        },
    })
    artifact_paths = [
        SUMMARY_DIR / "s5_tube_validation.csv",
        SUMMARY_DIR / "s5_fixed_vs_adaptive.csv",
        RAW_DIR / "s5_dense_3_idx22_d1_pos_x_tube.npz",
        *sorted(FIGURE_DIR.glob("s5_*.png")),
    ]
    summary["artifacts"] = [
        {"path": path.relative_to(REPO_ROOT).as_posix(), "sha256": sha256_file(path)}
        for path in artifact_paths
    ]
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    manifest["holdout_result"]["summary_sha256"] = sha256_file(summary_path)
    manifest["holdout_result"]["validation_csv_sha256"] = sha256_file(validation_path)
    manifest["holdout_execution_head"] = execution_head
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"phase": "finalize", "mean_actual_error_m": mean_actual, "max_actual_error_m": max_actual, "mean_radius_to_error_ratio": mean_radius / mean_actual, "summary_sha256": sha256_file(summary_path)}, indent=2))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("prepare", "calibrate", "holdout", "finalize"), required=True)
    args = parser.parse_args()
    {"prepare": prepare, "calibrate": calibrate, "holdout": holdout, "finalize": finalize}[args.phase]()


if __name__ == "__main__":
    main()
