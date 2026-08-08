"""Run S2 parity checks over all accepted S1 raw trajectories."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from baseline_metrics import (
    COLLISION_THRESHOLD,
    OFFICIAL_SOURCE_COMMIT,
    PARITY_TOLERANCE,
    PRIMARY_CANDIDATE_INDEX,
    ROBOT_RADIUS,
    _load_official_collision_function,
    _map_data,
    decode_primary_trajectory,
    evaluate_run,
    parse_planning_time,
)


def _row(metric: str, test_run: str, official: Any, reproduced: Any, error: float, tolerance: float, status: str, source_file: str, source_function: str) -> Dict[str, Any]:
    return {
        "metric": metric,
        "test_run": test_run,
        "official_value": json.dumps(official, separators=(",", ":")),
        "reproduced_value": json.dumps(reproduced, separators=(",", ":")),
        "abs_error": error,
        "tolerance": tolerance,
        "parity_status": status,
        "source_file": source_file,
        "source_function": source_function,
    }


def run_parity(s1_csv: Path, repo_root: Path) -> Dict[str, Any]:
    rows = list(csv.DictReader(s1_csv.open(encoding="utf-8")))
    parity_rows: List[Dict[str, Any]] = []
    collision_mismatches = 0
    path_errors = []
    acceleration_errors = []
    shape_passes = 0
    candidate_passes = 0
    official_collision = _load_official_collision_function(repo_root)

    for row in rows:
        result = evaluate_run(row, repo_root)
        run_id = row["run_id"]
        result_dir = repo_root / row["result_dir"]
        raw_paths = np.load(result_dir / "paths.npy")
        position, velocity, num_agents = decode_primary_trajectory(raw_paths)
        map_info = __import__("pickle").loads((result_dir / "map_info.pkl").read_bytes())
        obstacles, robot_data = _map_data(repo_root, map_info, num_agents)
        direct_collision = bool(
            official_collision(
                position,
                obstacles,
                robot_data,
                robot_radius=ROBOT_RADIUS,
                threshold=COLLISION_THRESHOLD,
            )
        )
        if direct_collision != result["official_collision_free"]:
            collision_mismatches += 1
        parity_rows.append(
            _row(
                "collision_boolean",
                run_id,
                direct_collision,
                result["official_collision_free"],
                float(direct_collision != result["official_collision_free"]),
                0.0,
                "PASS" if direct_collision == result["official_collision_free"] else "FAIL",
                "is_collision.py",
                "check_paths_ok",
            )
        )

        official_lengths = np.asarray(result["path_length_per_agent"], dtype=float)
        reference_lengths = np.asarray(result["reference_path_length_per_agent"], dtype=float)
        length_error = float(np.max(np.abs(official_lengths - reference_lengths)))
        path_errors.append(length_error)
        parity_rows.append(
            _row(
                "path_length",
                run_id,
                official_lengths.tolist(),
                reference_lengths.tolist(),
                length_error,
                PARITY_TOLERANCE,
                "PASS" if length_error <= PARITY_TOLERANCE else "FAIL",
                "deps/torch_robotics/torch_robotics/trajectory/metrics.py",
                "compute_path_length_from_pos",
            )
        )

        official_acc = np.asarray(result["acceleration_per_agent"], dtype=float)
        reference_acc = np.asarray(result["reference_acceleration_per_agent"], dtype=float)
        acceleration_error = float(np.max(np.abs(official_acc - reference_acc)))
        acceleration_errors.append(acceleration_error)
        parity_rows.append(
            _row(
                "average_acceleration",
                run_id,
                official_acc.tolist(),
                reference_acc.tolist(),
                acceleration_error,
                PARITY_TOLERANCE,
                "PASS" if acceleration_error <= PARITY_TOLERANCE else "FAIL",
                "deps/torch_robotics/torch_robotics/trajectory/metrics.py",
                "compute_average_acceleration_from_pos_vel",
            )
        )

        shape_ok = (
            result["state_dim"] == 4 * num_agents
            and result["position_shape"] == [num_agents, raw_paths.shape[1], 2]
            and result["velocity_shape"] == [num_agents, raw_paths.shape[1], 2]
        )
        candidate_ok = result["candidate_index"] == PRIMARY_CANDIDATE_INDEX
        shape_passes += int(shape_ok)
        candidate_passes += int(candidate_ok)

    parity_rows.extend(
        [
            _row("candidate_selection", "all_9_s1_runs", 0, PRIMARY_CANDIDATE_INDEX, 0.0, 0.0, "PASS" if candidate_passes == len(rows) else "FAIL", "is_collision.py", "paths_data[0,...]"),
            _row("position_extraction", "all_9_s1_runs", "(N,T,2)", "(N,T,2)", 0.0, 0.0, "PASS" if shape_passes == len(rows) else "FAIL", "is_collision.py", "paths_data[0,:,:2*N]"),
            _row("velocity_extraction", "all_9_s1_runs", "(N,T,2)", "(N,T,2)", 0.0, 0.0, "PASS" if shape_passes == len(rows) else "FAIL", "inference_multi_agent.py", "paths_np / state layout"),
        ]
    )
    return {
        "official_source_commit": OFFICIAL_SOURCE_COMMIT,
        "matrix_rows": len(rows),
        "collision_parity_count": len(rows) - collision_mismatches,
        "collision_boolean_mismatch": collision_mismatches,
        "path_length_parity": all(error <= PARITY_TOLERANCE for error in path_errors),
        "path_length_max_abs_error": max(path_errors),
        "acceleration_parity": all(error <= PARITY_TOLERANCE for error in acceleration_errors),
        "acceleration_max_abs_error": max(acceleration_errors),
        "candidate_selection_pass_count": candidate_passes,
        "shape_pass_count": shape_passes,
        "table_rows": parity_rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--s1-csv", type=Path, default=Path("experiments/summaries/s1_r1_core_reproduction.csv"))
    parser.add_argument("--output-csv", type=Path, default=Path("experiments/summaries/official_vs_reproduced_table.csv"))
    parser.add_argument("--output-json", type=Path, default=Path("experiments/summaries/metric_parity_results.json"))
    parser.add_argument("--runtime-json", type=Path, default=Path("experiments/summaries/s2_runtime_parity.json"))
    args = parser.parse_args()
    result = run_parity(args.s1_csv, Path(__file__).resolve().parents[2])
    runtime = json.loads(args.runtime_json.read_text(encoding="utf-8"))
    runtime_log = Path(runtime["stdout_log"])
    runtime_text = runtime_log.read_text(encoding="utf-16")
    parsed_runtime = parse_planning_time(runtime_text)
    runtime_error = abs(parsed_runtime - float(runtime["planning_time_s"]))
    result["runtime_parity"] = bool(
        runtime["retry_status"] == "PASS"
        and runtime["planning_time_le_wall_runtime"]
        and runtime_error <= 0.0
    )
    result["runtime_parity_run"] = runtime["retry_attempt_id"]
    result["table_rows"].append(
        _row(
            "runtime_parse",
            runtime["retry_attempt_id"],
            runtime["planning_time_s"],
            parsed_runtime,
            runtime_error,
            0.0,
            "PASS" if result["runtime_parity"] else "FAIL",
            "scripts/inference/inference_multi_agent.py",
            "Planning times",
        )
    )
    fields = ["metric", "test_run", "official_value", "reproduced_value", "abs_error", "tolerance", "parity_status", "source_file", "source_function"]
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(result["table_rows"])
    args.output_json.write_text(json.dumps({k: v for k, v in result.items() if k != "table_rows"}, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "table_rows"}, sort_keys=True))


if __name__ == "__main__":
    main()
