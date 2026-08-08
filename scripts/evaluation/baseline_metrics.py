"""S2 official SMD baseline metric evaluator.

This module is deliberately planning-level and read-only.  It consumes the
accepted S1 ``paths.npy``/``map_info.pkl`` artifacts, selects candidate 0,
uses the official Torch Robotics metric functions for the primary parity
metrics, and extracts the official collision function from ``is_collision.py``.

It is not a benchmark or paper-metric pipeline.
"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import pickle
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


OFFICIAL_SOURCE_COMMIT = "c87fc76044b350a37fcea7afc468c13c8371a237"
PRIMARY_CANDIDATE_INDEX = 0
ROBOT_RADIUS = 0.05
COLLISION_THRESHOLD = 1e-3
PARITY_TOLERANCE = 1e-6


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def _load_official_collision_function(repo_root: Path):
    source_path = repo_root / "is_collision.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    function_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "check_paths_ok"
    )
    namespace = {"np": np}
    module = ast.Module(body=[function_node], type_ignores=[])
    exec(compile(module, str(source_path), "exec"), namespace)
    return namespace["check_paths_ok"]


def _load_official_torch_metrics(repo_root: Path):
    deps_root = repo_root / "deps" / "torch_robotics"
    if str(deps_root) not in sys.path:
        sys.path.insert(0, str(deps_root))
    from torch_robotics.trajectory.metrics import (  # pylint: disable=import-outside-toplevel
        compute_average_acceleration_from_pos_vel,
        compute_path_length_from_pos,
    )

    return compute_path_length_from_pos, compute_average_acceleration_from_pos_vel


def decode_primary_trajectory(raw_paths: np.ndarray) -> Tuple[np.ndarray, np.ndarray, int]:
    """Decode the official candidate/state layout into (N,T,2) arrays."""
    if raw_paths.ndim != 3 or raw_paths.shape[0] <= PRIMARY_CANDIDATE_INDEX:
        raise ValueError(f"unexpected raw paths shape: {raw_paths.shape}")
    if raw_paths.shape[2] % 4 != 0:
        raise ValueError(f"state dimension is not 4*N: {raw_paths.shape}")
    num_agents = raw_paths.shape[2] // 4
    candidate = raw_paths[PRIMARY_CANDIDATE_INDEX]
    position = candidate[:, : 2 * num_agents].reshape(
        candidate.shape[0], num_agents, 2
    ).swapaxes(0, 1)
    velocity = candidate[:, 2 * num_agents : 4 * num_agents].reshape(
        candidate.shape[0], num_agents, 2
    ).swapaxes(0, 1)
    return position, velocity, num_agents


def _map_data(repo_root: Path, map_info: Dict[str, Any], num_agents: int):
    map_path = repo_root / "instances_data" / f"{map_info['map_name']}.pkl"
    map_records = pickle.loads(map_path.read_bytes())
    instance_records = map_records[int(map_info["instance_idx"])]
    return next(record for record in instance_records if len(record[1]) == num_agents)


def _signed_clearances(
    position: np.ndarray, obstacles: Sequence[Tuple[Sequence[float], float]]
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    min_center_distance = None
    min_inter_agent_clearance = None
    if position.shape[0] > 1:
        pair_distances = []
        for i in range(position.shape[0]):
            for j in range(i + 1, position.shape[0]):
                pair_distances.append(np.linalg.norm(position[i] - position[j], axis=1))
        min_center_distance = float(np.min(np.concatenate(pair_distances)))
        min_inter_agent_clearance = min_center_distance - 2 * ROBOT_RADIUS

    obstacle_clearances = []
    for center, radius in obstacles:
        distance = np.linalg.norm(position - np.asarray(center, dtype=float), axis=2)
        obstacle_clearances.append(distance - ROBOT_RADIUS - float(radius))
    min_obstacle_clearance = (
        float(np.min(np.concatenate(obstacle_clearances)))
        if obstacle_clearances
        else None
    )
    return min_center_distance, min_inter_agent_clearance, min_obstacle_clearance


def _numpy_path_length(position: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.diff(position, axis=1), axis=2).sum(axis=1)


def _numpy_average_acceleration(velocity: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.diff(velocity, axis=1), axis=2).mean(axis=1)


def evaluate_run(row: Dict[str, str], repo_root: Optional[Path] = None) -> Dict[str, Any]:
    repo_root = repo_root or _repo_root()
    result_dir = repo_root / row["result_dir"]
    paths_file = result_dir / "paths.npy"
    map_info_file = result_dir / "map_info.pkl"
    result: Dict[str, Any] = {
        "experiment_id": row["run_id"],
        "source_stage": row.get("run_origin", "S1_R1_NEW"),
        "git_head": row.get("git_head", "b68212d5f5e7ede4d001d68a40378d2cdf4647fa"),
        "map_name": row["map_name"],
        "instance_idx": int(row["instance_idx"]),
        "num_agents": int(row["num_agents"]),
        "candidate_index": PRIMARY_CANDIDATE_INDEX,
        "raw_result_exists": paths_file.is_file() and map_info_file.is_file(),
        "result_dir": row["result_dir"],
        "paths_sha256": _sha256(paths_file) if paths_file.is_file() else None,
        "map_info_sha256": _sha256(map_info_file) if map_info_file.is_file() else None,
        "wall_runtime_s": float(row["runtime_seconds"]),
        "retry_of": None,
        "failure_class": "NONE",
    }
    note = row.get("failure_note", "")
    if "error_2026-08-08-20-37-32" in note:
        result["retry_of"] = "dense_6_idx0_failure_20260808T203732"
    elif "error_2026-08-08-20-47-59" in note:
        result["retry_of"] = "dense_9_idx0_failure_20260808T204759"

    if not result["raw_result_exists"]:
        result.update(
            {
                "finite": False,
                "official_collision_free": None,
                "planning_success": False,
                "failure_class": "NO_OUTPUT_FAILURE",
                "planning_time_s": None,
            }
        )
        return result

    raw_paths = np.load(paths_file)
    result["finite"] = bool(np.isfinite(raw_paths).all())
    if not result["finite"]:
        result.update(
            {
                "official_collision_free": None,
                "planning_success": False,
                "failure_class": "NONFINITE_FAILURE",
                "planning_time_s": None,
            }
        )
        return result

    position, velocity, num_agents = decode_primary_trajectory(raw_paths)
    map_info = pickle.loads(map_info_file.read_bytes())
    obstacles, robot_data = _map_data(repo_root, map_info, num_agents)
    check_paths_ok = _load_official_collision_function(repo_root)
    official_collision_free = bool(
        check_paths_ok(
            position,
            obstacles,
            robot_data,
            robot_radius=ROBOT_RADIUS,
            threshold=COLLISION_THRESHOLD,
        )
    )

    official_path_length, official_acceleration = _load_official_torch_metrics(repo_root)
    import torch  # pylint: disable=import-outside-toplevel

    position_torch = torch.from_numpy(position)
    velocity_torch = torch.from_numpy(velocity)
    official_lengths = official_path_length(position_torch).detach().cpu().numpy()
    official_accelerations = official_acceleration(
        position_torch, velocity_torch
    ).detach().cpu().numpy()
    reference_lengths = _numpy_path_length(position)
    reference_accelerations = _numpy_average_acceleration(velocity)
    min_center_distance, min_inter_clearance, min_obstacle_clearance = _signed_clearances(
        position, obstacles
    )
    starts = np.asarray([robot[0] for robot in robot_data], dtype=float)
    goals = np.asarray([robot[1] for robot in robot_data], dtype=float)
    start_errors = np.linalg.norm(position[:, 0] - starts, axis=1)
    goal_errors = np.linalg.norm(position[:, -1] - goals, axis=1)

    result.update(
        {
            "raw_path_shape": list(raw_paths.shape),
            "position_shape": list(position.shape),
            "velocity_shape": list(velocity.shape),
            "state_dim": int(raw_paths.shape[2]),
            "expected_state_dim": int(4 * num_agents),
            "official_collision_free": official_collision_free,
            "planning_success": bool(official_collision_free),
            "mean_path_length_per_agent": float(np.mean(official_lengths)),
            "total_path_length": float(np.sum(official_lengths)),
            "path_length_per_agent": official_lengths.tolist(),
            "reference_path_length_per_agent": reference_lengths.tolist(),
            "official_mean_path_acceleration_per_agent": float(
                np.mean(official_accelerations)
            ),
            "acceleration_per_agent": official_accelerations.tolist(),
            "reference_acceleration_per_agent": reference_accelerations.tolist(),
            "min_inter_agent_center_distance": min_center_distance,
            "min_inter_agent_clearance": min_inter_clearance,
            "min_obstacle_clearance": min_obstacle_clearance,
            "start_error_per_agent": start_errors.tolist(),
            "goal_error_per_agent": goal_errors.tolist(),
            "planning_time_s": None,
        }
    )
    return result


def parse_planning_time(text: str) -> float:
    match = re.search(r"Planning times:\s*([0-9.+eE-]+)", text)
    if not match:
        raise ValueError("could not parse official 'Planning times:' output")
    return float(match.group(1))


def _json_default(value: Any):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(type(value).__name__)


def write_csv(rows: Iterable[Dict[str, Any]], output_path: Path) -> None:
    rows = list(rows)
    fields = [
        "experiment_id", "source_stage", "git_head", "map_name", "instance_idx",
        "num_agents", "candidate_index", "raw_result_exists", "finite",
        "official_collision_free", "planning_success", "mean_path_length_per_agent",
        "total_path_length", "official_mean_path_acceleration_per_agent",
        "min_inter_agent_center_distance", "min_inter_agent_clearance",
        "min_obstacle_clearance", "planning_time_s", "wall_runtime_s",
        "failure_class", "retry_of", "result_dir", "paths_sha256", "map_info_sha256",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--s1-csv", type=Path, default=Path("experiments/summaries/s1_r1_core_reproduction.csv"))
    parser.add_argument("--output-csv", type=Path, default=Path("experiments/summaries/baseline_metrics.csv"))
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args()
    rows = list(csv.DictReader(args.s1_csv.open(encoding="utf-8")))
    evaluated = [evaluate_run(row) for row in rows]
    write_csv(evaluated, args.output_csv)
    if args.output_json:
        args.output_json.write_text(json.dumps(evaluated, indent=2, default=_json_default), encoding="utf-8")
    print(json.dumps({"rows": len(evaluated), "output": str(args.output_csv)}, default=_json_default))


if __name__ == "__main__":
    main()
