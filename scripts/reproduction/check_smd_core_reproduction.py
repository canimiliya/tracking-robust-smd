"""
PROVISIONAL S1 REPRODUCTION CHECKER
NOT PAPER METRIC PIPELINE

This checker generalizes only result discovery, NUM_AGENTS, trajectory
dimension, and map selection around the official collision logic.

OFFICIAL LOGIC SOURCE:
is_collision.py @ c87fc76044b350a37fcea7afc468c13c8371a237
"""

from __future__ import annotations

import argparse
import ast
import json
import pickle
from pathlib import Path

import numpy as np


ROBOT_RADIUS = 0.05
THRESHOLD = 1e-3
OFFICIAL_CHECKER = Path(__file__).resolve().parents[2] / "is_collision.py"


def _load_official_check_paths_ok():
    """Load only official check_paths_ok, avoiding the hard-coded scan."""
    tree = ast.parse(OFFICIAL_CHECKER.read_text(encoding="utf-8"))
    function_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "check_paths_ok"
    )
    namespace = {"np": np}
    module = ast.Module(body=[function_node], type_ignores=[])
    exec(compile(module, str(OFFICIAL_CHECKER), "exec"), namespace)
    return namespace["check_paths_ok"]


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Provisional S1 official SMD core reproduction checker."
    )
    parser.add_argument("--paths-file", type=Path, required=True)
    parser.add_argument("--map-info-file", type=Path, required=True)
    parser.add_argument("--instances-root", type=Path, default=None)
    return parser.parse_args()


def _minimum_inter_agent_distance(paths):
    if paths.shape[0] < 2:
        return None
    distances = []
    for i in range(paths.shape[0]):
        for j in range(i + 1, paths.shape[0]):
            distances.append(np.linalg.norm(paths[i] - paths[j], axis=1))
    return float(np.min(np.concatenate(distances)))


def _minimum_obstacle_clearance(paths, obstacles):
    clearances = []
    for center, radius in obstacles:
        distances = np.linalg.norm(paths - np.asarray(center), axis=2)
        clearances.append(distances - ROBOT_RADIUS - float(radius))
    return float(np.min(np.concatenate(clearances))) if clearances else None


def check_one(paths_file, map_info_file, instances_root=None):
    repo_root = Path(__file__).resolve().parents[2]
    instances_root = instances_root or repo_root / "instances_data"

    paths_file = Path(paths_file)
    map_info_file = Path(map_info_file)
    raw_paths = np.load(paths_file)
    if raw_paths.ndim != 3 or raw_paths.shape[0] < 1 or raw_paths.shape[2] % 4 != 0:
        raise ValueError(f"unexpected raw paths shape: {raw_paths.shape}")

    num_agents = raw_paths.shape[2] // 4
    expected_last_dim = 4 * num_agents
    shape_contract_pass = raw_paths.shape[2] == expected_last_dim
    path_data = raw_paths[0, :, : 2 * num_agents].reshape(
        raw_paths.shape[1], num_agents, 2
    )
    path_data = path_data.swapaxes(0, 1)
    expected_parsed_shape = (num_agents, raw_paths.shape[1], 2)
    if tuple(path_data.shape) != expected_parsed_shape:
        raise ValueError(f"unexpected parsed path shape: {path_data.shape}")

    finite_paths = bool(np.isfinite(raw_paths).all())
    if not finite_paths:
        raise ValueError("raw paths contain NaN or Inf")

    map_info = pickle.loads(map_info_file.read_bytes())
    map_file = instances_root / f"{map_info['map_name']}.pkl"
    map_records = pickle.loads(map_file.read_bytes())
    instance_records = map_records[map_info["instance_idx"]]
    map_data = next(record for record in instance_records if len(record[1]) == num_agents)
    obstacles, robot_data = map_data

    check_paths_ok = _load_official_check_paths_ok()
    collision_free = bool(
        check_paths_ok(
            path_data,
            obstacles,
            robot_data,
            robot_radius=ROBOT_RADIUS,
            threshold=THRESHOLD,
        )
    )

    starts = np.asarray([robot[0] for robot in robot_data], dtype=float)
    goals = np.asarray([robot[1] for robot in robot_data], dtype=float)
    start_errors = np.linalg.norm(path_data[:, 0, :] - starts, axis=1)
    goal_errors = np.linalg.norm(path_data[:, -1, :] - goals, axis=1)
    result = {
        "checker": "is_collision.py::check_paths_ok",
        "official_checker": str(OFFICIAL_CHECKER),
        "official_checker_commit": "c87fc76044b350a37fcea7afc468c13c8371a237",
        "robot_radius": ROBOT_RADIUS,
        "threshold": THRESHOLD,
        "map_name": map_info["map_name"],
        "instance_idx": int(map_info["instance_idx"]),
        "num_agents": int(num_agents),
        "raw_path_shape": list(raw_paths.shape),
        "raw_path_dtype": str(raw_paths.dtype),
        "expected_last_dim": int(expected_last_dim),
        "shape_contract_pass": bool(shape_contract_pass),
        "parsed_path_shape": list(path_data.shape),
        "finite_paths": finite_paths,
        "collision_check_executed": True,
        "collision_free": collision_free,
        "minimum_inter_agent_distance": _minimum_inter_agent_distance(path_data),
        "minimum_obstacle_clearance": _minimum_obstacle_clearance(path_data, obstacles),
        "start_position_error_per_agent": start_errors.tolist(),
        "goal_position_error_per_agent": goal_errors.tolist(),
    }
    return result


def main():
    args = _parse_args()
    result = check_one(args.paths_file, args.map_info_file, args.instances_root)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
