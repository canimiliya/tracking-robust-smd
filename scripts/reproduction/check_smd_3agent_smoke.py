"""
PROVISIONAL S1 REPRODUCTION CHECKER
NOT PAPER METRIC PIPELINE

This wrapper generalizes only result discovery, agent-count selection, and the
official trajectory reshape for the authorized 3-agent S1 smoke run.

OFFICIAL COLLISION LOGIC SOURCE:
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
    """Load only the official function, avoiding is_collision.py's 9-agent scan."""
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
    parser = argparse.ArgumentParser(description="Provisional S1 3-agent collision smoke.")
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


def main():
    args = _parse_args()
    repo_root = Path(__file__).resolve().parents[2]
    instances_root = args.instances_root or repo_root / "instances_data"

    raw_paths = np.load(args.paths_file)
    if raw_paths.ndim != 3 or raw_paths.shape[0] < 1 or raw_paths.shape[2] % 4 != 0:
        raise ValueError(f"unexpected raw paths shape: {raw_paths.shape}")

    num_agents = raw_paths.shape[2] // 4
    if num_agents != 3:
        raise ValueError(f"authorized S1 smoke expects 3 agents, got {num_agents}")
    path_data = raw_paths[0, :, : num_agents * 2].reshape(raw_paths.shape[1], num_agents, 2)
    path_data = path_data.swapaxes(0, 1)
    if path_data.shape[0] != 3 or path_data.shape[2] != 2:
        raise ValueError(f"unexpected parsed path shape: {path_data.shape}")
    if not np.isfinite(path_data).all():
        raise ValueError("parsed paths contain NaN or Inf")

    map_info = pickle.loads(args.map_info_file.read_bytes())
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
        "parsed_path_shape": list(path_data.shape),
        "finite_paths": bool(np.isfinite(path_data).all()),
        "collision_free": collision_free,
        "minimum_inter_agent_distance": _minimum_inter_agent_distance(path_data),
        "minimum_obstacle_clearance": _minimum_obstacle_clearance(path_data, obstacles),
        "start_position_error_per_agent": start_errors.tolist(),
        "goal_position_error_per_agent": goal_errors.tolist(),
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
