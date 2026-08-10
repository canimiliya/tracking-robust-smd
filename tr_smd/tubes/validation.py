"""Frozen S5 reference catalog, disturbance trials, and coverage metrics."""

from __future__ import annotations

import hashlib
import json
import pickle
import re
from dataclasses import asdict
from pathlib import Path

import numpy as np

from tr_smd.evaluation.execution_gap import sha256_file
from tr_smd.tubes.scenario_tube import ScenarioSpec, disturbance_schedule


def reference_key(map_name: str, agent_count: int, instance_idx: int) -> str:
    return f"{map_name}|{agent_count}|{instance_idx}"


def split_for(map_name: str, agent_count: int, instance_idx: int) -> str:
    if agent_count == 3 and map_name == "instances_dense" and 18 <= instance_idx <= 24:
        return "holdout"
    if agent_count == 3 and map_name != "instances_dense" and instance_idx == 4:
        return "holdout"
    if agent_count == 3 and map_name == "instances_dense" and 9 <= instance_idx <= 17:
        return "validation"
    if agent_count == 3 and instance_idx == 3:
        return "validation"
    if map_name in ("instances_empty", "instances_shelf") and agent_count in (6, 9) and instance_idx == 0:
        return "validation"
    return "calibration"


def evaluation_specs(split: str) -> list[ScenarioSpec]:
    if split == "calibration":
        return [
            ScenarioSpec("cal_common_fixed_022", "common_fixed", np.deg2rad(22.5)),
            ScenarioSpec("cal_common_fixed_112", "common_fixed", np.deg2rad(112.5)),
            ScenarioSpec("cal_independent_fixed_011", "independent_fixed", np.deg2rad(11.25)),
            ScenarioSpec("cal_independent_fixed_056", "independent_fixed", np.deg2rad(56.25)),
            ScenarioSpec("cal_common_piecewise_1101", "common_piecewise", seed=1101),
            ScenarioSpec("cal_common_piecewise_1102", "common_piecewise", seed=1102),
            ScenarioSpec("cal_independent_piecewise_1201", "independent_piecewise", seed=1201),
            ScenarioSpec("cal_independent_piecewise_1202", "independent_piecewise", seed=1202),
        ]
    if split == "validation":
        return [
            ScenarioSpec("val_common_fixed_067", "common_fixed", np.deg2rad(67.5)),
            ScenarioSpec("val_common_fixed_157", "common_fixed", np.deg2rad(157.5)),
            ScenarioSpec("val_independent_fixed_033", "independent_fixed", np.deg2rad(33.75)),
            ScenarioSpec("val_independent_fixed_078", "independent_fixed", np.deg2rad(78.75)),
            ScenarioSpec("val_common_piecewise_2101", "common_piecewise", seed=2101),
            ScenarioSpec("val_common_piecewise_2102", "common_piecewise", seed=2102),
            ScenarioSpec("val_independent_piecewise_2201", "independent_piecewise", seed=2201),
            ScenarioSpec("val_independent_piecewise_2202", "independent_piecewise", seed=2202),
        ]
    if split == "holdout":
        return [
            ScenarioSpec("holdout_common_pos_x", "common_fixed", 0.0),
            ScenarioSpec("holdout_common_neg_x", "common_fixed", np.pi),
            ScenarioSpec("holdout_common_pos_y", "common_fixed", 0.5 * np.pi),
            ScenarioSpec("holdout_common_neg_y", "common_fixed", -0.5 * np.pi),
            ScenarioSpec("holdout_common_fixed_017", "common_fixed", np.deg2rad(17.0)),
            ScenarioSpec("holdout_independent_fixed_019", "independent_fixed", np.deg2rad(19.0)),
            ScenarioSpec("holdout_common_piecewise_3101", "common_piecewise", seed=3101),
            ScenarioSpec("holdout_common_piecewise_3102", "common_piecewise", seed=3102),
            ScenarioSpec("holdout_independent_piecewise_3201", "independent_piecewise", seed=3201),
            ScenarioSpec("holdout_independent_piecewise_3202", "independent_piecewise", seed=3202),
        ]
    raise ValueError(f"unknown split: {split}")


def schedule_sha256(spec: ScenarioSpec, num_agents: int, bound_n: float) -> str:
    schedule = disturbance_schedule(spec, num_agents, bound_n)
    digest = hashlib.sha256()
    digest.update(json.dumps(asdict(spec), sort_keys=True).encode("utf-8"))
    digest.update(schedule.tobytes(order="C"))
    return digest.hexdigest().upper()


def discover_reference_catalog(repo_root: Path) -> list[dict]:
    """Resolve the frozen 55-reference S4 catalog to deterministic local files."""
    repo_root = Path(repo_root)
    manifest = json.loads((repo_root / "experiments/manifests/s4_campaign_manifest.json").read_text(encoding="utf-8"))
    expected: dict[str, str] = {}
    for trial in manifest["trials"]:
        key = reference_key(trial["map"], int(trial["agent_count"]), int(trial["instance_idx"]))
        digest = trial["source_paths_sha256"].upper()
        if key in expected and expected[key] != digest:
            raise ValueError(f"S4 source hash conflict for {key}")
        expected[key] = digest
    candidates: dict[str, list[Path]] = {digest: [] for digest in set(expected.values())}
    for path in (repo_root / "scripts/inference").glob("**/paths.npy"):
        digest = sha256_file(path)
        if digest in candidates:
            candidates[digest].append(path)
    catalog = []
    for key, digest in sorted(expected.items()):
        paths = sorted(candidates[digest], key=lambda path: (len(str(path)), str(path)))
        if not paths:
            raise FileNotFoundError(f"cannot resolve frozen source {key} {digest}")
        path = paths[0]
        map_name, agent_text, idx_text = key.split("|")
        map_info = pickle.loads(path.with_name("map_info.pkl").read_bytes())
        if map_info["map_name"] != map_name or int(map_info["instance_idx"]) != int(idx_text):
            raise ValueError(f"map metadata mismatch for {path}")
        path_agents = next((int(match.group(1)) for part in path.parts if (match := re.match(r"num_agents___(\d+)", part))), None)
        if path_agents != int(agent_text):
            raise ValueError(f"agent-count mismatch for {path}")
        catalog.append({
            "reference_id": key,
            "map": map_name,
            "agent_count": int(agent_text),
            "instance_idx": int(idx_text),
            "split": split_for(map_name, int(agent_text), int(idx_text)),
            "paths_file": path.relative_to(repo_root).as_posix(),
            "map_info_file": path.with_name("map_info.pkl").relative_to(repo_root).as_posix(),
            "source_reference_sha256": digest,
        })
    counts = {split: sum(row["split"] == split for row in catalog) for split in ("calibration", "validation", "holdout")}
    if len(catalog) != 55 or counts != {"calibration": 26, "validation": 18, "holdout": 11}:
        raise AssertionError(f"unexpected frozen catalog split: {len(catalog)} {counts}")
    return catalog


def coverage_metrics(error: np.ndarray, rho: np.ndarray, tolerance_m: float = 1e-9) -> dict:
    error = np.asarray(error, dtype=float)
    rho = np.asarray(rho, dtype=float)
    if error.shape != rho.shape:
        raise ValueError(f"coverage shape mismatch: {error.shape} != {rho.shape}")
    violation = error - rho
    mask = violation > tolerance_m
    dt = 0.002
    return {
        "tube_coverage_rate": float(np.mean(~mask)),
        "sample_count": int(mask.size),
        "covered_sample_count": int(mask.size - np.count_nonzero(mask)),
        "trajectory_covered": bool(not np.any(mask)),
        "max_tube_violation_m": float(max(0.0, np.max(violation))),
        "violation_count": int(np.count_nonzero(mask)),
        "violation_duration_s": float(np.count_nonzero(mask) * dt),
        "max_actual_error_m": float(np.max(error)),
        "mean_actual_error_m": float(np.mean(error)),
        "max_tube_radius_m": float(np.max(rho)),
        "mean_tube_radius_m": float(np.mean(rho)),
        "p95_tube_radius_m": float(np.percentile(rho, 95.0)),
        "integrated_tube_radius_m_s": float(np.mean(np.sum(rho, axis=1) * dt)),
    }
