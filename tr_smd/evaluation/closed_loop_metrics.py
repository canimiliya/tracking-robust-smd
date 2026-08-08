"""Metrics for actual quadrotor execution and strict XY collision checks."""

from __future__ import annotations

import numpy as np


def _rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum(x * x, axis=-1))))


def compute_execution_metrics(time, nominal_position, actual_position, actual_velocity, actual_quaternion, rotor_thrust, obstacles, robot_radius=0.05):
    nominal_position = np.asarray(nominal_position, dtype=float)
    actual_position = np.asarray(actual_position, dtype=float)
    actual_velocity = np.asarray(actual_velocity, dtype=float)
    actual_quaternion = np.asarray(actual_quaternion, dtype=float)
    rotor_thrust = np.asarray(rotor_thrust, dtype=float)
    xy_error = actual_position[:, :, :2] - nominal_position[:, :, :2]
    per_rmse = np.array([_rmse(xy_error[i]) for i in range(xy_error.shape[0])])
    per_max = np.max(np.linalg.norm(xy_error, axis=-1), axis=1)
    per_final = np.linalg.norm(xy_error[:, -1], axis=-1)
    z_error = actual_position[:, :, 2] - nominal_position[:, :, 2]
    min_pair = np.inf
    for i in range(actual_position.shape[0]):
        for j in range(i + 1, actual_position.shape[0]):
            min_pair = min(min_pair, float(np.min(np.linalg.norm(actual_position[i, :, :2] - actual_position[j, :, :2], axis=1))))
    min_obstacle = np.inf
    for center, radius in obstacles:
        center = np.asarray(center, dtype=float)
        distances = np.linalg.norm(actual_position[:, :, :2] - center, axis=-1)
        min_obstacle = min(min_obstacle, float(np.min(distances - robot_radius - float(radius))))
    max_tilt = 0.0
    max_quaternion_norm_error = 0.0
    for q in actual_quaternion.reshape(-1, 4):
        norm = np.linalg.norm(q)
        max_quaternion_norm_error = max(max_quaternion_norm_error, float(abs(norm - 1.0)))
        w, x, y, z = q / norm
        b3_z = 1.0 - 2.0 * (x * x + y * y)
        max_tilt = max(max_tilt, float(np.degrees(np.arccos(np.clip(b3_z, -1.0, 1.0)))))
    inter_clearance = min_pair - 2.0 * robot_radius
    saturation_fraction = float(np.mean(rotor_thrust >= (0.027 * 9.81 * 2.25 / 4.0 - 1e-12)))
    return {
        "per_agent_xy_rmse_m": per_rmse.tolist(),
        "per_agent_xy_max_error_m": per_max.tolist(),
        "per_agent_final_xy_error_m": per_final.tolist(),
        "mean_xy_rmse_m": float(np.mean(per_rmse)),
        "max_xy_error_all_agents_m": float(np.max(per_max)),
        "z_rmse_m": float(np.sqrt(np.mean(z_error * z_error))),
        "z_max_error_m": float(np.max(np.abs(z_error))),
        "max_speed_mps": float(np.max(np.linalg.norm(actual_velocity, axis=-1))),
        "max_tilt_deg": max_tilt,
        "max_quaternion_norm_error": max_quaternion_norm_error,
        "rotor_saturation_fraction": saturation_fraction,
        "execution_geometric_collision_free": bool(inter_clearance >= 0.0 and min_obstacle >= 0.0),
        "min_actual_inter_agent_distance_m": float(min_pair) if np.isfinite(min_pair) else None,
        "min_actual_inter_agent_clearance_m": float(inter_clearance) if np.isfinite(inter_clearance) else None,
        "min_actual_obstacle_clearance_m": float(min_obstacle) if np.isfinite(min_obstacle) else None,
        "finite_state": bool(np.isfinite(actual_position).all() and np.isfinite(actual_velocity).all() and np.isfinite(actual_quaternion).all()),
    }
