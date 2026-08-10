"""Minimal local rigid-body error coordinates used for S5 diagnostics."""

from __future__ import annotations

import numpy as np

from tr_smd.dynamics.quadrotor import quat_normalize


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.asarray(left, dtype=float)
    w2, x2, y2, z2 = np.asarray(right, dtype=float)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quaternion_log_error(actual: np.ndarray, nominal: np.ndarray) -> np.ndarray:
    """Return the shortest local rotation vector from nominal to actual."""
    relative = quat_normalize(quat_multiply(quat_conjugate(nominal), actual))
    if relative[0] < 0.0:
        relative = -relative
    vector = relative[1:]
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        return 2.0 * vector
    angle = 2.0 * np.arctan2(norm, np.clip(relative[0], -1.0, 1.0))
    return angle * vector / norm


def minimal_error_state(actual: np.ndarray, nominal: np.ndarray) -> np.ndarray:
    """Map two 13-state vectors to [dp, dv, dtheta, domega]."""
    actual = np.asarray(actual, dtype=float)
    nominal = np.asarray(nominal, dtype=float)
    if actual.shape != (13,) or nominal.shape != (13,):
        raise ValueError("actual and nominal must be 13-state vectors")
    return np.concatenate([
        actual[0:3] - nominal[0:3],
        actual[3:6] - nominal[3:6],
        quaternion_log_error(actual[6:10], nominal[6:10]),
        actual[10:13] - nominal[10:13],
    ])
