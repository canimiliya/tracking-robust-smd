"""Frozen SMD candidate-0 timing, state layout, and Hermite reference bridge."""

from __future__ import annotations

import hashlib
import pickle
from pathlib import Path

import numpy as np

SMD_TRAJECTORY_DURATION_S = 5.0
SMD_SUPPORT_POINTS = 64
SMD_REFERENCE_DT_S = SMD_TRAJECTORY_DURATION_S / SMD_SUPPORT_POINTS
SMD_REFERENCE_SPAN_S = (SMD_SUPPORT_POINTS - 1) * SMD_REFERENCE_DT_S
REFERENCE_ALTITUDE_M = 1.0
REFERENCE_YAW_RAD = 0.0


def finite_difference_velocity(position: np.ndarray, dt: float) -> np.ndarray:
    """Central differences inside and one-sided differences at the ends."""
    position = np.asarray(position, dtype=np.float64)
    if position.ndim != 3 or position.shape[1] < 2:
        raise ValueError(f"expected (agents, support_points, dim), got {position.shape}")
    velocity = np.empty_like(position)
    velocity[:, 0] = (position[:, 1] - position[:, 0]) / dt
    velocity[:, -1] = (position[:, -1] - position[:, -2]) / dt
    velocity[:, 1:-1] = (position[:, 2:] - position[:, :-2]) / (2.0 * dt)
    return velocity


def position_derived_velocity(position: np.ndarray, dt: float) -> np.ndarray:
    """Build knot velocities from projected positions only.

    The SMD position projection does not guarantee that its retained diffusion
    velocity state is the derivative of the projected position.  The physical
    execution bridge therefore uses central differences at interior knots and
    the trajectory hard-condition velocity (zero) at both endpoints.
    """
    position = np.asarray(position, dtype=np.float64)
    if position.ndim != 3 or position.shape[1:] != (SMD_SUPPORT_POINTS, 2):
        raise ValueError(f"expected (agents, support_points, 2), got {position.shape}")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    velocity = np.zeros_like(position)
    velocity[:, 1:-1] = (position[:, 2:] - position[:, :-2]) / (2.0 * dt)
    return velocity


def _primary_candidate(paths: np.ndarray, num_agents: int) -> tuple[np.ndarray, np.ndarray]:
    paths = np.asarray(paths)
    if paths.ndim != 3 or paths.shape[0] < 1 or paths.shape[1] != SMD_SUPPORT_POINTS:
        raise ValueError(f"unexpected SMD paths shape: {paths.shape}")
    if paths.shape[2] != 4 * num_agents:
        raise ValueError(f"state layout mismatch: {paths.shape[2]} != {4 * num_agents}")
    candidate = paths[0]
    position = candidate[:, : 2 * num_agents].reshape(SMD_SUPPORT_POINTS, num_agents, 2).transpose(1, 0, 2)
    velocity = candidate[:, 2 * num_agents :].reshape(SMD_SUPPORT_POINTS, num_agents, 2).transpose(1, 0, 2)
    return position.astype(np.float64), velocity.astype(np.float64)


def load_primary_reference(paths_file: Path, map_info_file: Path, expected_agents: int = 3) -> dict:
    """Load candidate 0 without altering the frozen planner output."""
    paths_file = Path(paths_file)
    map_info_file = Path(map_info_file)
    paths = np.load(paths_file)
    position_xy, velocity_xy = _primary_candidate(paths, expected_agents)
    if not np.isfinite(position_xy).all() or not np.isfinite(velocity_xy).all():
        raise ValueError("SMD reference contains NaN or Inf")
    map_info = pickle.loads(map_info_file.read_bytes())
    sha256 = hashlib.sha256(paths_file.read_bytes()).hexdigest().upper()
    return {
        "paths_file": str(paths_file),
        "map_info_file": str(map_info_file),
        "paths_sha256": sha256,
        "map_name": map_info["map_name"],
        "instance_idx": int(map_info["instance_idx"]),
        "candidate_index": 0,
        "position_xy": position_xy,
        "velocity_xy": velocity_xy,
    }


class HermiteReference:
    """Piecewise cubic Hermite reference with analytic velocity/acceleration."""

    def __init__(self, position_xy: np.ndarray, velocity_xy: np.ndarray, altitude: float = REFERENCE_ALTITUDE_M):
        self.position_xy = np.asarray(position_xy, dtype=np.float64)
        self.velocity_xy = np.asarray(velocity_xy, dtype=np.float64)
        if self.position_xy.shape != self.velocity_xy.shape:
            raise ValueError("position and velocity shapes must match")
        if self.position_xy.shape[1:] != (SMD_SUPPORT_POINTS, 2):
            raise ValueError(f"expected (agents, 64, 2), got {self.position_xy.shape}")
        self.altitude = float(altitude)

    @property
    def num_agents(self) -> int:
        return self.position_xy.shape[0]

    def evaluate(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return p, v, a for all agents; final support point is held to 5 seconds."""
        t = float(np.clip(t, 0.0, SMD_TRAJECTORY_DURATION_S))
        if t >= SMD_REFERENCE_SPAN_S:
            p_xy = self.position_xy[:, -1].copy()
            v_xy = self.velocity_xy[:, -1].copy()
            a_xy = np.zeros_like(p_xy)
        else:
            segment = min(int(np.floor(t / SMD_REFERENCE_DT_S)), SMD_SUPPORT_POINTS - 2)
            tau = (t - segment * SMD_REFERENCE_DT_S) / SMD_REFERENCE_DT_S
            p0 = self.position_xy[:, segment]
            p1 = self.position_xy[:, segment + 1]
            v0 = self.velocity_xy[:, segment]
            v1 = self.velocity_xy[:, segment + 1]
            h = SMD_REFERENCE_DT_S
            h00 = 2 * tau**3 - 3 * tau**2 + 1
            h10 = tau**3 - 2 * tau**2 + tau
            h01 = -2 * tau**3 + 3 * tau**2
            h11 = tau**3 - tau**2
            p_xy = h00 * p0 + h10 * h * v0 + h01 * p1 + h11 * h * v1
            dh00 = 6 * tau**2 - 6 * tau
            dh10 = 3 * tau**2 - 4 * tau + 1
            dh01 = -6 * tau**2 + 6 * tau
            dh11 = 3 * tau**2 - 2 * tau
            v_xy = (dh00 * p0 + dh10 * h * v0 + dh01 * p1 + dh11 * h * v1) / h
            ddh00 = 12 * tau - 6
            ddh10 = 6 * tau - 4
            ddh01 = -12 * tau + 6
            ddh11 = 6 * tau - 2
            a_xy = (ddh00 * p0 + ddh10 * h * v0 + ddh01 * p1 + ddh11 * h * v1) / (h * h)
        p = np.column_stack((p_xy, np.full(self.num_agents, self.altitude)))
        v = np.column_stack((v_xy, np.zeros(self.num_agents)))
        a = np.column_stack((a_xy, np.zeros(self.num_agents)))
        return p, v, a

    def knot_parity(self) -> dict:
        p_errors = []
        v_errors = []
        for k in range(SMD_SUPPORT_POINTS):
            p, v, _ = self.evaluate(k * SMD_REFERENCE_DT_S)
            p_errors.append(np.max(np.abs(p[:, :2] - self.position_xy[:, k])))
            v_errors.append(np.max(np.abs(v[:, :2] - self.velocity_xy[:, k])))
        return {
            "position_knot_max_error": float(max(p_errors)),
            "velocity_knot_max_error": float(max(v_errors)),
        }


class PositionConsistentHermiteReference(HermiteReference):
    """Hermite reference whose physical derivatives come only from SMD positions.

    ``raw_velocity_xy`` is retained as an audit/planning-metric field, but is
    deliberately not used to construct the physical Hermite curve.
    """

    def __init__(
        self,
        position_xy: np.ndarray,
        raw_velocity_xy: np.ndarray,
        altitude: float = REFERENCE_ALTITUDE_M,
    ):
        position_xy = np.asarray(position_xy, dtype=np.float64)
        raw_velocity_xy = np.asarray(raw_velocity_xy, dtype=np.float64)
        if position_xy.shape != raw_velocity_xy.shape:
            raise ValueError("position and raw velocity shapes must match")
        derived_velocity_xy = position_derived_velocity(position_xy, SMD_REFERENCE_DT_S)
        super().__init__(position_xy, derived_velocity_xy, altitude=altitude)
        self.raw_velocity_xy = raw_velocity_xy
        self.position_derived_velocity_xy = derived_velocity_xy.copy()

    def raw_velocity_mismatch(self) -> dict:
        mismatch = self.raw_velocity_xy - self.position_derived_velocity_xy
        return {
            "raw_velocity_mismatch_rmse": float(np.sqrt(np.mean(mismatch * mismatch))),
            "raw_velocity_mismatch_max_error": float(np.max(np.abs(mismatch))),
        }


def lift_2d_to_3d(position_xy: np.ndarray, velocity_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    position_xy = np.asarray(position_xy, dtype=np.float64)
    velocity_xy = np.asarray(velocity_xy, dtype=np.float64)
    return (
        np.concatenate([position_xy, np.zeros((*position_xy.shape[:-1], 1)) + REFERENCE_ALTITUDE_M], axis=-1),
        np.concatenate([velocity_xy, np.zeros((*velocity_xy.shape[:-1], 1))], axis=-1),
    )
