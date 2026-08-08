"""Geometric SE(3) controller with frozen nominal gains."""

from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np

from tr_smd.dynamics.quadrotor import QuadrotorParameters, WrenchAllocator, quat_to_rotation


def _vee(M: np.ndarray) -> np.ndarray:
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


@dataclass(frozen=True)
class ControllerConfig:
    revision_count: int = 3
    config_frozen: bool = True
    position_gains: tuple[float, float, float] = (12.0, 12.0, 14.0)
    velocity_gains: tuple[float, float, float] = (7.0, 7.0, 8.0)
    attitude_gains: tuple[float, float, float] = (6.0e-3, 6.0e-3, 3.0e-3)
    angular_rate_gains: tuple[float, float, float] = (3.0e-4, 3.0e-4, 1.5e-4)
    reference_altitude: float = 1.0
    reference_yaw: float = 0.0

    def as_dict(self) -> dict:
        return asdict(self)


def desired_rotation(force_world: np.ndarray, yaw: float) -> np.ndarray:
    norm = np.linalg.norm(force_world)
    if norm < 1e-10:
        raise FloatingPointError("desired force is too small")
    b3 = force_world / norm
    b1_yaw = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    b2 = np.cross(b3, b1_yaw)
    b2_norm = np.linalg.norm(b2)
    if b2_norm < 1e-10:
        b1_yaw = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
        b2 = np.cross(b3, b1_yaw)
        b2_norm = np.linalg.norm(b2)
    b2 /= b2_norm
    b1 = np.cross(b2, b3)
    return np.column_stack((b1, b2, b3))


class GeometricController:
    def __init__(self, params: QuadrotorParameters | None = None, config: ControllerConfig | None = None):
        self.params = params or QuadrotorParameters()
        self.config = config or ControllerConfig()
        self.allocator = WrenchAllocator(self.params)

    def command(self, state: np.ndarray, p_ref: np.ndarray, v_ref: np.ndarray, a_ref: np.ndarray, yaw_ref: float | None = None):
        p = np.asarray(state[0:3], dtype=float)
        v = np.asarray(state[3:6], dtype=float)
        q = np.asarray(state[6:10], dtype=float)
        omega = np.asarray(state[10:13], dtype=float)
        p_ref = np.asarray(p_ref, dtype=float)
        v_ref = np.asarray(v_ref, dtype=float)
        a_ref = np.asarray(a_ref, dtype=float)
        kp = np.asarray(self.config.position_gains)
        kd = np.asarray(self.config.velocity_gains)
        force = self.params.mass * (a_ref + kp * (p_ref - p) + kd * (v_ref - v) - np.array([0.0, 0.0, -self.params.gravity]))
        R = quat_to_rotation(q)
        yaw = self.config.reference_yaw if yaw_ref is None else float(yaw_ref)
        R_des = desired_rotation(force, yaw)
        e_R = 0.5 * _vee(R_des.T @ R - R.T @ R_des)
        moment = -np.asarray(self.config.attitude_gains) * e_R - np.asarray(self.config.angular_rate_gains) * omega
        moment += np.cross(omega, self.params.inertia_matrix @ omega)
        thrust = float(force @ R[:, 2])
        desired_wrench = np.concatenate(([thrust], moment))
        thrust_cmd, thrust_clipped, actual_wrench = self.allocator.allocate(desired_wrench)
        return {
            "desired_wrench": desired_wrench,
            "actual_wrench": actual_wrench,
            "thrust_cmd": thrust_cmd,
            "thrust": thrust_clipped,
            "R_des": R_des,
            "force_des": force,
        }
