"""Deterministic nonlinear rigid-body CF2X quadrotor model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-15:
        raise FloatingPointError("invalid quaternion norm")
    return q / n


def quat_to_rotation(q: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    ox, oy, oz = omega
    return 0.5 * np.array([
        -x * ox - y * oy - z * oz,
        w * ox + y * oz - z * oy,
        w * oy - x * oz + z * ox,
        w * oz + x * oy - y * ox,
    ])


@dataclass(frozen=True)
class QuadrotorParameters:
    model_source: str = "learnsyslab/gym-pybullet-drones::gym_pybullet_drones/assets/cf2x.urdf"
    model_source_commit: str = "e712698a05a80728b06572819dcf044596707754"
    mass: float = 0.027
    inertia: tuple[float, float, float] = (1.4e-5, 1.4e-5, 2.17e-5)
    arm: float = 0.0397
    kf: float = 3.16e-10
    km: float = 7.94e-12
    thrust_to_weight: float = 2.25
    gravity: float = 9.81
    rotor_positions: tuple[tuple[float, float, float], ...] = (
        (0.028, -0.028, 0.0),
        (-0.028, -0.028, 0.0),
        (-0.028, 0.028, 0.0),
        (0.028, 0.028, 0.0),
    )
    yaw_signs: tuple[float, float, float, float] = (-1.0, 1.0, -1.0, 1.0)

    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag(self.inertia)

    @property
    def total_max_thrust(self) -> float:
        return self.thrust_to_weight * self.mass * self.gravity

    @property
    def rotor_max_thrust(self) -> float:
        return self.total_max_thrust / 4.0


class WrenchAllocator:
    def __init__(self, params: QuadrotorParameters):
        self.params = params
        self.matrix = self._build_matrix()
        self.rank = int(np.linalg.matrix_rank(self.matrix))

    def _build_matrix(self) -> np.ndarray:
        rows = [np.ones(4), [p[1] for p in self.params.rotor_positions], [-p[0] for p in self.params.rotor_positions]]
        rows.append([s * self.params.km / self.params.kf for s in self.params.yaw_signs])
        return np.asarray(rows, dtype=float)

    def allocate(self, wrench: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        desired = np.asarray(wrench, dtype=float)
        thrust_cmd = np.linalg.solve(self.matrix, desired)
        thrust_clipped = np.clip(thrust_cmd, 0.0, self.params.rotor_max_thrust)
        actual_wrench = self.matrix @ thrust_clipped
        return thrust_cmd, thrust_clipped, actual_wrench


@dataclass
class QuadrotorState:
    position: np.ndarray
    velocity: np.ndarray
    quaternion: np.ndarray
    body_rate: np.ndarray

    @classmethod
    def from_position(cls, position: np.ndarray) -> "QuadrotorState":
        return cls(np.asarray(position, dtype=float).copy(), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))

    def as_vector(self) -> np.ndarray:
        return np.concatenate([self.position, self.velocity, self.quaternion, self.body_rate])

    @classmethod
    def from_vector(cls, x: np.ndarray) -> "QuadrotorState":
        x = np.asarray(x, dtype=float)
        if x.shape != (13,):
            raise ValueError(f"expected 13-state vector, got {x.shape}")
        return cls(x[0:3].copy(), x[3:6].copy(), x[6:10].copy(), x[10:13].copy())


class QuadrotorDynamics:
    STATE_DIM = 13

    def __init__(self, params: QuadrotorParameters | None = None):
        self.params = params or QuadrotorParameters()
        self.gravity_world = np.array([0.0, 0.0, -self.params.gravity])

    def derivative(self, state_vector: np.ndarray, actual_wrench: np.ndarray) -> np.ndarray:
        state = QuadrotorState.from_vector(state_vector)
        p = self.params
        R = quat_to_rotation(state.quaternion)
        total_thrust, moment = float(actual_wrench[0]), np.asarray(actual_wrench[1:4], dtype=float)
        acceleration = self.gravity_world + R @ np.array([0.0, 0.0, total_thrust / p.mass])
        qdot = quat_derivative(state.quaternion, state.body_rate)
        J = p.inertia_matrix
        omega_dot = np.linalg.solve(J, moment - np.cross(state.body_rate, J @ state.body_rate))
        return np.concatenate([state.velocity, acceleration, qdot, omega_dot])

    def rk4_step(self, state_vector: np.ndarray, actual_wrench: np.ndarray, dt: float) -> np.ndarray:
        x = np.asarray(state_vector, dtype=float)
        k1 = self.derivative(x, actual_wrench)
        k2 = self.derivative(x + 0.5 * dt * k1, actual_wrench)
        k3 = self.derivative(x + 0.5 * dt * k2, actual_wrench)
        k4 = self.derivative(x + dt * k3, actual_wrench)
        out = x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        out[6:10] = quat_normalize(out[6:10])
        return out
