"""Synthetic-only controller gain audit permitted before SMD smoke freeze."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
import run_closed_loop_smoke as smoke

from tr_smd.control.geometric_controller import ControllerConfig, GeometricController
from tr_smd.dynamics.quadrotor import QuadrotorDynamics, QuadrotorParameters
from tr_smd.evaluation.closed_loop_metrics import compute_execution_metrics
from tr_smd.tracking.smd_reference import HermiteReference, SMD_SUPPORT_POINTS


def run(target, config):
    target = np.tile(np.asarray(target, dtype=float), (1, SMD_SUPPORT_POINTS, 1))
    reference = HermiteReference(target, np.zeros_like(target))
    params = QuadrotorParameters()
    data = smoke.simulate(reference, GeometricController(params, config), QuadrotorDynamics(params), [([0.0, 0.0], [0.0, 0.0], 0.05)])
    return compute_execution_metrics(data["time"], data["nominal_position"], data["actual_position"], data["actual_velocity"], data["actual_quaternion"], data["rotor_thrust"], [], 0.05)


def main():
    candidates = [
        ControllerConfig(revision_count=1),
        ControllerConfig(revision_count=2, position_gains=(8.0, 8.0, 10.0), velocity_gains=(5.0, 5.0, 6.0), attitude_gains=(4.0e-3, 4.0e-3, 2.0e-3), angular_rate_gains=(2.0e-4, 2.0e-4, 1.0e-4)),
        ControllerConfig(revision_count=3, position_gains=(12.0, 12.0, 14.0), velocity_gains=(7.0, 7.0, 8.0), attitude_gains=(6.0e-3, 6.0e-3, 3.0e-3), angular_rate_gains=(3.0e-4, 3.0e-4, 1.5e-4)),
    ]
    for config in candidates:
        hover = run([0.0, 0.0], config)
        x = run([0.10, 0.0], config)
        y = run([0.0, 0.10], config)
        print({"revision": config.revision_count, "hover_z_rmse": hover["z_rmse_m"], "x_final": x["per_agent_final_xy_error_m"][0], "y_final": y["per_agent_final_xy_error_m"][0], "x_tilt": x["max_tilt_deg"], "y_tilt": y["max_tilt_deg"], "finite": hover["finite_state"] and x["finite_state"] and y["finite_state"]})


if __name__ == "__main__":
    main()
