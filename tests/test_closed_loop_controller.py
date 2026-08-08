import numpy as np

from tr_smd.control.geometric_controller import GeometricController
from tr_smd.dynamics.quadrotor import QuadrotorState


def test_hover_command_is_bounded_and_finite():
    controller = GeometricController()
    state = QuadrotorState.from_position([0, 0, 1]).as_vector()
    command = controller.command(state, [0, 0, 1], [0, 0, 0], [0, 0, 0])
    assert np.isfinite(command["thrust"]).all()
    assert np.max(command["thrust"]) <= controller.params.rotor_max_thrust
    assert np.min(command["thrust"]) >= 0.0
    assert np.linalg.norm(command["actual_wrench"][1:]) <= 1e-12
