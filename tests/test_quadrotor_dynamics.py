import numpy as np

from tr_smd.dynamics.quadrotor import QuadrotorDynamics, QuadrotorParameters, QuadrotorState, WrenchAllocator


def test_free_fall():
    params = QuadrotorParameters()
    dynamics = QuadrotorDynamics(params)
    state = QuadrotorState.from_position(np.array([0.0, 0.0, 1.0])).as_vector()
    derivative = dynamics.derivative(state, np.zeros(4))
    assert abs(derivative[5] + 9.81) <= 1e-8


def test_hover_wrench_and_rank():
    params = QuadrotorParameters()
    allocator = WrenchAllocator(params)
    assert allocator.rank == 4
    thrust = np.full(4, params.mass * params.gravity / 4.0)
    wrench = allocator.matrix @ thrust
    derivative = QuadrotorDynamics(params).derivative(QuadrotorState.from_position([0, 0, 1]).as_vector(), wrench)
    assert np.linalg.norm(derivative[3:6]) <= 1e-12
    assert np.linalg.norm(derivative[10:13]) <= 1e-12


def test_allocation_signs_and_reconstruction():
    params = QuadrotorParameters()
    allocator = WrenchAllocator(params)
    for idx in range(4):
        f = np.full(4, params.mass * params.gravity / 4.0)
        f[idx] += 1e-5
        actual = allocator.matrix @ f
        assert abs(actual[1]) > 0 or abs(actual[2]) > 0 or abs(actual[3]) > 0
        recovered = np.linalg.solve(allocator.matrix, actual)
        assert np.max(np.abs(recovered - f)) <= 1e-10


def test_quaternion_normalization_after_rk4():
    dynamics = QuadrotorDynamics()
    state = QuadrotorState.from_position([0, 0, 1]).as_vector()
    out = dynamics.rk4_step(state, np.array([dynamics.params.mass * dynamics.params.gravity, 0, 0, 0]), 0.002)
    assert abs(np.linalg.norm(out[6:10]) - 1.0) <= 1e-8


def test_zero_disturbance_interface_is_exact():
    dynamics = QuadrotorDynamics()
    state = QuadrotorState.from_position(np.array([0.2, -0.1, 1.0])).as_vector()
    wrench = np.array([dynamics.params.mass * dynamics.params.gravity, 0.0, 0.0, 0.0])
    baseline = dynamics.rk4_step(state, wrench, 0.002)
    with_zero = dynamics.rk4_step(state, wrench, 0.002, np.zeros(3), np.zeros(3))
    assert np.array_equal(baseline, with_zero)


def test_external_force_acceleration_direction():
    dynamics = QuadrotorDynamics()
    state = QuadrotorState.from_position(np.zeros(3)).as_vector()
    derivative = dynamics.derivative(state, np.zeros(4), external_force_world=np.array([0.01, 0.0, 0.0]))
    assert derivative[3] == 0.01 / dynamics.params.mass
