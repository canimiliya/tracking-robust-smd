import numpy as np

from tr_smd.evaluation.execution_gap import simulate
from tr_smd.tracking.smd_reference import PositionConsistentHermiteReference, SMD_SUPPORT_POINTS
from tr_smd.tubes.error_state import minimal_error_state, quaternion_log_error
from tr_smd.tubes.scenario_tube import (
    ScenarioSpec,
    _conservative_support_mapping,
    disturbance_schedule,
    simulate_scheduled,
)


def stationary_reference():
    position = np.zeros((1, SMD_SUPPORT_POINTS, 2))
    velocity = np.zeros_like(position)
    return PositionConsistentHermiteReference(position, velocity)


def test_minimal_error_state_zero_and_rotation():
    state = np.array([0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], dtype=float)
    assert np.array_equal(minimal_error_state(state, state), np.zeros(12))
    angle = 1e-4
    rotated = np.array([np.cos(angle / 2), np.sin(angle / 2), 0, 0])
    error = quaternion_log_error(rotated, state[6:10])
    assert np.allclose(error, [angle, 0, 0], atol=1e-12)


def test_disturbance_schedules_are_bounded_and_deterministic():
    spec = ScenarioSpec("test", "independent_piecewise", seed=42, switch_ticks=25)
    first = disturbance_schedule(spec, 3, 0.005)
    second = disturbance_schedule(spec, 3, 0.005)
    assert np.array_equal(first, second)
    assert np.max(np.linalg.norm(first[..., :2], axis=-1)) <= 0.005 + 1e-12
    assert np.array_equal(first[..., 2], np.zeros(first.shape[:2]))


def test_scheduled_simulator_matches_s4_constant_force_transition():
    reference = stationary_reference()
    force = np.array([0.005, 0.0, 0.0])
    baseline, _ = simulate(reference, external_force_world=force)
    schedule = disturbance_schedule(ScenarioSpec("pos_x", "common_fixed", 0.0), 1, 0.005)
    scheduled, _ = simulate_scheduled(reference, schedule)
    for key in ("nominal_position", "actual_position", "actual_velocity", "actual_quaternion", "actual_body_rate", "rotor_thrust"):
        assert np.array_equal(baseline[key], scheduled[key])


def test_support_and_segment_mapping_never_undersample_dense_peak():
    time = np.arange(2501) * 0.002
    dense = np.zeros((2, len(time)))
    dense[0, 57] = 0.7
    dense[1, 1876] = 0.9
    support, segment = _conservative_support_mapping(time, dense)
    assert np.max(support[0]) == 0.7
    assert np.max(support[1]) == 0.9
    assert np.max(segment[0]) == 0.7
    assert np.max(segment[1]) == 0.9
