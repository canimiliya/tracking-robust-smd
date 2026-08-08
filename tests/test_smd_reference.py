from pathlib import Path

import numpy as np

from tr_smd.tracking.smd_reference import (
    HermiteReference,
    PositionConsistentHermiteReference,
    SMD_REFERENCE_DT_S,
    SMD_SUPPORT_POINTS,
    position_derived_velocity,
)


def test_timing_and_knot_parity():
    position = np.zeros((3, SMD_SUPPORT_POINTS, 2))
    velocity = np.zeros_like(position)
    reference = HermiteReference(position, velocity)
    parity = reference.knot_parity()
    assert SMD_REFERENCE_DT_S == 0.078125
    assert parity["position_knot_max_error"] <= 1e-7
    assert parity["velocity_knot_max_error"] <= 1e-6


def test_primary_reference_shape():
    p = Path("scripts/inference/results_s1_r0_3agent/2026-08-08-20-09-42/instance_name___EnvEmptyNoWait2DRobotCompositeThreePlanarDiskRandom/num_agents___3/planner___SMDComposite/single_agent_planner___SMDEnsemble/0/paths.npy")
    assert np.load(p).shape == (64, 64, 12)


def test_position_authoritative_derivative_contract():
    position = np.zeros((1, SMD_SUPPORT_POINTS, 2))
    position[0, :, 0] = np.arange(SMD_SUPPORT_POINTS, dtype=float)
    raw_velocity = np.full_like(position, 99.0)
    derived = position_derived_velocity(position, SMD_REFERENCE_DT_S)
    assert np.allclose(derived[:, 0], 0.0)
    assert np.allclose(derived[:, -1], 0.0)
    assert np.allclose(derived[:, 1:-1, 0], 1.0 / SMD_REFERENCE_DT_S)

    reference = PositionConsistentHermiteReference(position, raw_velocity)
    assert reference.raw_velocity_xy[0, 0, 0] == 99.0
    assert reference.velocity_xy[0, 0, 0] == 0.0
    assert reference.evaluate(4.921875)[1][0].tolist() == [0.0, 0.0, 0.0]
    assert reference.evaluate(5.0)[2][0].tolist() == [0.0, 0.0, 0.0]
