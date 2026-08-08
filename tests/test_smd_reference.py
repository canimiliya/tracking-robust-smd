from pathlib import Path

import numpy as np

from tr_smd.tracking.smd_reference import HermiteReference, SMD_REFERENCE_DT_S, SMD_SUPPORT_POINTS


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
