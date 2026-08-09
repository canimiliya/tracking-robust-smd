import numpy as np

from tr_smd.evaluation.execution_gap import ROBOT_RADIUS_M, clearance_traces


def test_strict_pair_clearance_boundary():
    position = np.zeros((2, 2, 3))
    position[1, :, 0] = 2.0 * ROBOT_RADIUS_M
    result = clearance_traces(position, [])
    assert result["minimum"] == 0.0
    assert not result["agent_agent_collision"]


def test_obstacle_collision_taxonomy():
    position = np.zeros((1, 2, 3))
    result = clearance_traces(position, [([0.0, 0.0], 0.01)])
    assert result["minimum"] < 0.0
    assert result["agent_obstacle_collision"]
    assert result["minimum_type"] == "agent-obstacle"
