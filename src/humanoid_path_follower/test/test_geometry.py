"""Tests for ROS-independent planar geometry helpers."""

import math

from humanoid_path_follower.core.geometry import (
    distance,
    map_vector_to_local,
    normalize_angle,
    remaining_path_length,
    select_carrot,
)
from humanoid_path_follower.core.types import Pose2D
import pytest


def test_angle_normalization_wraps_to_pi_interval():
    """Angles outside one revolution must wrap around zero."""
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(0.4) == pytest.approx(0.4)


def test_distance_and_remaining_path_length():
    """Remaining length must include the current-to-path connection."""
    current = Pose2D(0.0, 0.0, 0.0)
    path = [
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(1.0, 1.0, 0.0),
    ]

    assert distance(current, path[0]) == pytest.approx(1.0)
    assert remaining_path_length(current, path) == pytest.approx(2.0)
    assert remaining_path_length(current, []) == 0.0


def test_carrot_selection_uses_last_pose_for_short_path():
    """A path shorter than the requested index must use its last pose."""
    only_pose = Pose2D(1.0, 2.0, 0.3)

    robot = Pose2D(0.0, 0.0, 0.0)
    assert select_carrot([only_pose], 3, robot=robot) == only_pose
    assert select_carrot([], 1, robot=robot) is None


@pytest.mark.parametrize(
    'robot_x, offset, expected_index',
    [
        pytest.param(0.0, 1, 1, id='start'),
        pytest.param(1.9, 1, 3, id='past-first-carrot'),
        pytest.param(2.1, 1, 3, id='middle'),
        pytest.param(3.9, 1, 4, id='near-goal'),
        pytest.param(4.0, 1, 4, id='at-goal'),
        pytest.param(4.2, 1, 4, id='past-goal'),
        pytest.param(1.1, 2, 3, id='relative-offset'),
        pytest.param(2.1, 10, 4, id='offset-past-goal'),
        pytest.param(2.1, 0, 3, id='zero-offset-still-advances'),
        pytest.param(1.5, 1, 3, id='tie-prefers-later-index'),
    ],
)
def test_carrot_advances_from_nearest_waypoint(
    robot_x, offset, expected_index,
):
    """Select a forward offset from current progress instead of path start."""
    path = [Pose2D(float(index), 0.0, 0.0) for index in range(5)]
    robot = Pose2D(robot_x, 0.0, 0.0)

    assert select_carrot(path, offset, robot=robot) is path[expected_index]


def test_carrot_follows_curved_path_order():
    """Forward path progress may decrease both map X and map Y."""
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(2.0, 0.0, 0.0),
        Pose2D(2.0, 2.0, 0.0),
        Pose2D(1.0, 1.0, 0.0),
        Pose2D(-1.0, 1.0, 0.0),
    ]
    robot = Pose2D(2.0, 1.9, math.pi)

    assert select_carrot(path, 1, robot=robot) is path[3]


def test_carrot_rejects_negative_offset():
    """Keep rejecting invalid negative waypoint offsets."""
    robot = Pose2D(0.0, 0.0, 0.0)

    with pytest.raises(ValueError, match='non-negative'):
        select_carrot([robot], -1, robot=robot)


def test_map_vector_is_rotated_into_robot_frame():
    """A map X vector is robot-right when the robot faces map Y."""
    local_x, local_y = map_vector_to_local(
        1.0,
        0.0,
        math.pi / 2.0,
    )

    assert local_x == pytest.approx(0.0, abs=1.0e-12)
    assert local_y == pytest.approx(-1.0)
