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

    assert select_carrot([only_pose], 3) == only_pose
    assert select_carrot([], 1) is None


def test_map_vector_is_rotated_into_robot_frame():
    """A map X vector is robot-right when the robot faces map Y."""
    local_x, local_y = map_vector_to_local(
        1.0,
        0.0,
        math.pi / 2.0,
    )

    assert local_x == pytest.approx(0.0, abs=1.0e-12)
    assert local_y == pytest.approx(-1.0)
