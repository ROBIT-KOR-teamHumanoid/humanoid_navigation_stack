"""Tests for conversions between ROS messages and core types."""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from humanoid_path_follower.core.types import Pose2D, VelocityCommand
from humanoid_path_follower.ros2_adapter.message_conversion import (
    carrot_to_point_stamped,
    marker_to_pose2d,
    path_to_poses,
    velocity_command_to_twist,
)
from nav_msgs.msg import Path
import pytest
from visualization_msgs.msg import Marker


def _set_yaw(orientation, yaw: float) -> None:
    """Fill a geometry quaternion with a planar yaw rotation."""
    orientation.z = math.sin(yaw / 2.0)
    orientation.w = math.cos(yaw / 2.0)


def test_marker_position_and_quaternion_convert_to_pose2d():
    """Marker position and quaternion must preserve XY and yaw."""
    marker = Marker()
    marker.pose.position.x = 1.25
    marker.pose.position.y = -0.75
    _set_yaw(marker.pose.orientation, math.pi / 2.0)

    pose = marker_to_pose2d(marker)

    assert pose.x == 1.25
    assert pose.y == -0.75
    assert pose.yaw == pytest.approx(math.pi / 2.0)


def test_zero_quaternion_is_rejected():
    """An all-zero quaternion must not become a valid robot heading."""
    marker = Marker()
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0

    with pytest.raises(ValueError, match='zero quaternion'):
        marker_to_pose2d(marker)


def test_path_conversion_preserves_final_yaw():
    """Path conversion must retain positions and goal orientation."""
    message = Path()
    first = PoseStamped()
    first.pose.orientation.w = 1.0
    final = PoseStamped()
    final.pose.position.x = 2.0
    final.pose.position.y = 1.0
    _set_yaw(final.pose.orientation, -0.4)
    message.poses = [first, final]

    poses = path_to_poses(message)

    assert poses[0] == Pose2D(0.0, 0.0, 0.0)
    assert poses[1].x == 2.0
    assert poses[1].y == 1.0
    assert poses[1].yaw == pytest.approx(-0.4)


def test_command_and_carrot_convert_to_output_messages():
    """Core outputs must populate Twist and PointStamped fields."""
    twist = velocity_command_to_twist(
        VelocityCommand(0.2, -0.1, 0.3)
    )
    stamp = Time(sec=12, nanosec=34)
    carrot = carrot_to_point_stamped(
        Pose2D(1.0, 2.0, 0.0),
        'map',
        stamp,
    )

    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (
        0.2,
        -0.1,
        0.3,
    )
    assert carrot.header.frame_id == 'map'
    assert carrot.header.stamp == stamp
    assert (carrot.point.x, carrot.point.y) == (1.0, 2.0)
