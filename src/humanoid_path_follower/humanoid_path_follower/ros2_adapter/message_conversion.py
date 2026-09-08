"""Conversions between ROS messages and follower core types."""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped, Quaternion, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from ..core.types import Pose2D, VelocityCommand

_QUATERNION_EPSILON = 1.0e-12


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """Convert a valid quaternion to planar yaw in radians."""
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm <= _QUATERNION_EPSILON:
        raise ValueError('zero quaternion has no valid orientation')

    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def marker_to_pose2d(message: Marker) -> Pose2D:
    """Convert a robot pose Marker to a core pose."""
    return Pose2D(
        x=float(message.pose.position.x),
        y=float(message.pose.position.y),
        yaw=quaternion_to_yaw(message.pose.orientation),
    )


def path_to_poses(message: Path) -> list[Pose2D]:
    """Convert every pose in a path to core poses."""
    return [
        Pose2D(
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            yaw=quaternion_to_yaw(pose.pose.orientation),
        )
        for pose in message.poses
    ]


def velocity_command_to_twist(command: VelocityCommand) -> Twist:
    """Convert a core velocity command to ``geometry_msgs/Twist``."""
    message = Twist()
    message.linear.x = command.vx
    message.linear.y = command.vy
    message.angular.z = command.wz
    return message


def carrot_to_point_stamped(
    carrot: Pose2D,
    frame_id: str,
    stamp,
) -> PointStamped:
    """Convert a carrot pose to a stamped debug point."""
    message = PointStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.point.x = carrot.x
    message.point.y = carrot.y
    return message
