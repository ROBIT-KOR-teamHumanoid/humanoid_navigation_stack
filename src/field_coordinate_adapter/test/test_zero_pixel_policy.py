"""Tests for configurable zero-pixel handling in the ROS adapter."""

from types import SimpleNamespace
from unittest.mock import Mock

from field_coordinate_adapter.converter import FieldCoordinateConverter
from field_coordinate_adapter.node import FieldCoordinateAdapterNode
import pytest
from rclpy.clock import Clock
from visualization_msgs.msg import Marker


def _adapter(zero_pixel_is_invalid: bool):
    """Return a lightweight adapter object with mocked publishers."""
    node = SimpleNamespace(
        parameters=SimpleNamespace(
            output_frame_id='map',
            zero_pixel_is_invalid=zero_pixel_is_invalid,
        ),
        _converter=FieldCoordinateConverter(
            width_px=1100.0,
            height_px=800.0,
            width_m=11.0,
            height_m=8.0,
            yaw_offset_deg=90.0,
        ),
        _robot_pub=Mock(),
        _ball_pub=Mock(),
        _obstacles_pub=Mock(),
        _target_pub=Mock(),
    )
    node._invalid_pixel_position = lambda x, y: (
        FieldCoordinateAdapterNode._invalid_pixel_position(node, x, y)
    )
    node._new_marker = lambda stamp, namespace, marker_id: (
        FieldCoordinateAdapterNode._new_marker(
            node,
            stamp,
            namespace,
            marker_id,
        )
    )
    return node


@pytest.mark.parametrize(
    ('zero_pixel_is_invalid', 'expected_action', 'obstacle_count'),
    ((False, Marker.ADD, 2), (True, Marker.DELETE, 1)),
)
def test_zero_pixel_policy_applies_to_every_input(
    zero_pixel_is_invalid,
    expected_action,
    obstacle_count,
) -> None:
    """Robot, ball, target, and obstacles must share one zero policy."""
    node = _adapter(zero_pixel_is_invalid)
    stamp = Clock().now().to_msg()
    localization = SimpleNamespace(
        robot_x=0.0,
        robot_y=0.0,
        ball_x=0.0,
        ball_y=0.0,
        obstacles_x=[0.0],
        obstacles_y=[0.0],
    )
    target = SimpleNamespace(
        targetx=0,
        targety=0,
        angle_to_target=0.0,
    )

    FieldCoordinateAdapterNode._publish_robot(
        node,
        localization,
        None,
        stamp,
    )
    FieldCoordinateAdapterNode._publish_ball(node, localization, stamp)
    FieldCoordinateAdapterNode._publish_obstacles(node, localization, stamp)
    FieldCoordinateAdapterNode._publish_target(node, target, stamp)

    robot = node._robot_pub.publish.call_args.args[0]
    ball = node._ball_pub.publish.call_args.args[0]
    obstacles = node._obstacles_pub.publish.call_args.args[0]
    target_marker = node._target_pub.publish.call_args.args[0]
    assert robot.action == expected_action
    assert ball.action == expected_action
    assert target_marker.action == expected_action
    assert len(obstacles.markers) == obstacle_count
    if not zero_pixel_is_invalid:
        assert (robot.pose.position.x, robot.pose.position.y) == (-5.5, 4.0)
        assert (ball.pose.position.x, ball.pose.position.y) == (-5.5, 4.0)
        assert (
            obstacles.markers[1].pose.position.x,
            obstacles.markers[1].pose.position.y,
        ) == (-5.5, 4.0)
        assert (
            target_marker.pose.position.x,
            target_marker.pose.position.y,
        ) == (-5.5, 4.0)


def test_missing_target_is_deleted_when_zero_pixel_is_valid() -> None:
    """Message absence must remain distinct from a valid zero position."""
    node = _adapter(False)

    FieldCoordinateAdapterNode._publish_target(
        node,
        None,
        Clock().now().to_msg(),
    )

    marker = node._target_pub.publish.call_args.args[0]
    assert marker.action == Marker.DELETE
