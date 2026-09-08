"""Safety behavior tests for the Marker-based ROS follower node."""

from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from humanoid_path_follower.ros2_adapter.path_follower_node import (
    PathFollowerNode,
)
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker


def _parameter_overrides() -> list[Parameter]:
    """Return all required parameters for a standalone test node."""
    values = {
        'control_rate': 20.0,
        'topics.robot_pose': '/adapter/pose_marker',
        'topics.path': '/vg/path',
        'topics.cmd_vel': '/cmd_vel',
        'topics.carrot': '/debug/carrot',
        'controller.carrot_index': 1,
        'controller.min_vel_x': -0.20,
        'controller.max_vel_x': 0.30,
        'controller.min_vel_y': -0.15,
        'controller.max_vel_y': 0.15,
        'controller.min_vel_theta': -0.80,
        'controller.max_vel_theta': 0.80,
        'controller.translation_gain': 1.5,
        'controller.rotation_kp': 2.0,
        'controller.orient_to_goal_distance': 0.3,
        'controller.position_tolerance': 0.05,
        'controller.orientation_tolerance': 0.087,
        'controller.smoothing_tau': 0.04,
        'safety.pose_timeout': 0.5,
        'safety.path_timeout': 0.5,
    }
    return [Parameter(name, value=value) for name, value in values.items()]


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize one isolated ROS context for this test module."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def follower_node():
    """Create a follower node with mocked output publishers."""
    node = PathFollowerNode(parameter_overrides=_parameter_overrides())
    node._cmd_publisher = Mock()
    node._carrot_publisher = Mock()
    yield node
    Node.destroy_node(node)


def _robot_marker(frame_id: str = 'map') -> Marker:
    """Return a valid identity-oriented robot Marker."""
    message = Marker()
    message.header.frame_id = frame_id
    message.pose.orientation.w = 1.0
    return message


def _path_message(frame_id: str = 'map') -> Path:
    """Return a valid one-waypoint path in the requested frame."""
    message = Path()
    message.header.frame_id = frame_id
    goal = PoseStamped()
    goal.pose.position.x = 1.0
    goal.pose.orientation.w = 1.0
    message.poses = [goal]
    return message


def _assert_zero_twist(message) -> None:
    """Assert that every commanded planar velocity is exactly zero."""
    assert message.linear.x == 0.0
    assert message.linear.y == 0.0
    assert message.angular.z == 0.0


def test_valid_inputs_publish_command_and_carrot(follower_node):
    """Matching fresh inputs must run the core and publish its carrot."""
    follower_node._on_robot_pose(_robot_marker())
    follower_node._on_path(_path_message())
    follower_node._cmd_publisher.reset_mock()

    follower_node._on_timer()

    command = follower_node._cmd_publisher.publish.call_args.args[0]
    assert command.linear.x > 0.0
    carrot = follower_node._carrot_publisher.publish.call_args.args[0]
    assert carrot.header.frame_id == 'map'
    assert carrot.point.x == 1.0


def test_frame_mismatch_publishes_zero_twist(follower_node):
    """A robot/path frame mismatch must stop instead of controlling."""
    follower_node._on_robot_pose(_robot_marker('map'))
    follower_node._cmd_publisher.reset_mock()

    follower_node._on_path(_path_message('odom'))

    command = follower_node._cmd_publisher.publish.call_args.args[0]
    _assert_zero_twist(command)


def test_marker_deletion_publishes_zero_twist(follower_node):
    """Deleting the robot Marker must immediately clear stale motion."""
    follower_node._on_robot_pose(_robot_marker())
    follower_node._on_path(_path_message())
    follower_node._cmd_publisher.reset_mock()
    deletion = Marker()
    deletion.action = Marker.DELETE

    follower_node._on_robot_pose(deletion)

    command = follower_node._cmd_publisher.publish.call_args.args[0]
    _assert_zero_twist(command)
    assert follower_node._robot_pose is None


def test_empty_path_publishes_zero_twist(follower_node):
    """Receiving an empty path must immediately stop the robot."""
    follower_node._on_robot_pose(_robot_marker())
    follower_node._cmd_publisher.reset_mock()

    follower_node._on_path(Path())

    command = follower_node._cmd_publisher.publish.call_args.args[0]
    _assert_zero_twist(command)


@pytest.mark.parametrize('stale_input', ['pose', 'path'])
def test_stale_pose_or_path_publishes_zero_twist(
    follower_node,
    stale_input,
):
    """Either input exceeding its timeout must stop control output."""
    follower_node._on_robot_pose(_robot_marker())
    follower_node._on_path(_path_message())
    follower_node._cmd_publisher.reset_mock()
    if stale_input == 'pose':
        follower_node._robot_received_at -= 1.0
    else:
        follower_node._path_received_at -= 1.0

    follower_node._on_timer()

    command = follower_node._cmd_publisher.publish.call_args.args[0]
    _assert_zero_twist(command)
