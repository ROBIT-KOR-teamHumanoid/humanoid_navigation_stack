"""ROS 2 adapter for Marker-based follow-the-carrot control."""

from __future__ import annotations

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from .message_conversion import (
    carrot_to_point_stamped,
    marker_to_pose2d,
    path_to_poses,
    velocity_command_to_twist,
)
from ..core.controller import FollowTheCarrotController
from ..core.types import Pose2D, VelocityCommand
from ..parameters import load_parameters


class PathFollowerNode(Node):
    """Follow map-frame paths using the robot pose Marker without TF."""

    def __init__(self, *, parameter_overrides=None) -> None:
        """Create subscriptions, publishers, timer, and controller state."""
        super().__init__(
            'path_follower',
            parameter_overrides=parameter_overrides,
        )
        self.config = load_parameters(self)
        self.controller = FollowTheCarrotController(
            self.config.controller
        )

        self._robot_pose: Pose2D | None = None
        self._robot_frame_id = ''
        self._robot_received_at: float | None = None
        self._path: tuple[Pose2D, ...] = ()
        self._path_frame_id = ''
        self._path_received_at: float | None = None
        self._last_control_at: float | None = None
        self._last_frame_warning: tuple[str, str] | None = None

        self._cmd_publisher = self.create_publisher(
            Twist,
            self.config.topics.cmd_vel,
            10,
        )
        self._carrot_publisher = self.create_publisher(
            PointStamped,
            self.config.topics.carrot,
            10,
        )
        self.create_subscription(
            Marker,
            self.config.topics.robot_pose,
            self._on_robot_pose,
            10,
        )
        self.create_subscription(
            Path,
            self.config.topics.path,
            self._on_path,
            10,
        )
        self.create_timer(
            1.0 / self.config.control_rate,
            self._on_timer,
        )
        self.get_logger().info(
            'Path follower ready: '
            f'{self.config.topics.robot_pose} + '
            f'{self.config.topics.path} -> '
            f'{self.config.topics.cmd_vel}'
        )

    def _on_robot_pose(self, message: Marker) -> None:
        """Store the latest Marker pose or stop after a deletion."""
        if message.action in (Marker.DELETE, Marker.DELETEALL):
            self._clear_robot_pose()
            self._stop()
            return

        try:
            self._robot_pose = marker_to_pose2d(message)
        except ValueError as error:
            self.get_logger().warning(
                f'Ignoring invalid robot pose Marker: {error}'
            )
            self._clear_robot_pose()
            self._stop()
            return

        self._robot_frame_id = message.header.frame_id
        self._robot_received_at = self._now_seconds()
        if self._path and not self._check_frames():
            self._stop()

    def _on_path(self, message: Path) -> None:
        """Replace the active path and safely reset controller history."""
        try:
            path = tuple(path_to_poses(message))
        except ValueError as error:
            self.get_logger().warning(
                f'Ignoring path with invalid orientation: {error}'
            )
            path = ()

        self._path = path
        self._path_frame_id = message.header.frame_id
        self._path_received_at = self._now_seconds()
        self.controller.reset()
        self._last_control_at = None
        if not self._path or not self._check_frames():
            self._publish_zero()

    def _on_timer(self) -> None:
        """Publish one safe command at the configured control rate."""
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1.0e9
        if not self._inputs_ready(now_seconds):
            self._stop()
            return

        dt = None
        if self._last_control_at is not None:
            dt = max(0.0, now_seconds - self._last_control_at)
        self._last_control_at = now_seconds

        result = self.controller.step(
            self._robot_pose,
            self._path,
            dt=dt,
        )
        self._cmd_publisher.publish(
            velocity_command_to_twist(result.command)
        )
        if result.carrot is not None:
            self._carrot_publisher.publish(
                carrot_to_point_stamped(
                    result.carrot,
                    self._path_frame_id,
                    now.to_msg(),
                )
            )

    def _inputs_ready(self, now_seconds: float) -> bool:
        """Return whether both inputs are present, compatible, and fresh."""
        if self._robot_pose is None or not self._path:
            return False
        if not self._check_frames():
            return False
        if (
            self._robot_received_at is None
            or self._path_received_at is None
        ):
            return False
        if (
            now_seconds - self._robot_received_at
            > self.config.safety.pose_timeout
        ):
            return False
        if (
            now_seconds - self._path_received_at
            > self.config.safety.path_timeout
        ):
            return False
        return True

    def _check_frames(self) -> bool:
        """Warn once for an incompatible robot/path frame pair."""
        if self._robot_pose is None or not self._path:
            return False
        frames = (self._robot_frame_id, self._path_frame_id)
        if frames[0] and frames[0] == frames[1]:
            self._last_frame_warning = None
            return True
        if frames != self._last_frame_warning:
            self.get_logger().warning(
                'Robot pose and path frame mismatch: '
                f'{frames[0]!r} != {frames[1]!r}'
            )
            self._last_frame_warning = frames
        return False

    def _clear_robot_pose(self) -> None:
        """Remove all cached robot pose state."""
        self._robot_pose = None
        self._robot_frame_id = ''
        self._robot_received_at = None

    def _stop(self) -> None:
        """Reset controller history and publish an exact zero command."""
        self.controller.reset()
        self._last_control_at = None
        self._publish_zero()

    def _publish_zero(self) -> None:
        """Publish an exact zero velocity command."""
        self._cmd_publisher.publish(
            velocity_command_to_twist(VelocityCommand())
        )

    def _now_seconds(self) -> float:
        """Return the node clock time as floating-point seconds."""
        return self.get_clock().now().nanoseconds / 1.0e9

    def destroy_node(self):
        """Stop the robot before releasing ROS node resources."""
        self._publish_zero()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS path follower node."""
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.try_shutdown()
