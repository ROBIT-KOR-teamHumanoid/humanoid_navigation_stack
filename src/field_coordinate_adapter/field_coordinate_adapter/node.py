"""ROS 2 node entry point for the field coordinate adapter."""

from field_coordinate_adapter.converter import FieldCoordinateConverter
from field_coordinate_adapter.parameters import load_parameters
from humanoid_interfaces.msg import ImuMsg, Master2localization
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robocup_localization.msg import RobocupLocalization
from visualization_msgs.msg import Marker, MarkerArray


class FieldCoordinateAdapterNode(Node):
    """Convert localization pixels and publish planner-compatible markers."""

    def __init__(self) -> None:
        """Create subscriptions, publishers, converter, and timer."""
        super().__init__('field_coordinate_adapter')

        self.parameters = load_parameters(self)
        self._converter = FieldCoordinateConverter(
            width_px=self.parameters.field_width_px,
            height_px=self.parameters.field_height_px,
            width_m=self.parameters.field_width_m,
            height_m=self.parameters.field_height_m,
            yaw_offset_deg=self.parameters.yaw_offset_deg,
        )

        self._latest_localization = None
        self._latest_imu = None
        self._latest_target = None

        self._localization_sub = self.create_subscription(
            RobocupLocalization,
            self.parameters.topic_localization,
            self._on_localization,
            1,
        )
        self._imu_sub = self.create_subscription(
            ImuMsg,
            self.parameters.topic_imu,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self._target_sub = self.create_subscription(
            Master2localization,
            self.parameters.topic_input_target,
            self._on_target,
            1,
        )

        self._robot_pub = self.create_publisher(
            Marker,
            self.parameters.topic_robot,
            10,
        )
        self._obstacles_pub = self.create_publisher(
            MarkerArray,
            self.parameters.topic_obstacles,
            10,
        )
        self._ball_pub = self.create_publisher(
            Marker,
            self.parameters.topic_ball,
            10,
        )
        self._target_pub = self.create_publisher(
            Marker,
            self.parameters.topic_output_target,
            10,
        )

        self._timer = self.create_timer(
            1.0 / self.parameters.hz,
            self._on_timer,
        )

        self.get_logger().info(
            f'Field coordinate adapter started at '
            f'{self.parameters.hz:.1f} Hz'
        )

    def _on_localization(self, msg: RobocupLocalization) -> None:
        """Store the latest localization message."""
        self._latest_localization = msg

    def _on_imu(self, msg: ImuMsg) -> None:
        """Store the latest IMU message."""
        self._latest_imu = msg

    def _on_target(self, msg: Master2localization) -> None:
        """Store the latest target message."""
        self._latest_target = msg

    def _on_timer(self) -> None:
        """Convert and publish the latest localization snapshot."""
        localization = self._latest_localization
        target = self._latest_target

        stamp = self.get_clock().now().to_msg()
        if localization is not None:
            self._publish_robot(localization, self._latest_imu, stamp)
            self._publish_ball(localization, stamp)
            self._publish_obstacles(localization, stamp)
        self._publish_target(target, stamp)

    def _publish_robot(self, localization, imu, stamp) -> None:
        """Publish the converted robot marker."""
        marker = self._new_marker(stamp, 'robot', 0)
        robot_x = float(localization.robot_x)
        robot_y = float(localization.robot_y)

        if self._invalid_pixel_position(robot_x, robot_y):
            marker.action = Marker.DELETE
            self._robot_pub.publish(marker)
            return

        x, y = self._converter.to_field(robot_x, robot_y)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y

        if imu is None:
            marker.pose.orientation.w = 1.0
        else:
            quaternion = self._converter.yaw_to_quaternion(float(imu.yaw))
            (
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w,
            ) = quaternion

        marker.scale.x = 0.35
        marker.scale.y = 0.05
        marker.scale.z = 0.08
        marker.color.r = 0.2
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 1.0
        self._robot_pub.publish(marker)

    def _publish_ball(self, localization, stamp) -> None:
        """Publish the converted ball marker."""
        marker = self._new_marker(stamp, 'ball', 0)
        ball_x = float(localization.ball_x)
        ball_y = float(localization.ball_y)

        if self._invalid_pixel_position(ball_x, ball_y):
            marker.action = Marker.DELETE
            self._ball_pub.publish(marker)
            return

        x, y = self._converter.to_field(ball_x, ball_y)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self._ball_pub.publish(marker)

    def _publish_obstacles(self, localization, stamp) -> None:
        """Publish converted obstacle markers and clear stale markers."""
        marker_array = MarkerArray()
        clear = self._new_marker(stamp, 'obstacles', 0)
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for marker_id, (pixel_x, pixel_y) in enumerate(zip(
            localization.obstacles_x,
            localization.obstacles_y,
        )):
            if self._invalid_pixel_position(pixel_x, pixel_y):
                continue

            x, y = self._converter.to_field(float(pixel_x), float(pixel_y))
            marker = self._new_marker(stamp, 'obstacles', marker_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.10
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self._obstacles_pub.publish(marker_array)

    def _publish_target(self, target, stamp) -> None:
        """Publish the converted target marker."""
        marker = self._new_marker(stamp, 'target', 0)

        if target is None:
            marker.action = Marker.DELETE
            self._target_pub.publish(marker)
            return

        target_x = float(target.targetx)
        target_y = float(target.targety)
        if self._invalid_pixel_position(target_x, target_y):
            marker.action = Marker.DELETE
            self._target_pub.publish(marker)
            return

        x, y = self._converter.to_field(
            target_x,
            target_y,
        )
        quaternion = self._converter.yaw_to_quaternion(
            float(target.angle_to_target)
        )

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = quaternion

        marker.scale.x = 0.30
        marker.scale.y = 0.05
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        self._target_pub.publish(marker)

    def _invalid_pixel_position(self, x: float, y: float) -> bool:
        """Apply the configured zero-pixel policy to every position."""
        return (
            self.parameters.zero_pixel_is_invalid
            and x == 0.0
            and y == 0.0
        )

    def _new_marker(self, stamp, namespace: str, marker_id: int) -> Marker:
        """Create a marker with the common header and identity orientation."""
        marker = Marker()
        marker.header.frame_id = self.parameters.output_frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.pose.orientation.w = 1.0
        return marker


def main(args=None) -> None:
    """Run the field coordinate adapter node."""
    rclpy.init(args=args)
    node = FieldCoordinateAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
