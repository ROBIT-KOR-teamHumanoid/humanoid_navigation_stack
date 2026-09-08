"""ROS 2 adapter for the humanoid visibility-graph planner."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point as GeometryPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from ..core.obstacle import Point, RoundObstacle
from ..core.planner import Planner, PlanResult
from ..parameters import FieldParameters, load_parameters

_FIELD_BOUNDARY_MARKER_ID = 1000
_FIELD_BOUNDARY_Z = 0.01


class PathPlanningNode(Node):
    """Keep the existing ROBIT Marker interface and publish shortest paths."""

    def __init__(self) -> None:
        """Create subscriptions, publishers, configuration, and planner."""
        super().__init__('path_planning')
        self.config = load_parameters(self)
        self.planner = Planner(self.config)

        self._robot: Point | None = None
        self._target: Point | None = None
        self._target_marker: Marker | None = None
        self._opponents: tuple[RoundObstacle, ...] = ()
        self._ball: RoundObstacle | None = None
        self._avoid_ball = self.config.ball.avoid
        self._dirty = True
        self._last_result: PlanResult | None = None

        self.create_subscription(
            Marker,
            self.config.topics.robot,
            self._on_robot,
            10,
        )
        self.create_subscription(
            Marker,
            self.config.topics.target,
            self._on_target,
            10,
        )
        self.create_subscription(
            MarkerArray,
            self.config.topics.obstacles,
            self._on_obstacles,
            10,
        )
        self.create_subscription(
            Marker,
            self.config.topics.ball,
            self._on_ball,
            10,
        )
        self.create_subscription(
            Bool,
            self.config.topics.ball_obstacle_active,
            self._on_ball_obstacle_active,
            10,
        )

        self._path_publisher = self.create_publisher(
            Path,
            self.config.topics.path,
            10,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            self.config.topics.markers,
            10,
        )
        self.create_timer(1.0 / self.config.replan_hz, self._on_timer)
        self.get_logger().info(
            'Path planner ready: '
            f'{self.config.topics.robot} + '
            f'{self.config.topics.target} -> '
            f'{self.config.topics.path}'
        )

    def _on_robot(self, message: Marker) -> None:
        """Update the robot point from the existing pose Marker."""
        if message.action in (Marker.DELETE, Marker.DELETEALL):
            self._robot = None
        else:
            point = _marker_point(message)
            self._robot = None if self._invalid_position(point) else point
        self._dirty = True

    def _on_target(self, message: Marker) -> None:
        """Update the target point and preserve its final orientation."""
        if message.action in (Marker.DELETE, Marker.DELETEALL):
            self._target = None
            self._target_marker = None
        else:
            point = _marker_point(message)
            if self._invalid_position(point):
                self._target = None
                self._target_marker = None
            else:
                self._target = point
                self._target_marker = message
        self._dirty = True

    def _on_obstacles(self, message: MarkerArray) -> None:
        """Replace perceived robots with circular obstacles."""
        opponents: list[RoundObstacle] = []
        for marker in message.markers:
            if marker.action not in (Marker.ADD, Marker.MODIFY):
                continue
            point = _marker_point(marker)
            if self._invalid_position(point):
                continue
            diameter = max(float(marker.scale.x), float(marker.scale.y))
            radius = (
                diameter / 2.0
                if diameter > 0.0
                else self.config.obstacle.opponent_radius_fallback
            )
            opponents.append(RoundObstacle(point, radius))
        self._opponents = tuple(opponents)
        self._dirty = True

    def _on_ball(self, message: Marker) -> None:
        """Update the independently managed circular ball obstacle."""
        point = _marker_point(message)
        if (
            message.action in (Marker.DELETE, Marker.DELETEALL)
            or self._invalid_position(point)
        ):
            self._ball = None
        else:
            diameter = max(float(message.scale.x), float(message.scale.y))
            radius = diameter / 2.0 if diameter > 0.0 else self.config.ball.radius
            self._ball = RoundObstacle(point, radius)
        self._dirty = True

    def _on_ball_obstacle_active(self, message: Bool) -> None:
        """Enable or disable ball avoidance without deleting other robots."""
        self._avoid_ball = bool(message.data)
        self._dirty = True

    def _invalid_position(self, point: Point) -> bool:
        """Apply the configured zero-position policy to every input."""
        return (
            self.config.zero_position_is_invalid
            and point == (0.0, 0.0)
        )

    def _on_timer(self) -> None:
        """Replan on changed input and continuously publish the latest state."""
        if self._dirty:
            self._replan()
            self._dirty = False
        path = self._to_path(self._last_result)
        self._path_publisher.publish(path)
        self._publish_debug(self._last_result)

    def _replan(self) -> None:
        """Run one planning cycle when start and target are available."""
        if self._robot is None or self._target is None:
            self._last_result = None
            return
        self._last_result = self.planner.plan(
            self._robot,
            self._target,
            self._opponents,
            ball=self._ball,
            avoid_ball=self._avoid_ball,
        )
        if not self._last_result.successful:
            self.get_logger().warning('No collision-free path found')

    def _to_path(self, result: PlanResult | None) -> Path:
        """Convert a coordinate path to ``nav_msgs/Path``."""
        message = Path()
        message.header.frame_id = self.config.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        if result is None or not result.successful:
            return message

        for index, point in enumerate(result.path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            if index == len(result.path) - 1 and self._target_marker is not None:
                pose.pose.orientation = self._target_marker.pose.orientation
            else:
                next_point = result.path[min(index + 1, len(result.path) - 1)]
                yaw = math.atan2(
                    next_point[1] - point[1],
                    next_point[0] - point[0],
                )
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
        return message

    def _publish_debug(self, result: PlanResult | None) -> None:
        """Publish obstacle layers and optional visibility graph edges."""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        markers.markers.append(
            _field_boundary_marker(
                self.config.frame_id,
                self.config.field,
                now,
            )
        )
        if result is None:
            self._marker_publisher.publish(markers)
            return

        marker_id = 0
        for namespace, polygons, color in (
            ('critical', result.geometry.critical, (1.0, 0.1, 0.1, 0.9)),
            ('margin', result.geometry.margin, (1.0, 0.7, 0.0, 0.8)),
        ):
            for polygon in polygons:
                marker = Marker()
                marker.header.frame_id = self.config.frame_id
                marker.header.stamp = now
                marker.ns = namespace
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.025
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
                marker.points = [
                    _geometry_point(point) for point in polygon + polygon[:1]
                ]
                markers.markers.append(marker)

        if self.config.show_visibility_graph:
            graph = Marker()
            graph.header.frame_id = self.config.frame_id
            graph.header.stamp = now
            graph.ns = 'visibility_graph'
            graph.id = marker_id
            graph.type = Marker.LINE_LIST
            graph.action = Marker.ADD
            graph.pose.orientation.w = 1.0
            graph.scale.x = 0.008
            graph.color.r = 0.2
            graph.color.g = 0.5
            graph.color.b = 1.0
            graph.color.a = 0.35
            for start, end in result.visibility_edges:
                graph.points.extend((
                    _geometry_point(start),
                    _geometry_point(end),
                ))
            markers.markers.append(graph)
        self._marker_publisher.publish(markers)


def _marker_point(marker: Marker) -> Point:
    """Extract an XY point from a visualization Marker."""
    return (
        float(marker.pose.position.x),
        float(marker.pose.position.y),
    )


def _geometry_point(point: Point, *, z: float = 0.0) -> GeometryPoint:
    """Convert an XY tuple to a geometry message point."""
    message = GeometryPoint()
    message.x = point[0]
    message.y = point[1]
    message.z = z
    return message


def _field_boundary_marker(
    frame_id: str,
    field: FieldParameters,
    stamp,
) -> Marker:
    """Create the centered visual-only soccer field boundary marker."""
    half_length = field.length / 2.0
    half_width = field.width / 2.0
    corners = (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
        (-half_length, -half_width),
    )
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'field_boundary'
    marker.id = _FIELD_BOUNDARY_MARKER_ID
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = field.line_width
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.points = [
        _geometry_point(point, z=_FIELD_BOUNDARY_Z)
        for point in corners
    ]
    return marker


def main(args=None) -> None:
    """Run the ROS path planning node."""
    rclpy.init(args=args)
    node = PathPlanningNode()
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
