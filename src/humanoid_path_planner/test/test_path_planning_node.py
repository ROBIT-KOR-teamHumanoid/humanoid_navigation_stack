"""Tests for ROS path-planning debug visualization."""

from types import SimpleNamespace
from unittest.mock import Mock

from humanoid_path_planner.parameters import FieldParameters
from humanoid_path_planner.ros2_adapter.path_planning_node import (
    _field_boundary_marker,
    PathPlanningNode,
)
import pytest
from rclpy.clock import Clock
from visualization_msgs.msg import Marker, MarkerArray


def field_parameters() -> FieldParameters:
    """Return the configured nine-by-six-metre field dimensions."""
    return FieldParameters(length=9.0, width=6.0, line_width=0.05)


def test_field_boundary_is_closed_line_strip():
    """The marker must contain the exact centered closed rectangle."""
    marker = _field_boundary_marker(
        'map',
        field_parameters(),
        Clock().now().to_msg(),
    )
    coordinates = [(point.x, point.y) for point in marker.points]

    assert marker.type == Marker.LINE_STRIP
    assert len(coordinates) == 5
    assert coordinates[0] == coordinates[-1]
    assert coordinates == [
        (-4.5, -3.0),
        (4.5, -3.0),
        (4.5, 3.0),
        (-4.5, 3.0),
        (-4.5, -3.0),
    ]
    assert min(x for x, _ in coordinates) == -4.5
    assert max(x for x, _ in coordinates) == 4.5
    assert min(y for _, y in coordinates) == -3.0
    assert max(y for _, y in coordinates) == 3.0


def test_field_boundary_visual_properties():
    """The visual-only marker must use the requested RViz styling."""
    marker = _field_boundary_marker(
        'map',
        field_parameters(),
        Clock().now().to_msg(),
    )

    assert marker.header.frame_id == 'map'
    assert marker.ns == 'field_boundary'
    assert marker.id == 1000
    assert marker.pose.orientation.w == 1.0
    assert marker.scale.x == 0.05
    assert (
        marker.color.r,
        marker.color.g,
        marker.color.b,
        marker.color.a,
    ) == (1.0, 1.0, 1.0, 1.0)
    assert all(point.z == 0.01 for point in marker.points)


def test_debug_publication_keeps_boundary_without_plan():
    """Every debug publication must include the field boundary."""
    publisher = Mock()
    node = SimpleNamespace(
        config=SimpleNamespace(
            frame_id='map',
            field=field_parameters(),
        ),
        get_clock=Clock,
        _marker_publisher=publisher,
    )

    PathPlanningNode._publish_debug(node, None)

    message = publisher.publish.call_args.args[0]
    assert len(message.markers) == 2
    assert message.markers[0].action == Marker.DELETEALL
    assert message.markers[1].ns == 'field_boundary'


@pytest.mark.parametrize(
    ('zero_position_is_invalid', 'position_is_present'),
    ((False, True), (True, False)),
)
def test_zero_position_policy_applies_to_every_input(
    zero_position_is_invalid,
    position_is_present,
):
    """Robot, target, opponent, and ball must share one zero policy."""
    node = SimpleNamespace(
        config=SimpleNamespace(
            zero_position_is_invalid=zero_position_is_invalid,
            obstacle=SimpleNamespace(opponent_radius_fallback=0.075),
            ball=SimpleNamespace(radius=0.05),
        ),
        _dirty=False,
    )
    node._invalid_position = lambda point: (
        PathPlanningNode._invalid_position(node, point)
    )
    marker = Marker()
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    PathPlanningNode._on_robot(node, marker)
    PathPlanningNode._on_target(node, marker)
    PathPlanningNode._on_obstacles(node, MarkerArray(markers=[marker]))
    PathPlanningNode._on_ball(node, marker)

    assert (node._robot is not None) is position_is_present
    assert (node._target is not None) is position_is_present
    assert bool(node._opponents) is position_is_present
    assert (node._ball is not None) is position_is_present
