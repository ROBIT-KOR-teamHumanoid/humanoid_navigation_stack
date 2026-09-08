"""Tests for required ROS path-planner parameter loading."""

from pathlib import Path

from humanoid_path_planner.parameters import load_parameters
import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node


def test_load_parameters_uses_yaml_as_only_source():
    """The package YAML must provide every effective parameter value."""
    config = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'path_planning.yaml'
    )
    context = Context()
    rclpy.init(
        args=['--ros-args', '--params-file', str(config)],
        context=context,
    )
    node = Node('path_planning', context=context)
    try:
        parameters = load_parameters(node)
        assert parameters.frame_id == 'map'
        assert parameters.zero_position_is_invalid is False
        assert parameters.field.length == 9.0
        assert parameters.field.width == 6.0
        assert parameters.field.line_width == 0.05
        assert parameters.goal_obstacle.field_length == 9.0
        assert parameters.goal_obstacle.goal_width == 1.3
        assert parameters.goal_obstacle.goal_line_offset == 0.30
        assert parameters.goal_obstacle.back_extension == 0.0
        assert parameters.topics.robot == '/adapter/pose_marker'
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_load_parameters_rejects_missing_yaml():
    """Starting without the required parameter file must fail clearly."""
    context = Context()
    rclpy.init(args=[], context=context)
    node = Node(
        'path_planning',
        context=context,
        use_global_arguments=False,
    )
    try:
        with pytest.raises(
            ValueError,
            match="required ROS parameter 'field.length' is missing",
        ):
            load_parameters(node)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
