"""Tests for field coordinate adapter parameter loading."""

from pathlib import Path

from field_coordinate_adapter.parameters import load_parameters
import rclpy
from rclpy.context import Context
from rclpy.node import Node


def test_yaml_allows_zero_pixel_positions() -> None:
    """The installed policy must treat source pixel zero as valid."""
    config = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'field_coordinate_adapter.yaml'
    )
    context = Context()
    rclpy.init(
        args=['--ros-args', '--params-file', str(config)],
        context=context,
    )
    node = Node('field_coordinate_adapter', context=context)
    try:
        parameters = load_parameters(node)
        assert parameters.zero_pixel_is_invalid is False
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
