"""Tests for source-yaw to ROS-yaw conversion."""

from field_coordinate_adapter.converter import FieldCoordinateConverter
import pytest


@pytest.mark.parametrize(
    ('source_yaw_deg', 'expected_direction'),
    [
        (0.0, (0.0, 1.0)),
        (90.0, (-1.0, 0.0)),
        (-90.0, (1.0, 0.0)),
        (180.0, (0.0, -1.0)),
        (-180.0, (0.0, -1.0)),
    ],
)
def test_yaw_cardinal_directions(
    source_yaw_deg: float,
    expected_direction: tuple[float, float],
) -> None:
    """Map 0/up, 90/left, -90/right, and 180/down."""
    converter = FieldCoordinateConverter(
        width_px=1100.0,
        height_px=800.0,
        width_m=11.0,
        height_m=8.0,
        yaw_offset_deg=90.0,
    )

    _, _, z, w = converter.yaw_to_quaternion(source_yaw_deg)
    direction_x = w * w - z * z
    direction_y = 2.0 * w * z

    assert direction_x == pytest.approx(expected_direction[0], abs=1e-12)
    assert direction_y == pytest.approx(expected_direction[1], abs=1e-12)
