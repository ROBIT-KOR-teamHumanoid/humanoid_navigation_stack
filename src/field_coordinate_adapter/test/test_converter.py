"""Tests for field coordinate conversion."""

from field_coordinate_adapter.converter import FieldCoordinateConverter


def test_pixel_reference_points() -> None:
    """Map the requested corners and center into centered metric coordinates."""
    converter = FieldCoordinateConverter(
        width_px=1100.0,
        height_px=800.0,
        width_m=11.0,
        height_m=8.0,
        yaw_offset_deg=90.0,
    )

    assert converter.to_field(550.0, 400.0) == (0.0, 0.0)
    assert converter.to_field(0.0, 0.0) == (-5.5, 4.0)
    assert converter.to_field(1100.0, 800.0) == (5.5, -4.0)
