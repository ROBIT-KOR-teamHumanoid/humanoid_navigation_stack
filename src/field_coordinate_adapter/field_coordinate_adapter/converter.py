"""Coordinate conversion utilities for the field adapter."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FieldCoordinateConverter:
    """Convert top-left-origin pixel coordinates to centered field meters."""

    width_px: float
    height_px: float
    width_m: float
    height_m: float
    yaw_offset_deg: float

    def to_field(self, pixel_x: float, pixel_y: float) -> tuple[float, float]:
        """Convert a pixel position into a centered, Y-up metric position."""
        x = (pixel_x - self.width_px / 2.0) * self.width_m / self.width_px
        y = (self.height_px / 2.0 - pixel_y) * self.height_m / self.height_px
        return x, y

    def yaw_to_quaternion(
        self,
        yaw_deg: float,
    ) -> tuple[float, float, float, float]:
        """Convert the source yaw angle into a planar ROS quaternion."""
        yaw = math.radians(yaw_deg + self.yaw_offset_deg)
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )
