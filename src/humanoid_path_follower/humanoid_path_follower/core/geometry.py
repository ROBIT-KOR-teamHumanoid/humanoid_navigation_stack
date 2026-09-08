"""Planar geometry helpers for follow-the-carrot control."""

from __future__ import annotations

import math
from typing import Sequence

from .types import Pose2D


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval ``[-pi, pi)``."""
    return (angle + math.pi) % math.tau - math.pi


def distance(first: Pose2D, second: Pose2D) -> float:
    """Return Euclidean distance between two planar poses."""
    return math.hypot(second.x - first.x, second.y - first.y)


def remaining_path_length(
    current: Pose2D,
    path: Sequence[Pose2D],
) -> float:
    """Return path length from the current position to the final pose."""
    if not path:
        return 0.0

    total = distance(current, path[0])
    for first, second in zip(path, path[1:]):
        total += distance(first, second)
    return total


def select_carrot(
    path: Sequence[Pose2D],
    carrot_index: int,
) -> Pose2D | None:
    """Select the requested waypoint or the last pose for a short path."""
    if carrot_index < 0:
        raise ValueError('carrot_index must be non-negative')
    if not path:
        return None
    return path[min(carrot_index, len(path) - 1)]


def map_vector_to_local(
    dx: float,
    dy: float,
    robot_yaw: float,
) -> tuple[float, float]:
    """Rotate a map-frame vector into the robot local frame."""
    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
    )
