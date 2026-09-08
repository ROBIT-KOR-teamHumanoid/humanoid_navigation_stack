"""Immutable data types shared by the follower core."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """A planar position and heading in radians."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    """A holonomic planar velocity command in the robot frame."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class ControllerResult:
    """The command, selected carrot, and goal completion state."""

    command: VelocityCommand
    carrot: Pose2D | None
    goal_reached: bool
