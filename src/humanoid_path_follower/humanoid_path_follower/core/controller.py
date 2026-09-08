"""ROS-independent holonomic follow-the-carrot controller."""

from __future__ import annotations

import math
from typing import Sequence

from .geometry import (
    distance,
    map_vector_to_local,
    normalize_angle,
    remaining_path_length,
    select_carrot,
)
from .types import ControllerResult, Pose2D, VelocityCommand
from ..parameters import ControllerParameters

_EPSILON = 1.0e-12
_ZERO_COMMAND = VelocityCommand()


class FollowTheCarrotController:
    """Generate bounded holonomic velocity commands toward a path carrot."""

    def __init__(self, parameters: ControllerParameters) -> None:
        """Store controller parameters and initialize smoothing state."""
        self.parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Forget the previous command before following a new path."""
        self._last_command = _ZERO_COMMAND
        self._has_previous_command = False

    def step(
        self,
        robot: Pose2D,
        path: Sequence[Pose2D],
        *,
        dt: float | None = None,
    ) -> ControllerResult:
        """Calculate one follow-the-carrot control step."""
        carrot = select_carrot(path, self.parameters.carrot_index)
        if carrot is None:
            self.reset()
            return ControllerResult(_ZERO_COMMAND, None, False)

        goal = path[-1]
        position_error = distance(robot, goal)
        goal_yaw_error = normalize_angle(goal.yaw - robot.yaw)
        if (
            position_error <= self.parameters.position_tolerance
            and abs(goal_yaw_error)
            <= self.parameters.orientation_tolerance
        ):
            self._last_command = _ZERO_COMMAND
            self._has_previous_command = True
            return ControllerResult(_ZERO_COMMAND, carrot, True)

        dx = carrot.x - robot.x
        dy = carrot.y - robot.y
        local_x, local_y = map_vector_to_local(dx, dy, robot.yaw)
        local_distance = math.hypot(local_x, local_y)
        path_length = remaining_path_length(robot, path)

        vx = 0.0
        vy = 0.0
        if (
            position_error > self.parameters.position_tolerance
            and local_distance > _EPSILON
        ):
            direction_x = local_x / local_distance
            direction_y = local_y / local_distance
            speed = min(
                path_length * self.parameters.translation_gain,
                self._directional_speed_limit(
                    direction_x,
                    direction_y,
                ),
            )
            vx = direction_x * speed
            vy = direction_y * speed

        if path_length > self.parameters.orient_to_goal_distance:
            target_yaw = math.atan2(dy, dx)
            angle_error = normalize_angle(target_yaw - robot.yaw)
        else:
            angle_error = goal_yaw_error
        wz = _clamp(
            self.parameters.rotation_kp * angle_error,
            self.parameters.min_vel_theta,
            self.parameters.max_vel_theta,
        )

        command = self._smooth(VelocityCommand(vx, vy, wz), dt)
        if position_error <= self.parameters.position_tolerance:
            command = VelocityCommand(0.0, 0.0, command.wz)
        self._last_command = command
        self._has_previous_command = True
        return ControllerResult(command, carrot, False)

    def _directional_speed_limit(
        self,
        direction_x: float,
        direction_y: float,
    ) -> float:
        """Return a heading-dependent limit while preserving direction."""
        x_limit = (
            self.parameters.max_vel_x
            if direction_x >= 0.0
            else -self.parameters.min_vel_x
        )
        y_limit = (
            self.parameters.max_vel_y
            if direction_y >= 0.0
            else -self.parameters.min_vel_y
        )
        denominator = (
            x_limit * abs(direction_y)
            + y_limit * abs(direction_x)
        )
        if denominator <= _EPSILON:
            return 0.0
        return x_limit * y_limit / denominator

    def _smooth(
        self,
        target: VelocityCommand,
        dt: float | None,
    ) -> VelocityCommand:
        """Apply time-step-independent exponential command smoothing."""
        if (
            not self._has_previous_command
            or dt is None
            or dt < 0.0
            or self.parameters.smoothing_tau <= 0.0
        ):
            return target

        alpha = 1.0 - math.exp(-dt / self.parameters.smoothing_tau)
        previous = self._last_command
        return VelocityCommand(
            vx=alpha * target.vx + (1.0 - alpha) * previous.vx,
            vy=alpha * target.vy + (1.0 - alpha) * previous.vy,
            wz=alpha * target.wz + (1.0 - alpha) * previous.wz,
        )


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a scalar to an inclusive range."""
    return min(max(value, minimum), maximum)
