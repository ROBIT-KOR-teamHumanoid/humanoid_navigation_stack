"""Tests for the ROS-independent follow-the-carrot controller."""

import math

from humanoid_path_follower.core.controller import (
    FollowTheCarrotController,
)
from humanoid_path_follower.core.types import (
    ControllerResult,
    Pose2D,
    VelocityCommand,
)
from humanoid_path_follower.parameters import ControllerParameters
import pytest


def controller_parameters() -> ControllerParameters:
    """Return conservative controller parameters used by the package."""
    return ControllerParameters(
        carrot_index=1,
        min_vel_x=-0.20,
        max_vel_x=0.30,
        min_vel_y=-0.10,
        max_vel_y=0.15,
        min_vel_theta=-0.60,
        max_vel_theta=0.80,
        translation_gain=1.5,
        rotation_kp=2.0,
        orient_to_goal_distance=0.3,
        position_tolerance=0.05,
        orientation_tolerance=0.087,
        smoothing_tau=0.04,
    )


def command_for(goal: Pose2D, robot: Pose2D | None = None):
    """Return the first unsmoothed command for a one-pose path."""
    controller = FollowTheCarrotController(controller_parameters())
    return controller.step(robot or Pose2D(0.0, 0.0, 0.0), [goal])


def test_front_path_commands_forward_motion():
    """A carrot straight ahead must produce positive local X only."""
    result = command_for(Pose2D(1.0, 0.0, 0.0))

    assert result.command.vx > 0.0
    assert result.command.vy == pytest.approx(0.0)
    assert result.command.vx <= controller_parameters().max_vel_x


def test_side_path_commands_bounded_lateral_motion():
    """A lateral carrot must produce bounded local Y velocity."""
    result = command_for(Pose2D(0.0, 1.0, 0.0))

    assert 0.0 < result.command.vy <= controller_parameters().max_vel_y
    assert result.command.vx == pytest.approx(0.0, abs=1.0e-12)


def test_opposite_side_path_uses_min_lateral_velocity():
    """Negative local Y motion must use its independent lower bound."""
    parameters = controller_parameters()
    result = command_for(Pose2D(0.0, -1.0, 0.0))

    assert result.command.vy == parameters.min_vel_y
    assert result.command.vx == pytest.approx(0.0, abs=1.0e-12)


def test_rear_path_commands_backward_motion():
    """A carrot behind the robot must produce negative local X."""
    result = command_for(Pose2D(-1.0, 0.0, 0.0))

    assert controller_parameters().min_vel_x <= result.command.vx < 0.0
    assert result.command.vy == pytest.approx(0.0)


def test_diagonal_path_commands_both_translation_axes():
    """A diagonal carrot must produce simultaneous X and Y commands."""
    result = command_for(Pose2D(1.0, 1.0, 0.0))

    assert result.command.vx > 0.0
    assert result.command.vy > 0.0
    assert result.command.vx <= controller_parameters().max_vel_x
    assert result.command.vy <= controller_parameters().max_vel_y


def test_rotation_sign_and_limit_follow_angle_error():
    """Rotation P control must keep error sign and respect its limit."""
    positive = command_for(Pose2D(0.0, 1.0, 0.0))
    negative = command_for(Pose2D(0.0, -1.0, 0.0))

    assert positive.command.wz == controller_parameters().max_vel_theta
    assert negative.command.wz == controller_parameters().min_vel_theta


def test_near_goal_aligns_to_final_yaw():
    """Near the final position rotation must use the path goal yaw."""
    result = command_for(Pose2D(0.1, 0.0, math.pi / 2.0))

    assert result.command.wz == controller_parameters().max_vel_theta


def test_position_tolerance_stops_translation_during_alignment():
    """Position completion must stop translation while yaw converges."""
    result = command_for(Pose2D(0.01, 0.0, math.pi / 2.0))

    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.wz > 0.0
    assert result.goal_reached is False


def test_goal_reached_returns_exact_zero_velocity():
    """Both goal tolerances must produce a completed zero command."""
    result = command_for(
        Pose2D(0.0, 0.0, 0.05),
        robot=Pose2D(0.0, 0.0, 0.0),
    )

    assert result.command == VelocityCommand()
    assert result.goal_reached is True


def test_empty_path_returns_consistent_zero_result():
    """An empty path must never raise or change the result type."""
    controller = FollowTheCarrotController(controller_parameters())

    result = controller.step(Pose2D(0.0, 0.0, 0.0), [])

    assert result == ControllerResult(VelocityCommand(), None, False)


def test_smoothing_uses_elapsed_time_and_previous_command():
    """A direction reversal must be exponentially blended over time."""
    parameters = controller_parameters()
    controller = FollowTheCarrotController(parameters)
    path = [Pose2D(1.0, 0.0, 0.0)]
    first = controller.step(Pose2D(0.0, 0.0, 0.0), path)
    second = controller.step(
        Pose2D(0.0, 0.0, math.pi),
        path,
        dt=parameters.smoothing_tau,
    )
    alpha = 1.0 - math.exp(-1.0)
    expected_vx = (
        alpha * parameters.min_vel_x
        + (1.0 - alpha) * first.command.vx
    )

    assert first.command.vx == parameters.max_vel_x
    assert second.command.vx == pytest.approx(expected_vx)
    assert parameters.min_vel_x < second.command.vx < first.command.vx
