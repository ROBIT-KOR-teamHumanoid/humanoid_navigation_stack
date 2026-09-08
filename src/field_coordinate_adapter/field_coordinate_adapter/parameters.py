"""Parameter loading for the field coordinate adapter."""

from dataclasses import dataclass


@dataclass(frozen=True)
class AdapterParameters:
    """Field coordinate adapter parameters."""

    hz: float
    output_frame_id: str
    zero_pixel_is_invalid: bool
    field_width_px: float
    field_height_px: float
    field_width_m: float
    field_height_m: float
    yaw_offset_deg: float
    topic_localization: str
    topic_imu: str
    topic_input_target: str
    topic_robot: str
    topic_obstacles: str
    topic_ball: str
    topic_output_target: str


def load_parameters(node) -> AdapterParameters:
    """Declare, read, validate, and return adapter parameters."""
    node.declare_parameter('hz', 30.0)
    node.declare_parameter('output_frame_id', 'map')
    node.declare_parameter('zero_pixel_is_invalid', False)

    node.declare_parameter('field.width_px', 1100.0)
    node.declare_parameter('field.height_px', 800.0)
    node.declare_parameter('field.width_m', 11.0)
    node.declare_parameter('field.height_m', 8.0)
    node.declare_parameter('yaw_offset_deg', 90.0)

    node.declare_parameter('topics.input.localization', '/localization')
    node.declare_parameter('topics.input.imu', '/Imu')
    node.declare_parameter('topics.input.target', '/master2local')

    node.declare_parameter('topics.output.robot', '/adapter/pose_marker')
    node.declare_parameter(
        'topics.output.obstacles',
        '/adapter/obstacle_marker',
    )
    node.declare_parameter('topics.output.ball', '/adapter/ball_marker')
    node.declare_parameter('topics.output.target', '/adapter/target_marker')

    parameters = AdapterParameters(
        hz=float(node.get_parameter('hz').value),
        output_frame_id=str(node.get_parameter('output_frame_id').value),
        zero_pixel_is_invalid=bool(
            node.get_parameter('zero_pixel_is_invalid').value
        ),
        field_width_px=float(node.get_parameter('field.width_px').value),
        field_height_px=float(node.get_parameter('field.height_px').value),
        field_width_m=float(node.get_parameter('field.width_m').value),
        field_height_m=float(node.get_parameter('field.height_m').value),
        yaw_offset_deg=float(node.get_parameter('yaw_offset_deg').value),
        topic_localization=str(
            node.get_parameter('topics.input.localization').value
        ),
        topic_imu=str(node.get_parameter('topics.input.imu').value),
        topic_input_target=str(node.get_parameter('topics.input.target').value),
        topic_robot=str(node.get_parameter('topics.output.robot').value),
        topic_obstacles=str(
            node.get_parameter('topics.output.obstacles').value
        ),
        topic_ball=str(node.get_parameter('topics.output.ball').value),
        topic_output_target=str(
            node.get_parameter('topics.output.target').value
        )
    )

    positive_values = {
        'hz': parameters.hz,
        'field.width_px': parameters.field_width_px,
        'field.height_px': parameters.field_height_px,
        'field.width_m': parameters.field_width_m,
        'field.height_m': parameters.field_height_m,
    }
    for name, value in positive_values.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')

    if not parameters.output_frame_id:
        raise ValueError('output_frame_id must not be empty')

    return parameters
