# Coordinate System

## Pixel to Map

Source localization coordinates use a 1100 × 800 px image with the origin at the top-left corner.

The adapter converts them into a centered, meter-based map frame:

~~~text
x_m = (x_px - width_px / 2) × width_m / width_px
y_m = (height_px / 2 - y_px) × height_m / height_px
~~~

| Pixel | Map | Location |
|---|---|---|
| (0, 0) | (-5.5, 4.0) | Top-left |
| (550, 400) | (0.0, 0.0) | Center |
| (1100, 800) | (5.5, -4.0) | Bottom-right |

Axis directions in the map frame:

- Positive X: right
- Positive Y: up

## Yaw

~~~text
ROS yaw = source yaw + 90°
~~~

| Source yaw | Direction | ROS yaw |
|---:|---|---:|
| 0° | Up | 90° |
| 90° | Left | 180° |
| -90° | Right | 0° |
| ±180° | Down | -90° (or 270°) |

Robot yaw comes from ImuMsg.yaw. Target yaw comes from Master2localization.angle_to_target.

## Follower Velocity Frame

The planner Path and robot pose Marker must have the same non-empty frame ID,
normally `map`. The follower does not perform TF lookup or frame conversion.

The follower rotates the map-frame carrot vector into the robot-local frame
before publishing `geometry_msgs/Twist`:

- Positive `linear.x`: forward
- Negative `linear.x`: backward
- Positive `linear.y`: left
- Negative `linear.y`: right
- Positive `angular.z`: counter-clockwise rotation
- Negative `angular.z`: clockwise rotation

## Important Settings

- Set adapter `zero_pixel_is_invalid` to false when source pixel `(0, 0)` is a
  valid position. It converts to map `(-5.5, 4.0)` with the default field size.
- Keep `zero_position_is_invalid` set to false because the field center is
  `(0, 0)` and the policy applies to every planner input position.
- Adapter field dimensions define the converted coordinate range.
- Planner `field.length` is the single field-length value used for the RViz boundary and goal-line positions.
- Goal obstacles are centered at `±(field.length / 2 + goal_obstacle.goal_line_offset)` on the X axis.

See the [adapter configuration](../src/field_coordinate_adapter/config/field_coordinate_adapter.yaml), [planner configuration](../src/humanoid_path_planner/config/path_planning.yaml), and [follower configuration](../src/humanoid_path_follower/config/path_follower.yaml).
