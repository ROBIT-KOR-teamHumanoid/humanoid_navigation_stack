# Architecture

## Data Flow

~~~mermaid
flowchart LR
    Localization["/localization"] --> Adapter["field_coordinate_adapter"]
    Imu["/Imu"] --> Adapter
    Target["/master2local"] --> Adapter

    Adapter -->|robot, target, obstacles, ball| Planner["path_planning"]
    BallControl["/ball_obstacle_active"] --> Planner

    Adapter -->|/adapter/pose_marker| Follower["path_follower"]
    Planner -->|/vg/path| Follower
    Follower -->|/cmd_vel| Motion["robot motion"]
    Follower -->|/debug/carrot| RViz
    Planner -->|/vg/markers| RViz
~~~

## Components

### field_coordinate_adapter

Stores the latest localization, IMU, and target messages, then publishes converted Markers at 30 Hz.

### path_planning

Consumes the converted Markers and generates a visibility-graph path at up to 10 Hz.

### path_follower

Consumes the robot pose Marker and planned Path in the same map frame. A TF-free follow-the-carrot controller publishes local holonomic velocity commands and stops on missing, stale, or frame-incompatible input.

The ROS-independent controller, geometry helpers, and data types live under
`humanoid_path_follower/core`. ROS message conversion, subscriptions, and
publishers live under `humanoid_path_follower/ros2_adapter`.

## Main Topics

| Topic | Type | Description |
|---|---|---|
| /localization | RobocupLocalization | Robot, ball, and obstacle pixel positions |
| /Imu | ImuMsg | Robot yaw |
| /master2local | Master2localization | Target position and yaw |
| /adapter/pose_marker | Marker | Robot pose in the map frame |
| /adapter/target_marker | Marker | Target pose in the map frame |
| /adapter/obstacle_marker | MarkerArray | Opponent obstacles |
| /adapter/ball_marker | Marker | Ball position |
| /vg/path | Path | Planned path |
| /vg/markers | MarkerArray | Planner debug visualization |
| /cmd_vel | Twist | Robot-local holonomic velocity command |
| /debug/carrot | PointStamped | Selected path-following carrot |

## Obstacles

Opponent robots passed to the planner core are inflated by the robot radius and safety margin. The ball is included only while avoidance is active, initially configured by `ball.avoid` and updated at runtime through `/ball_obstacle_active`. Each enabled goal is modeled as a static U-shaped obstacle.

For dynamic obstacles closer to the robot start point than
`obstacle.near_distance`, the margin layer receives an additional
`obstacle.near_extra_margin`. This applies uniformly to every dynamic obstacle
passed to the core, including the ball while ball avoidance is active.
Overlapping inflated circles are merged into a conservative convex hull.

The planner maintains two obstacle layers:

- `critical`: the physical obstacle inflated by the robot radius
- `margin`: the critical area plus the configured safety margin

Normal visibility-graph edges that cross a margin are blocked. If the start or goal is already inside an obstacle, edges that let the path leave or enter that obstacle remain available. An allowed edge crossing a critical area uses `search.critical_cost_multiplier` as its cost multiplier. The direct start-to-goal edge is retained as a best-effort fallback candidate, allowing the planner to return a path when no collision-free detour exists.

~~~text
goal_x = ±(field.length / 2 + goal_obstacle.goal_line_offset)
~~~

## Launch

[path_planning_with_adapter.launch.py](../src/humanoid_path_planner/launch/path_planning_with_adapter.launch.py) starts the adapter and planner together.

[path_planning_debug.launch.py](../src/humanoid_path_planner/launch/path_planning_debug.launch.py) additionally starts RViz with the installed planner configuration.

[path_follower.launch.py](../src/humanoid_path_follower/launch/path_follower.launch.py) starts the follower with its installed YAML configuration.

[navigation.launch.py](../src/humanoid_path_follower/launch/navigation.launch.py) includes path planning with the coordinate adapter and the path follower.

[navigation_debug.launch.py](../src/humanoid_path_follower/launch/navigation_debug.launch.py) additionally starts RViz with the follower configuration, including the planned path, planner obstacle layers, and selected carrot.

The repository scripts and Makefile expose these launch files as
`make start-navigation` and `make debug-navigation`.
