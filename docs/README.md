# Humanoid Navigation Stack

ROS 2 Jazzy packages for field-coordinate conversion, collision-aware path
planning, and holonomic path following.

## Packages

| Package | Purpose |
|---|---|
| field_coordinate_adapter | Converts pixel positions and yaw angles into the map frame |
| humanoid_path_planner | Generates a collision-aware nav_msgs/Path |
| humanoid_path_follower | Converts a map-frame path into holonomic velocity commands |

## Build and Run

From the repository root:

~~~bash
make build-navigation
make start-navigation
~~~

Start the complete stack with RViz:

~~~bash
make debug-navigation
~~~

The complete navigation entry point starts the field coordinate adapter, path
planner, and path follower. The debug entry point additionally starts RViz and
shuts down the complete launch when RViz exits.

The path-planning subset remains available independently:

~~~bash
make build-path-planning
make start-path-planning
make debug-path-planning
~~~

Direct launch:

~~~bash
source ../../install/setup.bash
ros2 launch humanoid_path_follower navigation.launch.py
ros2 launch humanoid_path_follower navigation_debug.launch.py
~~~

Run only the follower when its input topics are already available:

~~~bash
ros2 launch humanoid_path_follower path_follower.launch.py
~~~

## Test

~~~bash
make test-navigation
~~~

## Configuration

- [Adapter configuration](../src/field_coordinate_adapter/config/field_coordinate_adapter.yaml)
- [Planner configuration](../src/humanoid_path_planner/config/path_planning.yaml)
- [Follower configuration](../src/humanoid_path_follower/config/path_follower.yaml)

## Documentation

- [Architecture](architecture.md)
- [Coordinate system](coordinate_system.md)
