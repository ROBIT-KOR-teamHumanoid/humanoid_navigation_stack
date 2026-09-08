#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

# ROS setup 스크립트는 nounset(set -u)과 호환되지 않음
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

set -u

exec ros2 launch \
  humanoid_path_planner \
  path_planning_with_adapter.launch.py \
  "$@"
