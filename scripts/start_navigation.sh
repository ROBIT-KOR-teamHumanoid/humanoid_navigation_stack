#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

# ROS setup 스크립트는 nounset(set -u)과 호환되지 않음
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec ros2 launch \
  humanoid_path_follower \
  navigation.launch.py \
  "$@"
