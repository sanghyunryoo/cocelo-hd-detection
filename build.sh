#!/usr/bin/env bash
# Build the native ROS 2 C++ package using the system Python required by ROS tooling.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-foxy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
[[ -f "$ros_setup" ]] || { echo "ROS setup not found: $ros_setup" >&2; exit 2; }
# ROS Foxy setup references optional variables that are unset under `set -u`.
set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u
cd "$project_dir"
colcon build --packages-select weldline_reflectivity_detector --cmake-args \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" -DPYTHON_EXECUTABLE=/usr/bin/python3
