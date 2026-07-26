#!/usr/bin/env bash
# Build the Python ROS 2 package using the system Python required by ROS tooling.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$project_dir/scripts/ros_environment.sh"
# shellcheck disable=SC1091
source "$project_dir/scripts/project_paths.sh"
source_ros_environment
resolve_project_paths "$project_dir"

mkdir -p "$colcon_build_base" "$colcon_install_base" "$colcon_log_base"
printf 'Building weldline_reflectivity_detector\n  source:  %s\n  build:   %s\n  install: %s\n  log:     %s\n' \
  "$project_dir" "$colcon_build_base" "$colcon_install_base" "$colcon_log_base"

colcon --log-base "$colcon_log_base" build \
  --base-paths "$project_dir" \
  --build-base "$colcon_build_base" \
  --install-base "$colcon_install_base" \
  --packages-select weldline_reflectivity_detector
