#!/usr/bin/env bash
# Launch the production C++ RGB-D detector.  Override ROS launch arguments after --.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-foxy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
[[ -f "$ros_setup" ]] || { echo "ROS setup not found: $ros_setup" >&2; exit 2; }
set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u
if [[ ! -f "$project_dir/install/setup.bash" ]]; then "$project_dir/build.sh"; fi
set +u
# shellcheck disable=SC1091
source "$project_dir/install/setup.bash"
set -u
weights="${WEIGHTS:-$project_dir/weights/best.onnx}"
exec ros2 launch weldline_reflectivity_detector yolo_weldline_3d.launch.xml weights:="$weights" "$@"
