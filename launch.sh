#!/usr/bin/env bash
# Launch the production C++ RGB-D detector.  Override ROS launch arguments after --.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/config/yolo_weldline_3d.yaml" ]]; then
  # Source-tree invocation: ./launch.sh
  project_dir="$script_dir"
  environment_helper="$script_dir/scripts/ros_environment.sh"
  parameter_config="$script_dir/config/yolo_weldline_3d.yaml"
else
  # Installed Debian invocation: ros2 run weldline_reflectivity_detector launch.sh
  project_dir="$(cd "$script_dir/../.." && pwd)"
  environment_helper="$script_dir/ros_environment.sh"
  parameter_config="$project_dir/share/weldline_reflectivity_detector/config/yolo_weldline_3d.yaml"
fi
read_deployment_value() {
  local key="$1"
  awk -F: -v key="$key" '
    $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      value=$2; sub(/^[[:space:]]*/, "", value); sub(/[[:space:]]*(#.*)?$/, "", value)
      gsub(/^"|"$/, "", value); print value; exit
    }' "$parameter_config"
}
[[ -f "$parameter_config" ]] || { echo "Parameter config not found: $parameter_config" >&2; exit 2; }
configured_domain_id="$(read_deployment_value ros_domain_id)"
configured_usb_port_id="$(read_deployment_value usb_port_id)"
domain_id="${ROS_DOMAIN_ID:-$configured_domain_id}"
[[ "$domain_id" =~ ^[0-9]+$ ]] && (( domain_id <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232." >&2; exit 2;
}
export ROS_DOMAIN_ID="$domain_id"
usb_port_id="${USB_PORT_ID:-$configured_usb_port_id}"
start_realsense=true
for argument in "$@"; do
  [[ "$argument" == "start_realsense:=false" ]] && start_realsense=false
done
if [[ "$start_realsense" == true && -z "$usb_port_id" ]]; then
  echo "usb_port_id is required when starting RealSense. Set it in config/yolo_weldline_3d.yaml or USB_PORT_ID." >&2
  exit 2
fi
# shellcheck disable=SC1091
source "$environment_helper"
source_ros_environment
if [[ ! -f "$project_dir/install/setup.bash" ]]; then "$project_dir/build.sh"; fi
set +u
# shellcheck disable=SC1091
source "$project_dir/install/setup.bash"
set -u
weights="${WEIGHTS:-$project_dir/weights/best.onnx}"
exec ros2 launch weldline_reflectivity_detector yolo_weldline_3d.launch.xml \
  weights:="$weights" usb_port_id:="$usb_port_id" "$@"
