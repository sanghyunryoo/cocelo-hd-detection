#!/usr/bin/env bash
# Launch the integrated RealSense + RGB-D detector + scenario commander bringup.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_launch_pid=""
vis_pid=""
terminate_ros_launch() {
  local exit_code="${1:-130}"
  trap - INT TERM
  if [[ -n "$vis_pid" ]] && kill -0 "$vis_pid" 2>/dev/null; then
    echo "Stopping weldline debug viewer..." >&2
    kill -TERM -- "-$vis_pid" 2>/dev/null || kill -TERM "$vis_pid" 2>/dev/null || true
    sleep 0.1
    if kill -0 "$vis_pid" 2>/dev/null; then
      kill -KILL -- "-$vis_pid" 2>/dev/null || kill -KILL "$vis_pid" 2>/dev/null || true
    fi
    wait "$vis_pid" 2>/dev/null || true
  fi
  if [[ -n "$ros_launch_pid" ]] && kill -0 "$ros_launch_pid" 2>/dev/null; then
    echo "Stopping ROS launch process group immediately..." >&2
    kill -TERM -- "-$ros_launch_pid" 2>/dev/null || kill -TERM "$ros_launch_pid" 2>/dev/null || true
    sleep 0.2
    if kill -0 "$ros_launch_pid" 2>/dev/null; then
      kill -KILL -- "-$ros_launch_pid" 2>/dev/null || kill -KILL "$ros_launch_pid" 2>/dev/null || true
    fi
    wait "$ros_launch_pid" 2>/dev/null || true
  fi
  exit "$exit_code"
}
trap 'terminate_ros_launch 130' INT
trap 'terminate_ros_launch 143' TERM
if [[ -f "$script_dir/config/yolo_weldline_3d.yaml" ]]; then
  # Source-tree invocation: ./launch.sh
  project_dir="$script_dir"
  environment_helper="$script_dir/scripts/ros_environment.sh"
  paths_helper="$script_dir/scripts/project_paths.sh"
  parameter_config="${WELDLINE_CONFIG:-$script_dir/config/yolo_weldline_3d.yaml}"
  scenario_config="${WELDLINE_SCENARIO:-$script_dir/config/scenario.yaml}"
  source_tree_invocation=true
  weights_default="$project_dir/weights/best.onnx"
else
  # Installed Debian invocation: ros2 run weldline_reflectivity_detector launch.sh
  project_dir="$(cd "$script_dir/../.." && pwd)"
  environment_helper="$script_dir/ros_environment.sh"
  parameter_config="${WELDLINE_CONFIG:-$project_dir/share/weldline_reflectivity_detector/config/yolo_weldline_3d.yaml}"
  scenario_config="${WELDLINE_SCENARIO:-$project_dir/share/weldline_reflectivity_detector/config/scenario.yaml}"
  source_tree_invocation=false
  weights_default="$project_dir/share/weldline_reflectivity_detector/weights/best.onnx"
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
[[ -f "$scenario_config" ]] || { echo "Scenario config not found: $scenario_config" >&2; exit 2; }
configured_domain_id="$(read_deployment_value ros_domain_id)"
configured_usb_port_id="$(read_deployment_value usb_port_id)"
domain_id="${ROS_DOMAIN_ID:-$configured_domain_id}"
[[ "$domain_id" =~ ^[0-9]+$ ]] && (( domain_id <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232." >&2; exit 2;
}
export ROS_DOMAIN_ID="$domain_id"
usb_port_id="${USB_PORT_ID:-$configured_usb_port_id}"
start_realsense=true
start_commander=true
vis=false
ros_launch_args=()
for argument in "$@"; do
  case "$argument" in
    --vis)
      vis=true
      ;;
    --no-vis)
      vis=false
      ;;
    start_realsense:=true)
      start_realsense=true
      ;;
    start_realsense:=false)
      start_realsense=false
      ;;
    start_commander:=true)
      start_commander=true
      ;;
    start_commander:=false)
      start_commander=false
      ;;
    *)
      ros_launch_args+=("$argument")
      ;;
  esac
done
if [[ "$start_realsense" == true && -z "$usb_port_id" ]]; then
  echo "usb_port_id is required when starting RealSense. Set it in config/yolo_weldline_3d.yaml or USB_PORT_ID." >&2
  exit 2
fi
# shellcheck disable=SC1091
source "$environment_helper"
source_ros_environment
if [[ "$source_tree_invocation" == true ]]; then
  # shellcheck disable=SC1090
  source "$paths_helper"
  resolve_project_paths "$project_dir"
  installed_node="$colcon_install_base/weldline_reflectivity_detector/lib/weldline_reflectivity_detector/scenario_commander_node"
  if [[ ! -x "$installed_node" ]]; then
    "$project_dir/build.sh"
  fi
  set +u
  # shellcheck disable=SC1091
  source "$colcon_install_base/setup.bash"
  set -u
fi
weights="${WEIGHTS:-$weights_default}"
echo "Starting integrated bringup: RealSense=${start_realsense}, detector=true, commander=${start_commander}" >&2
launch_command=(
  ros2 launch weldline_reflectivity_detector yolo_weldline_3d.launch.xml
  params_file:="$parameter_config"
  scenario_file:="$scenario_config"
  weights:="$weights"
  usb_port_id:="$usb_port_id"
  start_realsense:="$start_realsense"
  start_commander:="$start_commander"
  "${ros_launch_args[@]}"
)
if command -v setsid >/dev/null 2>&1; then
  setsid "${launch_command[@]}" &
else
  "${launch_command[@]}" &
fi
ros_launch_pid="$!"
if [[ "$vis" == true ]]; then
  annotated_topic="${WELDLINE_VIS_IMAGE_TOPIC:-$(read_deployment_value annotated_topic)}"
  point_topic="${WELDLINE_VIS_POINT_TOPIC:-$(read_deployment_value debug_point_topic)}"
  annotated_topic="${annotated_topic:-/weldline_yolo/debug/annotated_image}"
  point_topic="${point_topic:-/weldline_yolo/debug/center_point}"
  if [[ "$source_tree_invocation" == true ]]; then
    viewer_script="$project_dir/scripts/annotated_image_viewer.py"
  else
    viewer_script="$project_dir/lib/weldline_reflectivity_detector/annotated_image_viewer.py"
  fi
  vis_command=(python3 "$viewer_script" --image-topic "$annotated_topic" --point-topic "$point_topic" --max-fps "${WELDLINE_VIS_FPS:-30}")
  if command -v setsid >/dev/null 2>&1; then
    setsid "${vis_command[@]}" &
  else
    "${vis_command[@]}" &
  fi
  vis_pid="$!"
fi
set +e
wait "$ros_launch_pid"
launch_status="$?"
set -e
ros_launch_pid=""
if [[ -n "$vis_pid" ]] && kill -0 "$vis_pid" 2>/dev/null; then
  kill -TERM -- "-$vis_pid" 2>/dev/null || kill -TERM "$vis_pid" 2>/dev/null || true
  wait "$vis_pid" 2>/dev/null || true
fi
exit "$launch_status"
