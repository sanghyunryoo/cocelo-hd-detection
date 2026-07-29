#!/usr/bin/env bash
# Launch RealSense and the weldline RGB-D goal publisher.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_launch_pid=""
ros_launch_uses_group=false
cleanup_started=false

process_alive() {
  local pid="$1"
  local uses_group="$2"
  if [[ "$uses_group" == true ]]; then
    kill -0 -- "-$pid" 2>/dev/null
  else
    kill -0 "$pid" 2>/dev/null
  fi
}

signal_process() {
  local signal="$1"
  local pid="$2"
  local uses_group="$3"
  if [[ "$uses_group" == true ]]; then
    kill "-$signal" -- "-$pid" 2>/dev/null || true
  else
    kill "-$signal" "$pid" 2>/dev/null || true
  fi
}

wait_for_process_exit() {
  local pid="$1"
  local uses_group="$2"
  local timeout_sec="$3"
  local waited=0

  while process_alive "$pid" "$uses_group"; do
    if (( waited >= timeout_sec * 10 )); then
      return 1
    fi
    sleep 0.1
    waited=$((waited + 1))
  done
  return 0
}

cleanup() {
  local exit_code="${1:-$?}"
  if [[ "$cleanup_started" == true ]]; then
    exit "$exit_code"
  fi
  cleanup_started=true
  trap - EXIT INT TERM
  if [[ -n "$ros_launch_pid" ]] && process_alive "$ros_launch_pid" "$ros_launch_uses_group"; then
    echo "Stopping all ROS launch nodes..." >&2
    signal_process INT "$ros_launch_pid" "$ros_launch_uses_group"
    wait_for_process_exit "$ros_launch_pid" "$ros_launch_uses_group" 3 || true
  fi
  if [[ -n "$ros_launch_pid" ]] && process_alive "$ros_launch_pid" "$ros_launch_uses_group"; then
    signal_process TERM "$ros_launch_pid" "$ros_launch_uses_group"
    wait_for_process_exit "$ros_launch_pid" "$ros_launch_uses_group" 5 || true
  fi
  if [[ -n "$ros_launch_pid" ]] && process_alive "$ros_launch_pid" "$ros_launch_uses_group"; then
    echo "ROS launch did not stop cleanly; forcing shutdown." >&2
    signal_process KILL "$ros_launch_pid" "$ros_launch_uses_group"
  fi
  [[ -z "$ros_launch_pid" ]] || wait "$ros_launch_pid" 2>/dev/null || true
  exit "$exit_code"
}
trap cleanup EXIT
trap 'cleanup 130' INT
trap 'cleanup 143' TERM
if [[ -f "$script_dir/config/yolo_weldline_3d.yaml" ]]; then
  # Source-tree invocation: ./launch.sh
  project_dir="$script_dir"
  paths_helper="$script_dir/scripts/project_paths.sh"
  parameter_config="${WELDLINE_CONFIG:-$script_dir/config/yolo_weldline_3d.yaml}"
  source_tree_invocation=true
  weights_default="$project_dir/weights/best.onnx"
  # Source-tree launches discover ROS automatically and locate the external
  # colcon install tree. The installed launcher is already prepared by
  # /usr/bin/weldline-detector and does not need these development helpers.
  # shellcheck disable=SC1091
  source "$script_dir/scripts/ros_environment.sh"
  source_ros_environment
else
  # Installed Debian invocation via /usr/bin/weldline-detector.
  project_dir="$(cd "$script_dir/../.." && pwd)"
  parameter_config="${WELDLINE_CONFIG:-$project_dir/share/weldline_goal_publisher/config/yolo_weldline_3d.yaml}"
  source_tree_invocation=false
  weights_default="$project_dir/share/weldline_goal_publisher/weights/best.onnx"
fi
read_deployment_value() {
  local key="$1"
  awk -F: -v key="$key" '
    $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      value=$2; sub(/^[[:space:]]*/, "", value); sub(/[[:space:]]*(#.*)?$/, "", value)
      gsub(/^"|"$/, "", value); print value; exit
    }' "$parameter_config"
}

list_realsense_usb_port_ids() {
  local device_dir
  local vendor
  local product
  local manufacturer
  local name

  shopt -s nullglob
  for device_dir in /sys/bus/usb/devices/*; do
    [[ -r "$device_dir/idVendor" ]] || continue
    vendor="$(<"$device_dir/idVendor")"
    [[ "${vendor,,}" == "8086" ]] || continue

    product=""
    manufacturer=""
    [[ -r "$device_dir/product" ]] && product="$(<"$device_dir/product")"
    [[ -r "$device_dir/manufacturer" ]] && manufacturer="$(<"$device_dir/manufacturer")"
    name="${product} ${manufacturer}"
    [[ "${name,,}" == *realsense* ]] || continue

    basename "$device_dir"
  done
  shopt -u nullglob
}

detect_realsense_usb_port_id() {
  local matches=()
  mapfile -t matches < <(list_realsense_usb_port_ids)

  if [[ "${#matches[@]}" -eq 1 ]]; then
    printf '%s\n' "${matches[0]}"
    return 0
  fi

  if [[ "${#matches[@]}" -eq 0 ]]; then
    echo "No RealSense USB device was found for automatic usb_port_id selection." >&2
  else
    echo "Multiple RealSense USB devices were found: ${matches[*]}" >&2
    echo "Set USB_PORT_ID or config usb_port_id to choose one explicitly." >&2
  fi
  return 1
}

realsense_usb_port_exists() {
  local requested="$1"
  local port

  while IFS= read -r port; do
    [[ "$port" == "$requested" ]] && return 0
  done < <(list_realsense_usb_port_ids)
  return 1
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
ros_launch_args=()
for argument in "$@"; do
  case "$argument" in
    start_realsense:=true)
      start_realsense=true
      ;;
    start_realsense:=false)
      start_realsense=false
      ;;
    *)
      ros_launch_args+=("$argument")
      ;;
  esac
done
if [[ "$start_realsense" == true ]]; then
  if [[ -z "$usb_port_id" || "$usb_port_id" == "auto" ]]; then
    usb_port_id="$(detect_realsense_usb_port_id)" || exit 2
    echo "Auto-selected RealSense usb_port_id=${usb_port_id}" >&2
  elif ! realsense_usb_port_exists "$usb_port_id"; then
    if [[ -z "${USB_PORT_ID:-}" ]]; then
      echo "Configured RealSense usb_port_id=${usb_port_id} is not connected; trying automatic selection." >&2
      usb_port_id="$(detect_realsense_usb_port_id)" || exit 2
      echo "Auto-selected RealSense usb_port_id=${usb_port_id}" >&2
    else
      echo "Requested RealSense USB_PORT_ID=${usb_port_id} is not connected." >&2
      echo "Connected RealSense ports: $(list_realsense_usb_port_ids | xargs echo)" >&2
      exit 2
    fi
  fi
fi
check_realsense_dependency() {
  local ros_distro="${ROS_DISTRO:-unknown}"
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command was not found in PATH. Source the ROS 2 environment first." >&2
    exit 2
  fi

  if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
    echo "RealSense ROS package 'realsense2_camera' is not available in the current environment." >&2
    echo "Current ROS_DISTRO: ${ros_distro}" >&2
    echo "Install the package with: sudo apt install ros-${ros_distro}-realsense2-camera" >&2
    echo "If you already installed it, source the matching setup script first, for example:" >&2
    echo "  source /opt/ros/${ros_distro}/setup.bash" >&2
    echo "If you do not need the integrated camera driver, start without it:" >&2
    echo "  ./launch.sh start_realsense:=false" >&2
    exit 2
  fi
}

if [[ "$start_realsense" == true ]]; then
  check_realsense_dependency
fi

if [[ "$source_tree_invocation" == true ]]; then
  # shellcheck disable=SC1090
  source "$paths_helper"
  resolve_project_paths "$project_dir"
  installed_node="$colcon_install_base/weldline_goal_publisher/lib/weldline_goal_publisher/weldline_goal_node"
  if [[ ! -x "$installed_node" ]]; then
    "$project_dir/build.sh"
  fi
  set +u
  # shellcheck disable=SC1091
  source "$colcon_install_base/setup.bash"
  set -u
fi
weights="${WEIGHTS:-$weights_default}"
echo "Starting weldline bringup: RealSense=${start_realsense}, goal_topic=/weldline/goal_pose" >&2
launch_command=(
  ros2 launch weldline_goal_publisher yolo_weldline_3d.launch.xml
  params_file:="$parameter_config"
  weights:="$weights"
  usb_port_id:="$usb_port_id"
  start_realsense:="$start_realsense"
  "${ros_launch_args[@]}"
)
if command -v setsid >/dev/null 2>&1; then
  setsid "${launch_command[@]}" &
  ros_launch_uses_group=true
else
  "${launch_command[@]}" &
fi
ros_launch_pid="$!"
set +e
wait "$ros_launch_pid"
launch_status="$?"
set -e
exit "$launch_status"
