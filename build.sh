#!/usr/bin/env bash
# Build the native ROS 2 C++ package using the system Python required by ROS tooling.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$project_dir/scripts/ros_environment.sh"
source_ros_environment
cd "$project_dir"
model_path="${WELDLINE_ONNX_MODEL:-$project_dir/weights/best.onnx}"
[[ "$model_path" = /* ]] || model_path="$project_dir/$model_path"
onnxruntime_root="$("$project_dir/scripts/ensure_onnxruntime.sh")"
colcon build --packages-select weldline_reflectivity_detector --cmake-args \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DWELDLINE_ONNX_MODEL="$model_path" \
  -DONNXRUNTIME_ROOT="$onnxruntime_root"
