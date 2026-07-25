#!/usr/bin/env bash
# Build the native ROS 2 C++ package using the system Python required by ROS tooling.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$project_dir/scripts/ros_environment.sh"
# shellcheck disable=SC1091
source "$project_dir/scripts/project_paths.sh"
source_ros_environment
resolve_project_paths "$project_dir"

model_path="${WELDLINE_ONNX_MODEL:-$project_dir/weights/best.onnx}"
[[ "$model_path" = /* ]] || model_path="$project_dir/$model_path"
onnxruntime_root="$(
  ONNXRUNTIME_CACHE_DIR="$dependency_cache" "$project_dir/scripts/ensure_onnxruntime.sh"
)"

mkdir -p "$colcon_build_base" "$colcon_install_base" "$colcon_log_base"
printf 'Building weldline_reflectivity_detector\n  source:  %s\n  build:   %s\n  install: %s\n  log:     %s\n' \
  "$project_dir" "$colcon_build_base" "$colcon_install_base" "$colcon_log_base"

colcon --log-base "$colcon_log_base" build \
  --base-paths "$project_dir" \
  --build-base "$colcon_build_base" \
  --install-base "$colcon_install_base" \
  --packages-select weldline_reflectivity_detector \
  --cmake-args \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DWELDLINE_ONNX_MODEL="$model_path" \
  -DONNXRUNTIME_ROOT="$onnxruntime_root"
