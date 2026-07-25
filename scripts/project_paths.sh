#!/usr/bin/env bash
# Keep generated files in this standalone package, matching cocelo-hd-autonomy-light.

resolve_project_paths() {
  local package_dir="$1"

  package_dir="$(cd "$package_dir" && pwd)"
  project_root="$package_dir"
  colcon_build_base="${COCELO_BUILD_BASE:-$project_root/build}"
  colcon_install_base="${COCELO_INSTALL_BASE:-$project_root/install}"
  colcon_log_base="${COCELO_LOG_BASE:-$project_root/log}"
  dependency_cache="${ONNXRUNTIME_CACHE_DIR:-$project_root/third_party}"
}
