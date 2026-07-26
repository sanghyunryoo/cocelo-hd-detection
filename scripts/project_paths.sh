#!/usr/bin/env bash
# Keep generated files outside the source tree.

resolve_project_paths() {
  artifact_root="${COCELO_ARTIFACT_ROOT:-${XDG_CACHE_HOME:-${HOME}/.cache}/cocelo/weldline-reflectivity-detector}"
  colcon_build_base="${COCELO_BUILD_BASE:-$artifact_root/build}"
  colcon_install_base="${COCELO_INSTALL_BASE:-$artifact_root/install}"
  colcon_log_base="${COCELO_LOG_BASE:-$artifact_root/log}"
}
