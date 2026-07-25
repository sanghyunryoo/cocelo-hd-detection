#!/usr/bin/env bash
# Shared ROS 2 environment discovery for build, launch, and native Debian packaging.

resolve_ros_environment() {
  local -a installed_distros=()
  local prefix candidate

  if [[ -n "${ROS_DISTRO:-}" ]]; then
    ros_distro="$ROS_DISTRO"
  else
    # Prefer a sourced ROS installation even when an overlay workspace is active.
    IFS=':' read -r -a _ros_prefixes <<< "${AMENT_PREFIX_PATH:-}"
    for prefix in "${_ros_prefixes[@]}"; do
      if [[ "$prefix" == /opt/ros/* ]] && [[ -f "$prefix/setup.bash" ]]; then
        candidate="${prefix#/opt/ros/}"
        candidate="${candidate%%/*}"
        ros_distro="$candidate"
        break
      fi
    done
  fi

  if [[ -z "${ros_distro:-}" ]]; then
    while IFS= read -r candidate; do installed_distros+=("$candidate"); done < <(
      find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort
    )
    if [[ "${#installed_distros[@]}" -eq 1 ]]; then
      ros_distro="${installed_distros[0]}"
    elif [[ "${#installed_distros[@]}" -eq 0 ]]; then
      echo "No ROS 2 distribution found under /opt/ros. Source ROS 2 or set ROS_DISTRO." >&2
      return 2
    else
      echo "Multiple ROS 2 distributions are installed: ${installed_distros[*]}" >&2
      echo "Source the intended setup.bash or set ROS_DISTRO explicitly." >&2
      return 2
    fi
  fi

  ros_setup="/opt/ros/${ros_distro}/setup.bash"
  if [[ ! -f "$ros_setup" ]]; then
    echo "ROS setup for '${ros_distro}' was not found: $ros_setup" >&2
    return 2
  fi
  export ROS_DISTRO="$ros_distro"
}

source_ros_environment() {
  resolve_ros_environment
  # Some ROS setup files reference optional variables; callers commonly use `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup"
  set -u

  # An overlay may have been moved or cleaned since the parent shell was
  # created. colcon warns for every stale entry, so retain only live prefixes.
  local variable_name value prefix joined
  local -a prefixes=()
  for variable_name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do
    value="${!variable_name:-}"
    joined=""
    prefixes=()
    IFS=':' read -r -a prefixes <<< "$value"
    for prefix in "${prefixes[@]}"; do
      [[ -n "$prefix" && -d "$prefix" ]] || continue
      joined="${joined:+$joined:}$prefix"
    done
    printf -v "$variable_name" '%s' "$joined"
    export "$variable_name"
  done
}
