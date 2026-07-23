#!/usr/bin/env bash
# Creates a native-architecture Debian package with bloom + dpkg-buildpackage.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
machine_arch="$(dpkg --print-architecture)"
case "$machine_arch" in amd64|arm64|armhf) ;; *) echo "Unsupported Debian architecture: $machine_arch" >&2; exit 2;; esac
if [[ -n "${UBUNTU_CODENAME:-}" ]]; then
  ubuntu_codename="$UBUNTU_CODENAME"
elif [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  ubuntu_codename="${VERSION_CODENAME:-}"
fi
[[ -n "${ubuntu_codename:-}" ]] || {
  echo "Cannot determine OS codename. Set UBUNTU_CODENAME explicitly." >&2
  exit 2
}
command -v bloom-generate >/dev/null || { echo "Install bloom: sudo apt install python3-bloom" >&2; exit 2; }
command -v dpkg-buildpackage >/dev/null || { echo "Install dpkg-dev." >&2; exit 2; }
# shellcheck disable=SC1091
source "$project_dir/scripts/ros_environment.sh"
source_ros_environment
cd "$project_dir"
export WELDLINE_ONNX_MODEL="${WELDLINE_ONNX_MODEL:-$project_dir/weights/best.onnx}"
export ONNXRUNTIME_ROOT="$("$project_dir/scripts/ensure_onnxruntime.sh")"
if [[ -e debian ]]; then
  echo "debian/ already exists. Review and remove it before regenerating packaging metadata." >&2
  exit 2
fi
bloom-generate rosdebian --os-name ubuntu --os-version "$ubuntu_codename" --ros-distro "$ros_distro"
dpkg-buildpackage -b -us -uc -a"$machine_arch"
mkdir -p dist
package_file="$(find .. -maxdepth 1 -type f -name "ros-${ros_distro}-weldline-reflectivity-detector_*_${machine_arch}.deb" -print -quit)"
[[ -n "$package_file" ]] || { echo "Expected Debian package was not produced." >&2; exit 1; }
package_version="$(dpkg-deb -f "$package_file" Version | tr '/:' '__')"
artifact="dist/weldline_detector_${package_version}_ros-${ros_distro}_${machine_arch}.deb"
cp -f "$package_file" "$artifact"
{
  echo "package=$(dpkg-deb -f "$package_file" Package)"
  echo "version=$(dpkg-deb -f "$package_file" Version)"
  echo "architecture=$machine_arch"
  echo "ros_distro=$ros_distro"
  echo "ubuntu_codename=$ubuntu_codename"
} > "${artifact%.deb}.build-info"
echo "Created: $project_dir/$artifact"
