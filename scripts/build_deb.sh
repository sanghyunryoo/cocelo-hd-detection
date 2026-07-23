#!/usr/bin/env bash
# Creates a native-architecture Debian package with bloom + dpkg-buildpackage.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
machine_arch="$(dpkg --print-architecture)"
case "$machine_arch" in amd64|arm64|armhf) ;; *) echo "Unsupported Debian architecture: $machine_arch" >&2; exit 2;; esac
ros_distro="${ROS_DISTRO:-foxy}"
ubuntu_codename="${UBUNTU_CODENAME:-$(. /etc/os-release && echo "${VERSION_CODENAME:-focal}")}" 
command -v bloom-generate >/dev/null || { echo "Install bloom: sudo apt install python3-bloom" >&2; exit 2; }
command -v dpkg-buildpackage >/dev/null || { echo "Install dpkg-dev." >&2; exit 2; }
[[ -f "/opt/ros/${ros_distro}/setup.bash" ]] || { echo "ROS ${ros_distro} is not installed." >&2; exit 2; }
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ros_distro}/setup.bash"
set -u
cd "$project_dir"
if [[ -e debian ]]; then
  echo "debian/ already exists. Review and remove it before regenerating packaging metadata." >&2
  exit 2
fi
bloom-generate rosdebian --os-name ubuntu --os-version "$ubuntu_codename" --ros-distro "$ros_distro"
dpkg-buildpackage -b -us -uc -a"$machine_arch"
mkdir -p dist
find .. -maxdepth 1 -type f -name "ros-${ros_distro}-weldline-reflectivity-detector_*_${machine_arch}.deb" -exec cp -f {} dist/ \;
echo "Created packages: $project_dir/dist (architecture: $machine_arch)"
