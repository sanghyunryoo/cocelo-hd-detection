#!/usr/bin/env bash
# Build a lightweight native Debian package from the local colcon install tree.
# This is intentionally not a bloom release script; it is for field/CI artifacts.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_name="cocelo-weldline-detector"
machine_arch="$(dpkg --print-architecture)"
case "$machine_arch" in amd64|arm64|armhf) ;; *) echo "Unsupported Debian architecture: $machine_arch" >&2; exit 2;; esac
command -v dpkg-deb >/dev/null || { echo "dpkg-deb is required. Install the base dpkg package." >&2; exit 2; }

# shellcheck disable=SC1091
source "$project_dir/scripts/ros_environment.sh"
source_ros_environment

version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$project_dir/package.xml" | head -1)"
[[ -n "$version" ]] || { echo "Cannot read package version from package.xml." >&2; exit 2; }

export WELDLINE_ONNX_MODEL="${WELDLINE_ONNX_MODEL:-$project_dir/weights/best.onnx}"
[[ "$WELDLINE_ONNX_MODEL" = /* ]] || export WELDLINE_ONNX_MODEL="$project_dir/$WELDLINE_ONNX_MODEL"
export ONNXRUNTIME_ROOT="$("$project_dir/scripts/ensure_onnxruntime.sh")"

"$project_dir/build.sh"

install_prefix="$project_dir/install/weldline_reflectivity_detector"
binary="$install_prefix/lib/weldline_reflectivity_detector/yolo_weldline_3d_node"
[[ -x "$binary" ]] || { echo "Built detector binary was not found: $binary" >&2; exit 1; }

dist_dir="$project_dir/dist"
work_dir="$dist_dir/.deb_build_${package_name}_${ros_distro}_${machine_arch}"
stage_dir="$work_dir/root"
rm -rf "$work_dir"
mkdir -p "$stage_dir/DEBIAN" "$stage_dir/opt/ros/$ros_distro" "$dist_dir"
cp -a "$install_prefix/." "$stage_dir/opt/ros/$ros_distro/"

declare -a depends=(
  "libc6"
  "libgcc-s1"
  "libstdc++6"
  "python3"
  "python3-numpy"
  "python3-opencv"
  "ros-${ros_distro}-rclcpp"
  "ros-${ros_distro}-std-msgs"
  "ros-${ros_distro}-sensor-msgs"
  "ros-${ros_distro}-geometry-msgs"
  "ros-${ros_distro}-visualization-msgs"
  "ros-${ros_distro}-tf2"
  "ros-${ros_distro}-tf2-ros"
  "ros-${ros_distro}-tf2-geometry-msgs"
  "ros-${ros_distro}-message-filters"
  "ros-${ros_distro}-cv-bridge"
  "ros-${ros_distro}-image-transport"
  "ros-${ros_distro}-realsense2-camera"
  "ros-${ros_distro}-ros2launch"
)

for library_name in libopencv_core libopencv_imgproc; do
  library_path="$(ldd "$binary" | awk -v name="$library_name" '$1 ~ name {print $3; exit}')"
  [[ -n "$library_path" && -e "$library_path" ]] || continue
  real_library_path="$(readlink -f "$library_path")"
  package="$(dpkg-query -S "$library_path" "$real_library_path" 2>/dev/null | head -1 | cut -d: -f1 || true)"
  [[ -n "$package" ]] && depends+=("$package")
done

depends_csv="$(
  printf '%s\n' "${depends[@]}" | sed '/^$/d' | sort -u |
    awk 'BEGIN { sep = "" } { printf "%s%s", sep, $0; sep = ", " } END { print "" }'
)"

cat > "$stage_dir/DEBIAN/control" <<CONTROL
Package: ${package_name}
Version: ${version}
Section: robotics
Priority: optional
Architecture: ${machine_arch}
Maintainer: Cocelo Engineering <engineering@cocelo.ai>
Depends: ${depends_csv}
Description: Cocelo ROS 2 weldline RGB-D detector
 Native ROS 2 C++ weldline detector package with RealSense bringup,
 Nav2-compatible goal output, debug topics, and bundled ONNX Runtime.
CONTROL

artifact="$dist_dir/weldline_detector_${version}_ros-${ros_distro}_${machine_arch}.deb"
dpkg-deb --root-owner-group --build "$stage_dir" "$artifact"
{
  echo "package=$package_name"
  echo "version=$version"
  echo "architecture=$machine_arch"
  echo "ros_distro=$ros_distro"
  echo "install_prefix=/opt/ros/$ros_distro"
  echo "onnxruntime_root=$ONNXRUNTIME_ROOT"
  echo "depends=$depends_csv"
} > "${artifact%.deb}.build-info"
rm -rf "$work_dir"
echo "Created: $artifact"
