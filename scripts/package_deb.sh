#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/package_deb.sh [options]

Build a source-free runtime .deb for cocelo weldline detection.

Assumption:
  The target already has the same Ubuntu/ROS 2 pair used to build this package.
  This package bundles the weldline detector runtime install tree, ONNX weights,
  ONNX Runtime shared library prepared by build.sh, editable config, launcher,
  visualizer helper, and doctor helper.

Options:
  --version VERSION       Debian package version. Default: package.xml version.
  --revision REV          Debian revision base. Default: 1.
  --output-dir DIR        Output directory. Default: dist.
  --ros-distro NAME       ROS distro. Default: ROS_DISTRO or auto-detected /opt/ros.
  --skip-build            Package the existing install tree without rebuilding.
  --no-strip              Do not strip runtime binaries/libraries.
  -h, --help              Show this help.

The package installs:
  /opt/cocelo/weldline-detector/install   ROS 2 runtime install tree
  /etc/cocelo/weldline-detector           Editable runtime config
  /usr/bin/weldline-detector              Runtime launcher
  /usr/bin/weldline-detector-visualize    USB/RGB/detection visualizer
  /usr/bin/weldline-detector-doctor       Runtime self-check helper
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-}"
VERSION=""
REVISION="1"
OUTPUT_DIR="${REPO_DIR}/dist"
SKIP_BUILD="false"
DO_STRIP="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --version)
      VERSION="${2:?--version requires a value}"
      shift 2
      ;;
    --version=*)
      VERSION="${1#--version=}"
      shift
      ;;
    --revision)
      REVISION="${2:?--revision requires a value}"
      shift 2
      ;;
    --revision=*)
      REVISION="${1#--revision=}"
      shift
      ;;
    --output-dir)
      OUTPUT_DIR="${2:?--output-dir requires a directory}"
      shift 2
      ;;
    --output-dir=*)
      OUTPUT_DIR="${1#--output-dir=}"
      shift
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"
      shift 2
      ;;
    --ros-distro=*)
      ROS_DISTRO_NAME="${1#--ros-distro=}"
      shift
      ;;
    --skip-build)
      SKIP_BUILD="true"
      shift
      ;;
    --no-strip)
      DO_STRIP="false"
      shift
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

read_package_version() {
  python3 - "${REPO_DIR}/package.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
print(root.findtext("version", "0.1.0"))
PY
}

detect_deb_arch() {
  local arch=""

  if command -v dpkg >/dev/null 2>&1; then
    arch="$(dpkg --print-architecture)"
  else
    case "$(uname -m)" in
      x86_64|amd64) arch="amd64" ;;
      aarch64|arm64) arch="arm64" ;;
      *) arch="" ;;
    esac
  fi

  case "${arch}" in
    amd64|arm64)
      printf '%s\n' "${arch}"
      ;;
    *)
      echo "error: unsupported build architecture: ${arch:-$(uname -m)}" >&2
      echo "supported Debian architectures: amd64, arm64" >&2
      exit 1
      ;;
  esac
}

read_ubuntu_version_id() {
  local version_id=""

  if [[ -r /etc/os-release ]]; then
    # shellcheck source=/dev/null
    source /etc/os-release
    version_id="${VERSION_ID:-}"
  fi

  if [[ -z "${version_id}" ]] && command -v lsb_release >/dev/null 2>&1; then
    version_id="$(lsb_release -rs)"
  fi

  if [[ -z "${version_id}" ]]; then
    echo "error: unable to detect Ubuntu version from /etc/os-release" >&2
    exit 1
  fi

  printf '%s\n' "${version_id}"
}

preferred_ros_for_ubuntu() {
  case "$1" in
    20.04) printf '%s\n' "foxy" ;;
    22.04) printf '%s\n' "humble" ;;
    24.04) printf '%s\n' "jazzy" ;;
    *) printf '%s\n' "" ;;
  esac
}

detect_ros_distro() {
  local ubuntu_version="$1"
  local requested="${ROS_DISTRO_NAME}"
  local ros_root="/opt/ros"

  if [[ -n "${requested}" ]]; then
    if [[ ! -f "${ros_root}/${requested}/setup.bash" ]]; then
      echo "error: /opt/ros/${requested}/setup.bash not found" >&2
      exit 1
    fi
    printf '%s\n' "${requested}"
    return
  fi

  local preferred
  preferred="$(preferred_ros_for_ubuntu "${ubuntu_version}")"
  if [[ -n "${preferred}" && -f "${ros_root}/${preferred}/setup.bash" ]]; then
    printf '%s\n' "${preferred}"
    return
  fi

  local candidates=()
  local setup_file
  shopt -s nullglob
  for setup_file in "${ros_root}"/*/setup.bash; do
    candidates+=("$(basename "$(dirname "${setup_file}")")")
  done
  shopt -u nullglob

  if [[ "${#candidates[@]}" -eq 1 ]]; then
    printf '%s\n' "${candidates[0]}"
    return
  fi

  if [[ "${#candidates[@]}" -gt 1 ]]; then
    echo "error: multiple ROS distros found under /opt/ros: ${candidates[*]}" >&2
    echo "hint: choose one with --ros-distro NAME." >&2
  else
    echo "error: no ROS 2 setup.bash found under /opt/ros." >&2
  fi
  exit 1
}

ros_ubuntu_revision_suffix() {
  local ros_distro="$1"
  local ubuntu_version="$2"
  printf '%s%s\n' "${ros_distro}" "${ubuntu_version}"
}

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -e "${src}" ]]; then
    cp -aL "${src}" "${dst}"
  fi
}

copy_install_tree() {
  local src="$1"
  local dst_parent="$2"
  local base
  base="$(basename "${src}")"

  mkdir -p "${dst_parent}"
  (
    cd "$(dirname "${src}")"
    find "${base}" \
      \( \( -type l ! -exec test -e {} \; \) -o -name __pycache__ -o -name '*.pyc' \) -prune -o \
      -print0 |
      tar --null --dereference --no-recursion --files-from - -cf -
  ) | (
    cd "${dst_parent}"
    tar -xf -
  )
}

if [[ -z "${VERSION}" ]]; then
  VERSION="$(read_package_version)"
fi

ARCH="$(detect_deb_arch)"
UBUNTU_VERSION_ID="$(read_ubuntu_version_id)"
ROS_DISTRO_NAME="$(detect_ros_distro "${UBUNTU_VERSION_ID}")"
REVISION_SUFFIX="$(ros_ubuntu_revision_suffix "${ROS_DISTRO_NAME}" "${UBUNTU_VERSION_ID}")"
PACKAGE_VERSION="${VERSION}-${REVISION}+${REVISION_SUFFIX}"
PACKAGE_NAME="cocelo-weldline-detector"
STAGE_ROOT="$(mktemp -d /tmp/${PACKAGE_NAME}.XXXXXX)"
trap 'rm -rf "${STAGE_ROOT}"' EXIT

if [[ "${SKIP_BUILD}" != "true" ]]; then
  ROS_DISTRO="${ROS_DISTRO_NAME}" "${REPO_DIR}/build.sh"
fi

INSTALL_ROOT="${REPO_DIR}/install"
PACKAGE_INSTALL_ROOT="${INSTALL_ROOT}/weldline_reflectivity_detector"
if [[ ! -d "${PACKAGE_INSTALL_ROOT}" ]]; then
  echo "error: missing install tree: ${PACKAGE_INSTALL_ROOT}" >&2
  echo "hint: run ${REPO_DIR}/build.sh first, or omit --skip-build." >&2
  exit 1
fi

mkdir -p \
  "${STAGE_ROOT}/DEBIAN" \
  "${STAGE_ROOT}/opt/cocelo/weldline-detector/install" \
  "${STAGE_ROOT}/etc/cocelo/weldline-detector" \
  "${STAGE_ROOT}/usr/bin" \
  "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}"

for file in setup.bash setup.sh setup.zsh local_setup.bash local_setup.sh local_setup.zsh \
  _local_setup_util_sh.py COLCON_IGNORE .colcon_install_layout
do
  copy_if_exists "${INSTALL_ROOT}/${file}" "${STAGE_ROOT}/opt/cocelo/weldline-detector/install/"
done

copy_install_tree "${PACKAGE_INSTALL_ROOT}" "${STAGE_ROOT}/opt/cocelo/weldline-detector/install/"

mkdir -p "${STAGE_ROOT}/opt/cocelo/weldline-detector/install/share/colcon-core/packages"
copy_if_exists \
  "${INSTALL_ROOT}/share/colcon-core/packages/weldline_reflectivity_detector" \
  "${STAGE_ROOT}/opt/cocelo/weldline-detector/install/share/colcon-core/packages/"

cp -aL "${REPO_DIR}/config/yolo_weldline_3d.yaml" \
  "${STAGE_ROOT}/etc/cocelo/weldline-detector/yolo_weldline_3d.yaml"

cp -aL "${REPO_DIR}/README.md" "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/README.md"
cat > "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/runtime_assumptions.txt" <<EOF
cocelo-weldline-detector runtime assumptions
============================================

This .deb is intended to be self-contained for weldline-detector-specific
runtime artifacts while assuming the target system already provides:

- Ubuntu userspace for ${ARCH}
- ROS 2 ${ROS_DISTRO_NAME} installed at /opt/ros/${ROS_DISTRO_NAME}
- Python 3, NumPy, OpenCV Python, and pyrealsense2 for the visualizer
- Intel RealSense ROS 2 driver from the configured ROS apt repository

Bundled in this package:

- weldline_reflectivity_detector binary, launch, config, and package resources
- ONNX model weights installed with the package
- ONNX Runtime shared library used by the C++ detector
- direct RealSense USB visualizer script

The package deliberately does not bundle /opt/ros/${ROS_DISTRO_NAME}, glibc,
libstdc++, OpenCV system libraries, or the RealSense driver package.
EOF

cat > "${STAGE_ROOT}/usr/bin/weldline-detector" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO_NAME}"
export WELDLINE_CONFIG="\${WELDLINE_CONFIG:-/etc/cocelo/weldline-detector/yolo_weldline_3d.yaml}"

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  set +u
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
else
  echo "error: /opt/ros/${ROS_DISTRO_NAME}/setup.bash not found" >&2
  exit 1
fi

set +u
source "/opt/cocelo/weldline-detector/install/setup.bash"
set -u

exec /opt/cocelo/weldline-detector/install/weldline_reflectivity_detector/lib/weldline_reflectivity_detector/launch.sh "\$@"
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/weldline-detector"

cat > "${STAGE_ROOT}/usr/bin/weldline-detector-visualize" <<EOF
#!/usr/bin/env bash
set -euo pipefail

exec python3 /opt/cocelo/weldline-detector/install/weldline_reflectivity_detector/lib/weldline_reflectivity_detector/realsense_visualize.py "\$@"
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/weldline-detector-visualize"

cat > "${STAGE_ROOT}/usr/bin/weldline-detector-doctor" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO_NAME}"
CONFIG_FILE="\${WELDLINE_CONFIG:-/etc/cocelo/weldline-detector/yolo_weldline_3d.yaml}"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "/opt/cocelo/weldline-detector/install/setup.bash"
set -u

echo "== weldline-detector runtime =="
echo "config: \${CONFIG_FILE}"
echo "ros_distro: ${ROS_DISTRO_NAME}"
echo

echo "== package prefixes =="
ros2 pkg prefix weldline_reflectivity_detector
ros2 pkg prefix realsense2_camera
echo

echo "== configured RealSense selection =="
if [[ -r "\${CONFIG_FILE}" ]]; then
  grep -E '^[[:space:]]*(ros_domain_id|usb_port_id|color_topic|depth_topic|camera_info_topic|weights):' "\${CONFIG_FILE}" || true
else
  echo "warning: config not readable: \${CONFIG_FILE}"
fi
echo

echo "== connected RealSense devices =="
weldline-detector-visualize --list-only || true
echo

echo "== weldline topics on current ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} =="
ros2 topic list | grep -E 'weldline|goal_pose|camera' || true
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/weldline-detector-doctor"

cat > "${STAGE_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${PACKAGE_VERSION}
Section: robotics
Priority: optional
Architecture: ${ARCH}
Maintainer: Cocelo <engineering@cocelo.ai>
Depends: bash, python3, python3-numpy, python3-opencv, ros-${ROS_DISTRO_NAME}-rclpy, ros-${ROS_DISTRO_NAME}-geometry-msgs, ros-${ROS_DISTRO_NAME}-sensor-msgs, ros-${ROS_DISTRO_NAME}-realsense2-camera, ros-${ROS_DISTRO_NAME}-ros2launch
Description: Cocelo weldline detector runtime
 Source-free runtime bundle for RealSense RGB-D weldline detection, Nav2 goal
 output, debug topics, ONNX Runtime inference, and USB/detection visualization.
 This package assumes ROS 2 ${ROS_DISTRO_NAME} is already installed on the target
 system.
EOF

cat > "${STAGE_ROOT}/DEBIAN/conffiles" <<'EOF'
/etc/cocelo/weldline-detector/yolo_weldline_3d.yaml
EOF

cat > "${STAGE_ROOT}/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e
echo
echo "cocelo-weldline-detector installed."
echo "Edit config: /etc/cocelo/weldline-detector/yolo_weldline_3d.yaml"
echo "Pick RealSense USB port: weldline-detector-visualize --no-detect"
echo "Run: weldline-detector"
echo "Run with debug image viewer: weldline-detector --vis"
echo "Check: weldline-detector-doctor"
echo
EOF
chmod 0755 "${STAGE_ROOT}/DEBIAN/postinst"

if [[ "${DO_STRIP}" == "true" ]]; then
  while IFS= read -r file; do
    if file "${file}" | grep -Eq 'ELF .* (executable|shared object)'; then
      strip --strip-unneeded "${file}" 2>/dev/null || true
    fi
  done < <(find "${STAGE_ROOT}/opt/cocelo/weldline-detector" -type f)
fi

find "${STAGE_ROOT}" -type d -exec chmod 0755 {} +
find "${STAGE_ROOT}/opt/cocelo/weldline-detector" -type f -name '*.sh' -exec chmod 0755 {} +
find "${STAGE_ROOT}/opt/cocelo/weldline-detector/install" -type f \
  -path '*/lib/weldline_reflectivity_detector/*' \
  -exec chmod 0755 {} +

mkdir -p "${OUTPUT_DIR}"
DEB_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}_${PACKAGE_VERSION}_${ARCH}.deb"
dpkg-deb --build --root-owner-group "${STAGE_ROOT}" "${DEB_PATH}"

{
  echo "package=${PACKAGE_NAME}"
  echo "version=${PACKAGE_VERSION}"
  echo "architecture=${ARCH}"
  echo "ros_distro=${ROS_DISTRO_NAME}"
  echo "ubuntu_version=${UBUNTU_VERSION_ID}"
  echo "install_root=/opt/cocelo/weldline-detector/install"
  echo "config=/etc/cocelo/weldline-detector/yolo_weldline_3d.yaml"
} > "${DEB_PATH%.deb}.build-info"

echo
echo "Created: ${DEB_PATH}"
echo
cat <<EOF
Install on target ${ARCH} system:
  cd "$(cd "${OUTPUT_DIR}" && pwd)"
  sudo apt install ./$(basename "${DEB_PATH}")

Runtime config:
  sudoedit /etc/cocelo/weldline-detector/yolo_weldline_3d.yaml

Pick the RealSense USB port before running detection:
  weldline-detector-visualize --no-detect

Run detection:
  weldline-detector

Run detection with annotated image + 3D XYZ viewer:
  weldline-detector --vis

Check installation / topics / RealSense visibility:
  weldline-detector-doctor

Packaged runtime:
  install tree: /opt/cocelo/weldline-detector/install
  config:       /etc/cocelo/weldline-detector/yolo_weldline_3d.yaml
  docs:         /usr/share/doc/${PACKAGE_NAME}/

Runtime assumption:
  ROS 2 ${ROS_DISTRO_NAME} is pre-installed at /opt/ros/${ROS_DISTRO_NAME}.
  The package depends on ros-${ROS_DISTRO_NAME}-realsense2-camera and ros-${ROS_DISTRO_NAME}-ros2launch.
  For the USB visualizer, pyrealsense2 must be available on the target Python.
EOF
