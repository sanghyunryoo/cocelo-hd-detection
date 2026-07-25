#!/usr/bin/env bash
# Provide a local ONNX Runtime C++ SDK for the current CPU architecture.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "$script_dir/.." && pwd)"
version="${ONNXRUNTIME_VERSION:-1.18.1}"

if [[ -z "${ONNXRUNTIME_CACHE_DIR:-}" ]]; then
  # shellcheck disable=SC1091
  source "$script_dir/project_paths.sh"
  resolve_project_paths "$project_dir"
fi

cache_dir="${ONNXRUNTIME_CACHE_DIR:-$dependency_cache}"
root="${ONNXRUNTIME_ROOT:-$cache_dir/onnxruntime}"

if [[ -f "$root/include/onnxruntime_cxx_api.h" && -f "$root/lib/libonnxruntime.so" ]]; then
  echo "$root"
  exit 0
fi

for candidate in /usr/local /usr; do
  if [[ -f "$candidate/include/onnxruntime_cxx_api.h" && -f "$candidate/lib/libonnxruntime.so" ]]; then
    echo "$candidate"
    exit 0
  fi
done

arch="$(dpkg --print-architecture 2>/dev/null || uname -m)"
case "$arch" in
  amd64|x86_64) asset_arch="x64" ;;
  arm64|aarch64) asset_arch="aarch64" ;;
  *) echo "Unsupported ONNX Runtime architecture: $arch" >&2; exit 2 ;;
esac

archive="onnxruntime-linux-${asset_arch}-${version}.tgz"
url="https://github.com/microsoft/onnxruntime/releases/download/v${version}/${archive}"
download_dir="$cache_dir/downloads"
mkdir -p "$download_dir" "$cache_dir"

if [[ ! -f "$download_dir/$archive" ]]; then
  echo "Downloading ONNX Runtime ${version} for ${asset_arch}..." >&2
  if command -v curl >/dev/null 2>&1; then
    curl -fL "$url" -o "$download_dir/$archive"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "$download_dir/$archive" "$url"
  else
    echo "curl or wget is required to download ONNX Runtime." >&2
    exit 2
  fi
fi

rm -rf "$root.tmp"
mkdir -p "$root.tmp"
tar -xzf "$download_dir/$archive" -C "$root.tmp" --strip-components=1
rm -rf "$root"
mv "$root.tmp" "$root"
echo "$root"
