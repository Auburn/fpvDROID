#!/usr/bin/env bash
# Apply patches/ to the wifi-sdr and devourer submodules, for building wifi-sdr outside kepyur's CMake (which
# does this itself: native/cmake/patches.cmake). Idempotent: a patch already in is left alone.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)

apply() {  # submodule dir, patch
  local dir=$1 patch=$2
  if git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
    echo "already applied: $(basename "$patch")"
  elif git -C "$dir" apply --check "$patch" 2>/dev/null; then
    git -C "$dir" apply "$patch"
    echo "applied $(basename "$patch") to $dir"
  else
    echo "error: $(basename "$patch") neither applies to nor is already in $dir (at commit" \
      "$(git -C "$dir" rev-parse --short HEAD 2>/dev/null || echo '?')): the submodule pin and the patch are" \
      "out of step (patches/README.md)" >&2
    exit 1
  fi
}

apply "$ROOT/third_party/wifi-sdr" "$ROOT/patches/wifi-sdr-0001-playrx-stream.patch"
apply "$ROOT/third_party/wifi-sdr/external/devourer" "$ROOT/patches/devourer-0001-playrx-usb.patch"
