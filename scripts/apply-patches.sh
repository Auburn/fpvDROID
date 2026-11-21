#!/usr/bin/env bash
# Apply patches/ to the wifi-sdr and devourer submodules, for building wifi-sdr outside kepyur's CMake (which
# does this itself). Idempotent: a patch already in is left alone. The work is native/cmake/patches.cmake's.
set -euo pipefail
exec cmake -P "$(cd "$(dirname "$0")/.." && pwd -P)/native/cmake/patches.cmake"
