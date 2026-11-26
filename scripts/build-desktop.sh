#!/usr/bin/env bash
# Desktop tools: kp_replay (recordings), kp_live (adapter plugged into this PC), kp_sim, kp_gen, and the native
# unit tests (run here: no adapter needed). Run inside nix-shell.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cmake -S "$ROOT/native" -B "$ROOT/build/desktop" -G Ninja -DFPVDROID_RADIO=ON >/dev/null
ninja -C "$ROOT/build/desktop" kp_replay kp_live kp_sim kp_gen kp_radio_tests kp_decoder_tests kp_spectrum_tests kp_wifi_tests kp_screen_tests
ctest --test-dir "$ROOT/build/desktop" --output-on-failure
# the app's renderer off screen: only where EGL + OpenGL ES 3 development files are installed
if ninja -C "$ROOT/build/desktop" -t targets all | grep -q '^kp_render:'; then ninja -C "$ROOT/build/desktop" kp_render; fi
