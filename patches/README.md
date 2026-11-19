# Patches to the submodules

Changes to `third_party/wifi-sdr` and its `external/devourer` that are not (yet) upstream. They come from
[Play RX](https://github.com/scerstt/PlayRX) (GPL-2.0, like kepyur), a Windows receiver built on kepyur, and
were regenerated against the submodule commits this repository pins.

| Patch | Applies in | What it carries |
|---|---|---|
| `wifi-sdr-0001-playrx-stream.patch` | `third_party/wifi-sdr` | `RxSettings::centre` (put the hardware centre beside the pilot, off the chip's DC point; `shift_hz` stays `freq - centre`); late or short banks in the first few of a run dropped instead of restarting the stream (behind a USB hub the first bank ships late every time); mid-run, an occasional late or overrun bank dropped instead of a restart (a burst still ends the run); `request_usb3(..., force)`; `WSDR_URB_DEPTH` and `WSDR_FW_TABLE_OFLD` overrides, the firmware table offload on by default only on Windows |
| `devourer-0001-playrx-usb.patch` | `third_party/wifi-sdr/external/devourer` | control reads retried with a back-off; a circuit breaker that fails every control transfer at once after 8 failures in a row (or one NO_DEVICE), instead of minutes of retries on a vanished adapter; on Windows only: synchronous EP0 writes and retried writes |

The `#ifdef _WIN32` parts are inert on Android, Linux and the web build.

## Applying

The build applies them: `native/CMakeLists.txt` and `web/CMakeLists.txt` include `native/cmake/patches.cmake`,
which applies each patch unless it is already in, and stops the configure if one neither applies nor is in (the
submodule pin moved without the patch). To build wifi-sdr some other way, run `scripts/apply-patches.sh` first.
`.gitmodules` has `ignore = dirty` for wifi-sdr, so the patched working trees do not show in `git status`.

## Regenerating one

After changing the patched code, or after moving a submodule pin and fixing whatever no longer applies:

    git -C third_party/wifi-sdr diff --ignore-submodules > patches/wifi-sdr-0001-playrx-stream.patch
    git -C third_party/wifi-sdr/external/devourer diff --ignore-submodules > patches/devourer-0001-playrx-usb.patch
