# Patches to the submodules

Changes to `third_party/wifi-sdr` and its `external/devourer` that are not (yet) upstream. They come from
[Play RX](https://github.com/scerstt/PlayRX) (GPL-2.0, like fpvDROID), a Windows receiver built on kepyur, and
were regenerated against the submodule commits this repository pins. The wifi-sdr patch also carries
additions of fpvDROID's own (below), on top of Play RX's content, which is otherwise as Play RX has it.

| Patch | Applies in | What it carries |
|---|---|---|
| `wifi-sdr-0001-playrx-stream.patch` | `third_party/wifi-sdr` | `RxSettings::centre` (put the hardware centre beside the pilot, off the chip's DC point; `shift_hz` stays `freq - centre`); late or short banks in the first few of a run dropped instead of restarting the stream (behind a USB hub the first bank ships late every time); mid-run, an occasional late or overrun bank dropped instead of a restart (a burst still ends the run); `request_usb3(..., force)`; `WSDR_URB_DEPTH` and `WSDR_FW_TABLE_OFLD` overrides, the firmware table offload on by default only on Windows. **fpvDROID's additions** are listed below |
| `devourer-0001-playrx-usb.patch` | `third_party/wifi-sdr/external/devourer` | control reads retried with a back-off; a circuit breaker that fails every control transfer at once after 8 failures in a row (or one NO_DEVICE), instead of minutes of retries on a vanished adapter; on Windows only: synchronous EP0 writes and retried writes |

The `#ifdef _WIN32` parts are inert on Android, Linux and the web build.

## fpvDROID's additions to the wifi-sdr patch

Their comments are marked `fpvDROID:`. How each was found, and what it measured on the phones, is in
[c2h-wedge.md](c2h-wedge.md).

- **Dropped banks are reported while a run goes on**, in `RxStatus::mid_drops` and `RxStatus::start_late_skips`
  (through `StreamCfg::on_drop`), and `BankMeta::seq` steps over them. Without that nothing above the stream
  could tell that banks were missing: a run that keeps dropping one now and then never ends, so never returns
  its `StreamStats`.
- **The C2H wedge is freed without opening the adapter again.** On some USB 3 hosts the chip's C2H engine stops
  taking requests (wifi-sdr's FINDINGS §4). The onboard program (`src/fw/stream8822b.S`, `SEND`) switches the
  RX DMA enables off and on when a request has been pending for `StreamCfg::unwedge_us`: 100 us on Android at
  USB 3 once a run has ended wedged on this host, off elsewhere (`WSDR_WEDGE_US` sets it from the start, 0 for
  off). The stream puts the freed packet back together, drops only the bank of a packet that came cut short,
  and skips transfers that are none of its own.
- **A bank is late part by part:** when one of its four data packets started after the ADC came back to it, by
  the times the trailer now carries (`TrailerStamps`). Before, it was late when the whole bank ended after the
  ADC came back to its start, which dropped every bank of a host that sends a little slower.
- **The onboard program skips a bank that starts too late** to go out whole before the ADC is back at it (20 us
  late at 40 MS/s, ~225 us at 20 MS/s), and the next bank goes on time. The stream counts skipped banks as
  dropped, not as a burst (`StreamCfg::skip_late`; `WSDR_SKIP=0` turns it off).
- **Why a receiver gave up** is in `RxStatus::gave_up` (`RxGaveUp::Wedged`: open the adapter again;
  `StreamFailing`: five failed starts in a row), so its owner need not read the error text.
  `RxStatus::unwedged` counts the wedges freed.
- **A restart leaves a shorter hole:** 10 ms between starts instead of 100, and a drain of 10 ms instead of 30.
- **Whether the link is SuperSpeed** comes from the descriptors when the kernel cannot say (older Android
  kernels).

## Applying

The build applies them: `native/CMakeLists.txt` and `web/CMakeLists.txt` include `native/cmake/patches.cmake`,
which applies each patch unless it is already in, and stops the configure if one neither applies nor is in (the
submodule pin moved without the patch). To build wifi-sdr some other way, run `scripts/apply-patches.sh` first:
it runs the same file as a script (`cmake -P native/cmake/patches.cmake`).
`.gitmodules` has `ignore = dirty` for wifi-sdr, so the patched working trees do not show in `git status`.

It works on the sources as plain directories too (no submodule metadata: an archive, or fpvDROID copied into
another repository), and on a copy of a checkout made without its top `.git` directory, whose submodules still
have a `.git` file that now leads nowhere. It checks after the fact that a line each patch adds is really in
the tree: the marker, at the end of `patches.cmake`. A patch that adds or removes that line needs its marker
changed with it.

## Regenerating one

After changing the patched code, or after moving a submodule pin and fixing whatever no longer applies:

    git -C third_party/wifi-sdr diff --ignore-submodules > patches/wifi-sdr-0001-playrx-stream.patch
    git -C third_party/wifi-sdr/external/devourer diff --ignore-submodules > patches/devourer-0001-playrx-usb.patch
