# fpvDROID: details and development

The exact behaviour behind each tab, the web version, the source layout, and how to build and test. The
[readme](../README.md) is the short version for people who use the app.

Android analog FPV receiver for spectating, a fork of [kepyur](https://github.com/gilankpam/kepyur). An RTL8812BU Wi-Fi adapter plugged into the phone (a generic one that reports `0bda:b812`, or a TP-Link Archer T4U v3)
streams 40 MS/s raw IQ through [wifi-sdr](https://github.com/snokvist/wifi-sdr) (fork:
`gilankpam/wifi-sdr`, branch `kepyur-android`) and [devourer](https://github.com/OpenIPC/devourer); fpvDROID
decodes the channel you pick (or scans the 5.8 GHz bands for one) in colour (NTSC and PAL; grey when the colour
burst is too weak).

The picture processing (comb filter, clean OSD edges, smooth lines, sparkle repair), two-antenna diversity,
tuning beside the channel and the sturdier USB streaming come from [Play RX](https://github.com/scerstt/PlayRX),
which built them on top of kepyur for Windows.

The TP-Link Archer T3U (`2357:012d`) and T3U Plus (`2357:0138`) carry the same chip and should work the same
way, but are untested so far: reports welcome.

The latest APK from `main` is the rolling **main** pre-release (tag `main-latest`), rebuilt by CI
(`.github/workflows/android.yml`) on every push to `main`. It is a debug build signed with CI's key: if a
locally built fpvDROID is installed, uninstall it first (`adb uninstall com.fpvdroid.app`). Builds from before
the rename are `com.kepyur.app`, a separate app that stays installed beside it until uninstalled.

## The app

The picture fills the screen. The menu lies over a black screen, with the picture moved into a small preview
at the foot of its left-hand column; the menu is open when the app starts. Tapping the preview closes the menu
and leaves nothing on screen but the picture. A tap on the picture brings up a **MENU** button (bottom centre)
for three seconds; Back does the same. Where there is no picture (no adapter, a scan or sweep, a channel with nothing on
it, a picture that has stopped), full size or in the preview, colour bars show the chosen channel (**R8**), or
**NO SIGNAL** before one has been chosen.

- **Channel** — the band (Raceband, Fatshark, Boscam A / B / E, Lowband) and its eight channels. **Scan** is
  optional: it sweeps every band, marks the channels with video on them and tunes the strongest it found (or the
  one you were on, if it found that one again). It retunes every 20 MHz and sees 33 MHz at each, so a transmitter
  close by (15–20 MHz wide) may never be whole in one: F1–F8 and R7 sit halfway between two retunes, E8 past the
  highest. Such a transmitter gets a retune of its own as soon as the scan knows where it is (up to eight a scan,
  each one more in the count under Scan). The channel is remembered for the next start.
- **Spectrum** — a spectrum analyser: the band swept over and over (5635–5955 MHz, or 5340–5640 for Lowband),
  each channel of the chosen band marked under it, the ones with something on the air lit and listed (any
  transmitter, analog or not, at least 2 MHz wide and 6 dB over the noise). A pass takes about two seconds
  (11 or 12 retunes). The picture is paused while it sweeps: the sweep starts when the tab is opened (or
  **Sweep** is pressed) and ends when a channel is picked on it, which tunes that channel, or when another tab
  is opened or the menu closed, which goes back to the channel. Besides the analog bands it can mark the
  channels of the digital systems flown beside them: **DJI** V1/V2 (the DJI FPV System's CH1–CH8; Walksnail
  Avatar uses the same ones), O3 and O4 (CH1–CH3 at 20 or 10 MHz and the one 40/60 MHz channel, from DJI's
  spec pages: the O4's CH2 and CH3 are not the O3's), and **HDZero** (R1–R8, E1, F1, F2, F4; its unlockable
  L1–L8 are the Lowband channels). A signal is then named by that system's channel, and for DJI by its width
  too, since DJI numbers its wide modes apart: 30 MHz or wider is V1/V2 at 50 Mbps (its CH1–CH3 are the 25 Mbps
  CH2, CH4 and CH6, per Oscar Liang's chart; DJI publishes only the 25 Mbps table) or the O3's or O4's widest
  channel. Only analog video can be watched, so picking a digital channel tunes nothing.

  The sweep also tells digital signals apart by itself, whichever channels are marked, from each retune's 12 ms
  of samples, and the signal's chip says so before its channel's name ("DJI R2", "HDZero R4", "DJI CH3" with
  DJI's channels marked): **DJI** by its TDD frame of 2.5 ms (it sends in bursts, with the goggles' uplink in the
  gaps), **HDZero** by a flat block that is sent without a break, 22–31 MHz wide and within 2 MHz of one of its
  channels (R1–R8, E1, F1, F2, F4, L1–L8), **HDZero narrow** the same 14–22 MHz wide (its 17 MHz mode), and
  **Digital** any other flat block 12–31 MHz wide that is there all the time or in a regular frame of its own.
  Analog video (its spectrum is not flat), Wi-Fi (bursts with no rhythm: named below), and anything narrower than 12 MHz
  (DJI's 10 MHz channels) or wider than 31 (DJI's 40 and 60 MHz modes: wider than a retune can see) are not
  judged and keep their channel's name alone. A chip that says DJI, HDZero or Digital tunes nothing either. A
  signal does not always fit in one retune's 33 MHz: one cut between two neighbouring retunes is put back
  together, and since the two grids of passes cut a signal in different places, it may be judged on one of them
  only. So a verdict is kept until the retunes over it (those at the ends reach 16.5 MHz past the band shown) have
  not found it again for 8 s, and it stays with the rest of the picture when the sweep stops. Each retune's
  samples are cut up once for all of this (its spectrum, and its 12 ms in 6.4 µs frames), in about half the time
  the spectrum alone took before (on a PC: `kp_digital_tests --time`). The detector comes from FPVDesktopWiFiReceiver. Its DJI frame was
  measured there on recordings of a real DJI transmitter (20 MHz on R2); its HDZero side has been tested only on
  synthetic signals (continuous OFDM 27 and 17 MHz wide, as Divimath describes HDZero), so a real HDZero may go
  unnamed, or be named Digital, until it has been tried. fpvDROID's own tests are synthetic too: none of it has
  been tried yet with the adapter on a real DJI or HDZero transmitter.

  It also recognises **5 GHz Wi-Fi** networks, the most common thing in the way at 5.8 GHz, and marks each one on
  the graph as a span of its own, named at the top (**Wi-Fi 149**, or **Wi-Fi 100–112** for one of 80 MHz), with a
  chip after the others that says whether it is **in use** (on the air 5 % of the time or more) or **idle**. A
  network is a channel of the Wi-Fi grid whose whole 16.6 MHz lights up at once, evenly, in bursts, and goes quiet
  again: each retune's 12 ms are looked at a tenth of a millisecond at a time, in the same slices as the digital
  signals (it adds about 0.02 ms to a retune on a PC: `kp_wifi_tests --time`). Analog video (a few MHz wide at any
  instant, its sync pulses there all the time) and a digital link that never stops are not taken for one. An idle
  network only sends its beacons (a few tenths of a millisecond, ten times a second), so it barely shows in the band
  picture, but takes its channel as soon as someone uses it. It is only caught now and then and may take a minute
  to show; every network seen stays marked for 5 minutes after it was last seen (and with the rest of the picture
  when the sweep stops), with dashed edges while only its beacons have been heard (a network of 40 or 80 MHz sends
  them on one 20 MHz channel, and lights its whole block only when it is in use). One in use shows in the band
  picture too, as something on the air: it lights its channel under the graph but has no chip beside the network's.
  Channels next to one another within one 40, 80 or 160 MHz block of the Wi-Fi grid read as one network. Every
  region's channels are looked for (36–177, the US plan, which holds Europe's): a router only uses what its country
  allows, so what is seen is what is there, and there is nothing to set. Not marked: a network right under a VTX
  that is on (that channel is taken anyway), a network so busy it is never off (it shows as something on the air),
  the shoulders of a strong network, the bursts that a VTX very close to the phone makes 10 to 45 MHz either side
  of it, and what the adapter itself makes of a network and a VTX: its third-order product, at twice the VTX's
  frequency less the network's, bursts with the network and is as flat as it, so it looks like one (a VTX on R3
  beside a network on 149 put a weak, idle "network" on 144 on a phone), and a channel there that is weaker than
  both is taken for it. Tapping a Wi-Fi chip tunes nothing. The radio lists the networks with the digital signals
  (`spectrum_json`'s "signals", of kind "wifi", in use or idle as "active"; a busy one's carrier has its kind). The
  idea comes from the channel analyser of [FPVDesktopWiFiReceiver](https://github.com/Pwn3dP4nd4/FPVDesktopWiFiReceiver).
- **View** — aspect ratio (4:3, or 16:9 for a widescreen camera), scaling (**Fit** the whole picture,
  **Stretch** it to the screen, or **Crop** it to fill the screen), size (50–150 %), and **Goggles**: the
  picture twice, side by side, for goggles that hold a phone. Goggles have a size of their own (in place of
  the screen's), and to line the two pictures up with the lenses an eye spacing slider and a horizontal and
  vertical offset that move both together.
- **Picture** — comb filter (sharper, no echo after edges), clean OSD edges (no rainbow fringe around text),
  smooth lines (no stair steps, no shimmer), sparkle repair (for a weak signal), colour comb (fewer rainbows
  on fine detail). The first three are on by default. Smooth lines and sparkle repair work on every camera;
  the other three only change an NTSC picture. None adds delay. Two more are about the delay itself. **Max screen
  Hz** (on by default) holds the display in its fastest mode while the app is in front, with or without a picture
  (Battery Saver, heat and the phone's own refresh-rate setting can still cap it), and says under it what rate
  the screen runs at; off, the display is asked for the camera's field rate, which suits a screen that has such a
  mode. **Vsync** (off by default) shows each picture whole, once its field has ended; off, each part of the
  picture goes up as soon as its lines are decoded, a strip of 32 lines at a time. Where the phone's graphics
  allow it, each strip is drawn straight into the buffer the display is showing, past the compositor: on a Pixel
  8 Pro at 120 Hz the top line is then drawn 5 ms after the adapter took it, where the compositor shows it after
  34 ms (the app's own readout), for a quarter of a core more. On other phones the strips go to the compositor
  once a refresh. The Signal tab's numbers include the picture's delay, from the adapter taking its top and
  bottom lines to the display, and say when the picture is drawn straight to the screen.
- **Signal** — sample rate (40 MS/s, or 20 for a USB link that cannot carry 40; the app also falls back by
  itself when 40 keeps restarting or losing blocks past the first 10 s of watching, and stops doing so for the
  session once 20 has locked up on that phone, where 20 is the worse of the two), diversity (both antennas combined: fit two; it adds
  about half again to the work of one of the two processing threads), tune beside the channel (keeps the
  picture off the chip's DC point; at 40 MS/s only: at 20 the stream is too narrow and the radio stays on
  the channel), gain (automatic, or by hand for a very close transmitter), and a line of numbers (USB speed,
  processing load, drops, restarts).

A scan marks a channel as found when a picture locks on the signal there. A strong analog transmitter close by (a
quad on the table next to the phone) can show a copy of itself a few dB weaker beside it, whose picture locks too:
FPVDesktopWiFiReceiver measured one on R3 at 5731 MHz with a copy at 5743 MHz, which it listed as A7. So a signal
within 15 MHz of one at least 2 dB stronger is taken for that one's copy: it is not marked, and the scan never tunes
it. Two real transmitters that close would spoil each other's picture anyway. The chosen channel is the exception:
whether a signal there is a copy cannot be told for sure, so it stays marked beside the stronger one, and the scan
does not move you from it to another transmitter.

The status at the top left says what the adapter is doing. Nothing there needs attention unless it says so:
a radio that does not start, or stops delivering samples, is closed and started again by itself, on the
channel it was on. On some phones' USB 3 ports (a Pixel 8 Pro's, for one) the chip's USB data path locks up
every few seconds to minutes (wifi-sdr's "C2H wedge", FINDINGS §4), and only opening the adapter again brings
it back: that is done at once, a gap of about three seconds without the menu coming up (the last picture
stays up, dimmed, until the new one comes), and when it keeps happening the status says that another phone or
adapter may hold up better. The shorter stream restarts that some phones make every few seconds leave a hole
of about 55 ms; the frame torn by one is not shown (the one before it stays up), at most twice a second. While watching it also says when the phone is not keeping up, and what to switch off: when
a block of samples is lost in the app's own queues, when a processing thread has no time to spare, or when
the adapter's stream gives up on three blocks within about five seconds (one now and then is normal and is
only counted, under Signal). When the picture goes away for such a reason while the menu is closed (the adapter
unplugged, the radio being started again), the menu opens by itself to say so, and closes again when the
picture is back unless it was touched meanwhile. After 30 s in the background the adapter is let go; it is
opened and tuned again when the app returns.

Dragging the Size slider or a goggles slider makes the menu step aside and shows the picture sharp, so the
change can be judged as it is made.

## Web version

The same receiver in Chrome or Edge on a computer (Windows, macOS, Linux): plug the adapter in, click Connect,
fly with the monitor as the video receiver. It is the app's own decoder compiled to WebAssembly, talking to the
adapter over WebUSB, and it watches at 20 MS/s (what WebUSB can carry). It is not hosted anywhere: build it with
`scripts/build-web.sh` and serve it with `node web/serve.mjs` ([Build](#build)). Firefox and Safari have no WebUSB.

- **Windows:** swap the adapter's driver for WinUSB once with [Zadig](https://zadig.akeo.ie/) (Options → List All
  Devices → the adapter, e.g. "802.11ac NIC" with USB ID 2357 0115 → WinUSB → Replace Driver). The page has the same steps under
  "First time on Windows?". Untested so far: reports welcome (open the page with `?debug` and use Copy report).
- **Linux:** keep `rtw88_8822bu` off the card and give your user access (a udev rule for its USB ID).
- **macOS:** nothing to set up.

On screen: the OSD pill (channel, frequency, SNR, decode → screen latency) can be dragged, ✕ / **O** hides it;
⚙ (top right) has Scan and the channels the last scan found (click one to watch it); full screen bottom right or
**F**.

## Layout

- `native/dsp/` — video decoder (`video.cpp`: sync, fields; the FM discriminator, acquisition, the sync levels, the
  hold and channel filter for a weak signal, the colour reference, the luma and colour filters,
  line drawing, sparkle repair and deinterlacing in files of their own), spectrum + pilot finding, the band picture the Spectrum tab draws
  (`BandMap`), a sweep step's 12 ms cut into time and frequency once for every measure that reads it (`step_grid.cpp`),
  DJI, HDZero and other digital signals told apart (`digital.cpp`), the Wi-Fi networks it marks (`wifi.cpp`). No
  hardware.
- `native/radio/` — the adapter (wifi-sdr `Receiver`) + a DSP thread: scan, watch, sweep, stream test; antenna
  combining (`combine.cpp`); when 40 MS/s gives way to 20 (`fallback_policy.cpp`); torn frames kept off the
  screen (`frame_gate.cpp`); the retunes of a scan or sweep and each one's capture (`steps.cpp`); the scan's
  plan and the retunes it adds for a transmitter none of them sees whole (`scan.cpp`); what a scan found,
  less a strong transmitter's copy, and what it watches after (`pilots.cpp`); the sweep's passes, what each step's
  capture is measured for, and the band picture that goes into, as the app reads it, with the signals the sweep
  has recognised in it (`sweep.cpp`).
- `native/tests/` — unit tests of the radio's and the scan's rules, the decoder's sync, the Spectrum tab's band
  picture (and the FFT under it), its telling digital signals apart and its Wi-Fi networks, no hardware (`ctest`;
  `scripts/build-desktop.sh` runs them).
- `native/render/` — the picture on screen (OpenGL ES 3): layout, side-by-side view, the menu's preview,
  the NO SIGNAL card.
- `native/jni/` — the app's native side; a render thread draws each frame on the video Surface as it is decoded.
- `native/tools/` — desktop: `kp_replay` (recordings), `kp_live` (adapter on this PC), `kp_sim` (kp_live
  against a recording played back as a tunable adapter), `kp_gen` (a synthetic transmitter, analog or digital, and
  Wi-Fi beside a VTX: recordings to test with; its OFDM, `digital_tx.h`, is the tests' too), `kp_render` (the app's
  renderer off screen, to a picture file).
- `android/` — the app (Kotlin, plain Views): `MainActivity` (the screen), `MenuView` + `Widgets` (the menu),
  `UsbLink` (the adapter's life: permission, USB 3 switch, open, watch over, reopen), `Settings`.
- `patches/` — Play RX's changes to wifi-sdr and devourer, applied to the submodules by the build
  (`native/cmake/patches.cmake`; by hand: `scripts/apply-patches.sh`).
- `web/` — the web version (`fpvdroid_web.cpp` + `site/`). `scripts/` — build and phone helpers.

## Build

Everything comes from `shell.nix`:

```sh
git clone https://github.com/Auburn/fpvDROID && cd fpvDROID
git submodule update --init third_party/libusb third_party/wifi-sdr
git -C third_party/wifi-sdr submodule update --init external/devourer
nix-shell
scripts/build-desktop.sh      # build/desktop/kp_replay, kp_live, kp_sim, kp_gen (+ kp_render with EGL / GLES 3)
scripts/build-android.sh      # android/app/build/outputs/apk/debug/app-debug.apk
scripts/build-web.sh          # build/web/site; serve it with: node web/serve.mjs (http://localhost:8080)
```

The app's own code runs without a phone, in JVM tests (Robolectric) against a stand-in for the native side:
the activity and the adapter's state machine through a failed start, a radio that dies or stalls, the USB 3
switch, unplugging, the background; every menu control down to the call it makes; and the menu itself drawn
to PNG files, with a transparent background where the picture shows through.

```sh
cd android && gradle testDebugUnitTest                                                               # everything
cd android && gradle testDebugUnitTest --tests com.fpvdroid.app.MenuScreenshotTest -Pshots=/tmp/menu   # /tmp/menu/*.png
```

## On the phone

The adapter takes the USB-C port, so adb runs over Wi-Fi (Developer options → Wireless debugging):

```sh
scripts/phone.sh pair 192.168.1.23:37000 123456   # once ("Pair device with pairing code")
scripts/phone.sh connect 192.168.1.23:41000       # the port on the Wireless debugging screen
scripts/phone.sh install
scripts/phone.sh usbtest    # 60 s at 40 MS/s + 60 s at 20 MS/s, then scan/watch; logs -> logs/
scripts/phone.sh run        # normal start
scripts/phone.sh logs NAME  # pull fpvdroid.log + logcat
```

Without an adapter, `adb shell am start -n com.fpvdroid.app/.MainActivity --ez testcard true` puts a moving test
picture where the video would be (`false` takes it away): enough to try the view settings, or to line up
goggles before there is anything to watch.

To see how long a phone's USB link holds up, `--ez soak true` streams at 40 MS/s after every open instead of
watching; `fpvdroid.log` then has each stream fault as it happens (with what wifi-sdr itself reports) and the
adapter's USB 3 power management at every open. `--es env WSDR_URB_DEPTH=512` passes a switch to wifi-sdr
(several: comma-separated), in place from the next time the adapter is opened. `WSDR_WEDGE_US=0` is one: it turns
off the chip's own recovery from the adapter's USB 3 lock-up (it frees itself within the block, and the status
line counts that as `unwedged`), so that each lock-up shows in the log as a stream fault. The app's own, under the same
extra: `KP_CONCEAL=0` shows the frames torn by a stream restart (at once), and `KP_FAST_CPUS=0` lets Android put
the two processing threads on any core (from the next open; by default they are kept off the phone's slowest
cores, where they fall behind). `KP_LEVELS=0`, `KP_FLYWHEEL=0`, `KP_HOLD=0` and `KP_FILTER=0` turn off the
things the decoder does to keep its picture, to see each by itself (from the next channel or rate watched):
following a signal whose sync levels jump (a light flashing in the picture does that to some transmitters),
beginning a field where the fields before say when its vertical sync is not found, holding the line grid through a
signal too weak for its lines' hsyncs to be found (the status line then says `held`, and counts `holds`), and the
channel filter that comes in ahead of the FM detector while the sync is noisy (the status line has the `sync snr`
it goes by and how wide the filter is; it needs the hold).
The screen's, at once: `KP_FRONT_BUFFER=0` keeps Vsync off from drawing straight
into the buffer the display shows (the log says whether the phone can), so that the strips go through the
compositor; `KP_PRESENT_ALIGN=0` then puts each strip up as it comes (by default they go up once a refresh,
`KP_PRESENT_MARGIN_US` before the compositor's next frame). And `--ez vsync true|false`, `--es refresh field|max|none` (the display rate asked for,
in place of what Max screen Hz says) and `--ez menu false` (the menu closed, as when flying) for measuring the
picture's delay.

## Desktop bench

```sh
build/desktop/kp_gen rec/r4 --mhz 5769 --label R4 --cnr 12   # a synthetic transmitter: rec/r4.cs16 + .json
build/desktop/kp_gen rec/cp --mhz 5764 --centre 5770 --dev 5 --cnr 35 --copy 5776 4   # + a copy at 5776, 4 dB under (--mirror DB: its mirror image)
KP_SIM_FILE=rec/cp.cs16 build/desktop/kp_sim --scan   # "5776.2 MHz taken as a copy of 5764.2 MHz": one pilot found
build/desktop/kp_replay rec/r4.cs16 --luma-comb --adaptive --smooth --repair   # the picture switches (--comb: colour comb)
KP_SIM_FILE=rec/r4.cs16 build/desktop/kp_sim --scan --offset-tune --diversity   # the radio's switches, no hardware
KP_SIM_FILE=rec/r4.cs16 KP_SIM_LEAD_NS=146 build/desktop/kp_sim --watch 5769 --diversity   # path B 146 ns ahead of A: the combiner measures and undoes it
KP_SIM_FILE=rec/r4.cs16 build/desktop/kp_sim --sweep 5635 5955   # the Spectrum tab's sweep: its band picture every second
build/desktop/kp_gen rec/dji --digital dji --mhz 5770 --centre 5770   # a digital one: hdzero, narrow, dji, dji_idle, wifi, ofdm
KP_SIM_FILE=rec/dji.cs16 build/desktop/kp_sim --sweep 5635 5955   # ... among the band picture's "signals", its carrier with "kind":"dji"
build/desktop/kp_digital_tests --time   # what a sweep step costs: its spectrum, its grid, telling digital signals apart
build/desktop/kp_spectrum_tests --time  # an FFT, planned and as it was before (twiddles by recurrence)
build/desktop/kp_gen rec/wifi --mhz 5785 --centre 5770 --wifi 153   # a Wi-Fi network in use on channel 153 beside a VTX (--wifi-quiet 153: beacons only)
KP_SIM_FILE=rec/wifi.cs16 build/desktop/kp_sim --sweep 5635 5955   # ... among the "signals", one of "kind":"wifi": Wi-Fi 153 within a second (a quiet one: within a minute)
build/desktop/kp_wifi_tests --time   # what reading a sweep step's bursts for Wi-Fi costs, against the whole step
KP_SIM_FILE=rec/r4.cs16 KP_SIM_DROP_EVERY=700 build/desktop/kp_sim --watch 5769   # a link that loses every 700th block: falls back to 20 MS/s (after the 10 s grace)
KP_SIM_FILE=rec/r4.cs16 KP_SIM_RESTART_EVERY_MS=1500 build/desktop/kp_sim --watch 5769   # a stream that restarts every 1.5 s: torn frames hidden, then 20 MS/s
KP_SIM_FILE=rec/r4.cs16 build/desktop/kp_sim --watch 5769 --msps 20               # watch at 20 MS/s (kp_live takes --msps too)
build/desktop/kp_render out.ppm --view 2400x1080 --aspect 16:9 --fit crop --stereo   # what a view setting draws
build/desktop/kp_render out.ppm --scale 0.8 --stereo --eye -0.02 --offset 0,0.04   # goggles: each picture 2 % of the width further out, both raised 4 % of the height
build/desktop/kp_render out.ppm --nofeed --label R8 --inset 1 --box 68,722,493,289 --radius 37   # no picture on R8, in the menu's preview
build/desktop/kp_replay ../vtx-spike/rec/demo.cs16            # lock numbers
build/desktop/kp_replay ../vtx-spike/rec/r4.cs16 --cnr 8      # with noise added (CNR in an 18 MHz channel)
build/desktop/kp_replay ../vtx-spike/rec/r4.cs16 --frames | ffplay -f rawvideo -pixel_format rgb0 -video_size 720x484 -i -
KP_SIM_FILE=../vtx-spike/rec/r4.cs16 build/desktop/kp_sim --scan   # scan -> watch without hardware
build/desktop/kp_gen rec/f2 --mhz 5760 --centre 5770 --cnr 35 --label F2   # F2, halfway between the 5750 and 5770 retunes
KP_SIM_FILE=rec/f2.cs16 build/desktop/kp_sim --scan   # right after 5770, a retune at 5760 for it ("scan: 5770.0 MHz step: extra step at 5760...")
build/desktop/kp_live --scan --frames | ffplay -f rawvideo -pixel_format rgb0 -video_size 720x484 -framerate 60 -fflags nobuffer -i -
```

Measured on the spike's recordings (R4 NTSC VTX on the bench): 96.5 % of lines locked, no lock losses
over 10 s; the picture holds to about 5 dB CNR (it gets snowy, it doesn't drop).

License: GPL-2.0 (wifi-sdr is GPL-2.0-only).
