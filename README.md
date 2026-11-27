# fpvDROID

fpvDROID turns an Android phone into an analog FPV video receiver for spectating. Plug a cheap USB Wi-Fi
adapter with the RTL8812BU chip into the phone, pick a 5.8 GHz channel or let the app scan for one, and watch
the camera's picture in colour (NTSC and PAL).

It is a fork of [kepyur](https://github.com/gilankpam/kepyur) with a new menu, a spectrum analyser, more picture
processing and lower latency. [What changed since kepyur](#what-changed-since-kepyur) lists all of it.

![The Channel tab, with the live picture in the preview at the bottom left](docs/screenshots/channel.png)

## What you need

- A phone with Android 10 or later and a USB-C port that does USB 3.
- A USB 3 Wi-Fi adapter with the Realtek RTL8812BU chip. A cheap generic one works: fpvDROID is developed with
  [this Comfast one from AliExpress](https://www.aliexpress.com/item/1005007123363018.html) (antenna version).
- A USB 3 OTG adapter or cable between the two. Video needs USB 3, and the app tells you when the link is only USB 2.

The app accepts an adapter that reports Realtek's own USB ID for the chip (`0bda:b812`), as the Comfast does, or
one of TP-Link's Archer T4U v3, T3U and T3U Plus. The two T3U models are untested so far. Reports are welcome.

fpvDROID has been tested on a Pixel 8 Pro, a Pixel 7a and an LG G7+. The adapter takes the phone's USB-C port, so the
phone cannot charge by cable while you watch.

It shows analog video only. DJI, HDZero and Walksnail signals appear on the Spectrum tab, but the app cannot
decode them.

## Install

Download [`fpvdroid-main.apk`](https://github.com/Auburn/fpvDROID/releases/download/main-latest/fpvdroid-main.apk)
from the [main pre-release](https://github.com/Auburn/fpvDROID/releases/tag/main-latest) and open it on the
phone. That release is rebuilt on every change to `main`, so the link always gives the newest build.

It is a debug build signed with the build server's key. If a fpvDROID you built yourself is on the phone,
uninstall it first. An older kepyur build is a separate app and stays installed until you remove it.

## Watching

1. Plug the adapter into the phone and open fpvDROID. When Android asks whether the app may use the adapter,
   allow it.
2. Give it a few seconds: the adapter switches to USB 3 and the radio starts. The line at the top of the menu
   says what is going on.
3. On the Channel tab, pick a band and a channel, or press Scan to look for video on every channel.
4. Tap the small picture at the bottom left. The menu closes and the picture fills the screen.

To get the menu back, press Back, or tap the picture and then the MENU button that shows for three seconds.
The app remembers your channel for next time.

![The picture full screen, with the menu closed](docs/screenshots/watching.jpg)

## Features

### Spectrum

![The Spectrum tab during a sweep, with a DJI signal, an analog one and two Wi-Fi networks](docs/screenshots/spectrum.png)

- A spectrum analyser that sweeps 5635 to 5955 MHz (5340 to 5640 MHz for Lowband) about every two seconds.
- The channels of the chosen band sit under the graph. Those with a signal are lit and listed, and tapping an
  analog one tunes it.
- The marks can switch from the analog bands to the channels of DJI (V1/V2, O3, O4) or HDZero. Walksnail Avatar
  uses the DJI V1/V2 channels.
- The sweep recognises DJI, HDZero and other digital signals and names them, whichever channels are marked.
  This part has not yet been tried on a real DJI or HDZero transmitter.
- It marks 5 GHz Wi-Fi networks, the most common thing in the way at 5.8 GHz, and says whether each is in use
  or idle.
- The picture is paused while the tab sweeps.

### View

![Goggles mode: the picture twice, side by side](docs/screenshots/goggles.jpg)

- Aspect ratio: 4:3, or 16:9 for a widescreen camera.
- Scaling: fit the whole picture with black bars, stretch it to the screen, or crop it to fill the screen.
- Size, from 50 to 150 %.
- Screen brightness: the phone's own, or a level you set for the picture alone. The set level applies while the
  menu is closed.
- Goggles: the picture twice, side by side, for goggles that hold a phone. Sliders for size, eye spacing and
  horizontal and vertical offset line the two pictures up with the lenses.

While you drag the size slider or a goggles slider, the menu steps aside so you can judge the change on the
picture.

### Picture

![The Picture tab](docs/screenshots/picture.png)

| Setting | What it does | Default |
|---|---|---|
| Comb filter | A sharper picture with no echo after edges | On |
| Clean OSD edges | No rainbow fringe or halo around text and sharp edges | On |
| Smooth lines | A steady picture with clean diagonals instead of stair steps | On |
| Sparkle repair | Patches the white and black sparkles of a weak signal | Off |
| Colour comb | Fewer rainbows on fine detail, at the cost of more processing | Off |
| Max screen Hz | Holds the screen at its fastest refresh rate | On |
| Vsync | Shows only complete pictures. Off, each part of the picture is drawn as soon as it is decoded | Off |

Comb filter, Clean OSD edges and Colour comb only change an NTSC picture, and Clean OSD edges needs the comb
filter. None of the first five adds delay.

The decoder shows NTSC and PAL in colour, and drops to black and white when the colour signal is too weak. It
holds the picture on a signal too weak for its sync pulses to be found, and on a transmitter whose sync levels
jump when a light flashes in the picture.

The last two settings are about delay. With Vsync off, on phones whose graphics allow it, the app draws
straight into the buffer the screen is showing.

#### Measured latency

On a Pixel 8 Pro with its screen at 120 Hz and the default settings (Max screen Hz on, Vsync off)
Tested using a 240fps recording, glass to screen latency using a flashing LED

| Measured | Result |
|---|---|
| The picture starts to change | 13 ms |
| Half the picture has changed | 22 ms |

### Signal

![The Signal tab with diversity on](docs/screenshots/signal.png)

- Sample rate: 40 MS/s hears more of the channel, 20 MS/s is easier on the USB link. The app switches to 20 by
  itself when 40 keeps dropping out.
- Diversity combines both of the adapter's antennas, so a fade on one is covered by the other.
- Tune beside the channel keeps the chip's blind spot out of the picture, where it would show as stripes at the
  left edge and ghost outlines. It needs 40 MS/s.
- Gain is automatic, or set by hand for a transmitter so close that it washes the picture out.
- A line of numbers under the settings gives the USB speed, processing load, dropped blocks, restarts and the
  picture's delay.

### When something goes wrong

The line at the top of the menu says what the adapter is doing. With the menu closed it shows over the picture
when there is something to report. Nothing in it needs your attention unless it says so.

- A radio that does not start, or stops sending data, is started again on the channel it was on.
- Some phones lock up the adapter's USB 3 link every few seconds to minutes. The adapter frees itself, and if
  it keeps happening the app says that another phone or adapter may do better.
- A frame torn by a stream restart is not shown. The one before it stays up.
- When the phone cannot keep up, the app says so and names the settings to turn off.
- If the picture goes away while the menu is closed (the adapter unplugged, for one), the menu opens to say
  why, and closes again when the picture is back.
- After 30 seconds in the background the app lets go of the adapter, and opens it again when you return.

## What changed since kepyur

When fpvDROID forked from it, kepyur's Android app scanned the band, listed the transmitters it found and showed
the one you tapped. fpvDROID builds on its radio and decoder and adds:

- Handles errors in the USB 3 link on some phones, which lock-ups the feed. Certain phones go from unusable to working.
- Improved decoder: vastly improved image quality and sync lock through low signal strength.
- A faster decoder: its filters were reworked for about 40 % less processor time on a phone, helps slower
  phones keep up with decoding.
- A menu with five tabs in place of the transmitter list. Any band and channel can be picked directly, scanning
  is optional, and the channel is remembered.
- The Spectrum tab, with its recognition of DJI, HDZero and Wi-Fi.
- Picture processing: comb filter, clean OSD edges, smooth lines and sparkle repair.
- Two-antenna diversity and tuning beside the channel
- View settings: aspect ratio, scaling, size, picture brightness and goggles mode.
- Lower latency ([measured](#measured-latency)). The picture is drawn in strips as it is decoded, straight to
  the screen where the phone allows, with the screen held at its fastest refresh rate.

## Credits

- [kepyur](https://github.com/gilankpam/kepyur) is the app fpvDROID is forked from.
- [Play RX](https://github.com/scerstt/PlayRX), a Windows receiver built on kepyur, is the source of the picture
  processing, antenna diversity, tuning beside the channel and sturdier USB streaming.
- [wifi-sdr](https://github.com/snokvist/wifi-sdr) and [devourer](https://github.com/OpenIPC/devourer) make the
  adapter deliver raw radio samples.
- [FPVDesktopWiFiReceiver](https://github.com/Pwn3dP4nd4/FPVDesktopWiFiReceiver) is the source of the digital
  signal detector and of the idea for the Wi-Fi marks.

## Building and details

[docs/development.md](docs/development.md) has the build steps, the phone and desktop test tools, and the exact
rules behind the scan and the Spectrum tab.

License: GPL-2.0 (wifi-sdr is GPL-2.0-only).
