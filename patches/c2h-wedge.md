# The C2H wedge and the 40 MS/s deadlines: lab record

What was tried and measured behind fpvDROID's additions to `wifi-sdr-0001-playrx-stream.patch` (its comments
marked `fpvDROID:`). [README.md](README.md) says what the patch carries, and the code comments give each rule
with its reason. The numbers behind them are here, in the order the work went.

- A **bank** is 64 KiB of samples (205 us of them at 40 MS/s, 410 us at 20 MS/s), shipped as four data packets
  (its **parts**) and a trailer.
- A **new start** restarts the stream on the open adapter and leaves a hole of ~40 to 55 ms. An **open** of the
  adapter takes ~3 s.
- The **wedge** is wifi-sdr's FINDINGS §4: on some USB 3 hosts the chip's C2H engine stops taking requests.
- The phones: a Pixel 8 Pro (Android 17), an LG G7+ (Android 10, kernel 4.9, the one host with U1 on) and a
  Pixel 7a (Android 17).
- No video transmitter was on in any of these runs: they measure the stream, not the picture.

## Where it stands

The patch as built on 2026-11-24:

| | Pixel 8 Pro | LG G7+ | Pixel 7a |
|---|---|---|---|
| Wedges, each freed in place | ~130 a minute | ~1 a minute | ~50 a minute |
| 20 MS/s watch | 185 s: one new start (the first wedge), one bank dropped | 120 s: two new starts from the first wedge, then none | 150 s: one new start, 10 banks dropped |
| 40 MS/s watch | 155 s: two new starts; fell back to 20 MS/s after 150 s | 2 x 150 s: no new start, no fall back | fell back to 20 MS/s 16 to 25 s in |
| 40 MS/s stream test | 4 min: one new start | 150 s: no new start | 146 s: one new start, ~400 banks dropped |

## The pulse that frees a wedge

What frees a wedged engine is switching the RX DMA enables in REG_CR (0x100 bits 1 and 3) off and on. It was
found on the Pixel 8 Pro by giving each of 93 wedges one of nine candidates in turn: this one freed 13 of 13;
a new start alone, a USB port reset and the others freed 0 of 13.

The onboard program (`src/fw/stream8822b.S`, `SEND`) gives that pulse itself when a request has been pending
for `StreamCfg::unwedge_us`, and counts its pulses in the bank's trailer.

The packet it frees comes with 8192 bytes too many, as a 16384-byte and an 8184-byte transfer, and the stream
puts it back together.

- **20 MS/s:** 97 % of the wedges are at a bank's first packet, the bank still arrives in time and nothing is
  lost. Of 334 wedges in 148 s, 322 went so and 12 were at a later packet, which cost a new start of ~40 ms
  each. None cost an open.
- **40 MS/s:** a bank has no time to spare for it, so a wedge there cost a new start. There are far fewer of
  them: about 10 in 190 s, against about 100 a minute at 20 MS/s.
- In three minutes at each rate the stream delivered 99.7 % and 99.6 % of its banks.

## Holding the pulse back (tried, not in)

The pulse also fires when the host stops reading for more than `unwedge_us` (96 times in one 10 ms stall). Two
ways of holding it back were tried against forced stalls of ~0.5 to 1.2 ms at 20 MS/s, about 46 stalls a
setting. The trial's sources are not in the tree.

| Setting | Stalls that ended the run |
|---|---|
| The pulse as it is | 65 % |
| At most one pulse a request | 71 % |
| No pulse while RXDMA_REQ (REG 0x284 BIT19) reads 1 | 48 %, but 11 new starts in its 88 s check run, against 4 in 193 s without it |
| No pulse at all | 76 %, most of them as a wedge |

So neither is in. With no pulse, the chip wedged about 15 times as often as usual while the host was not
collecting its transfers.

## What sets the wedge off (tried, not in)

This was tried from both ends, at 20 MS/s in settings of 30 s. These sources are not in the tree either.

When the host resubmits its transfers matters:

| How the host resubmits | Wedges a minute |
|---|---|
| As the stream thread does it (it sleeps between packets) | 100 to 136 |
| Each packet collected at once, so the bank's last transfer is resubmitted a full idle gap before the next bank | 284 |
| The same with three more resubmits spread through the gap | 88, no better than as it is |
| The last two held back until the next bank starts | 1301 |

The idle gap matters too. With the chip pausing between packets so that a bank's five start 60 us apart, there
were 16 wedges a minute; 88 us apart (no gap over ~55 us), 2. But those two settings had 10 and 9 new starts
in their 30 s, against 1 to 5 as it is and none in the two settings that collect at once. The cause was packets
that came as a 16384-byte transfer with no 8184-byte one after it, which the stream did not then put back
together. So spreading the packets cost more than it saved.

Spreading was tried again once the stream could put those packets back together (next section), 50 s each:
2 wedges and one bank dropped with it, 82 wedges and one bank dropped without, no new start either way. It
leaves a bank ~85 us to spare instead of ~215 and gains nothing more, so it is not in. On the LG at 20 MS/s it
made no difference: 0 to 1 wedges in 3 minutes either way.

## The shapes a freed packet comes in

Ten of the packets that were not put back together were recorded (that transfer and the four after it). All
ten are one shape: K bytes, then a whole 16376-byte packet starting at K, as a 16384-byte transfer and one of
K - 8, with the stream in step after it.

- **In seven,** the K bytes are the packet's own start over again (K is 6144, 13312 or 14336, where the wedged
  packet has 8192). The pulse came when the packet was already well on its way, and the packet at K is whole.
- **In three,** K is 15360, the bytes before it are the packet before, short of its last 1016, and the chip had
  given no pulse. That packet is lost; the one at K is whole. A part 3 so cut comes with the trailer straight
  after it in one transfer: 15532 bytes with the trailer of that day, 15552 with today's.

The stream now puts the first kind back together at any K, and drops only the bank of a packet cut short
(`StreamStats::cut`). Before, each cost a new start.

With that, 55 s at 20 MS/s: 127 wedges freed, three packets put back together from 13312, 5120 and 6144 bytes
in, one bank dropped for a cut packet, and no new start. Two runs of 50 s before it had none either; as it was,
there were 1 to 5 in 30 s.

## 40 MS/s: strays and the burst rule

At 40 MS/s the packets run back to back and a bank has ~14 us to spare: it ends 171 us after due, of the 185
then allowed. So a packet put back together there cost its bank and the one or two after it.

Eleven new starts at that rate were recorded:

- **Eight** were the host pausing for 1 to 4 ms with the onboard program pulsing all the while. Bare 24-byte
  descriptors and pieces of packets come out.
- **Three** were a 134 to 138-byte packet that is none of the stream's but has the C2H bit in byte 11.

So a transfer of none of the stream's lengths is now skipped whatever that bit says, and so is a 16384-byte
transfer that the rest of a packet does not follow. More than 256 in one bank still end the run. The bank they
were in comes short and is dropped.

With that:

- **20 MS/s, 3 minutes:** 364 wedges freed, four packets put back together at other cut points, one bank
  dropped, no new start. In about 15 minutes at 20 MS/s since the wider re-join there was one new start, when
  the host stopped reading for 4.4 ms.
- **40 MS/s:** 5 new starts and 25 banks dropped in a 2 minute watch; 7 and 92 in a 4 minute stream test.
  Every new start left at 40 MS/s was the burst rule (more than 4 banks dropped within 64).

One thing changed for the app. Lost blocks at 40 MS/s now come in ones to fours where a new start came before,
and that can reach the 10 in 5 s at which a 40 MS/s watch goes on at 20 MS/s
(`native/radio/fallback_policy.h`). It did in one run of three, 20 s in.

## The pulse waits for the first wedge

The pulse is not given from the start, because of the LG. At 20 MS/s the LG runs for minutes without a wedge,
and then wedges at every start for a while.

- **The pulse from the start:** three runs of four had new starts: 1, 5, and one that failed start after start
  for its last minute, through six opens of the adapter. Two runs without the pulse had one new start between
  them. The pulse also falls on packets that are merely slow, and on that phone those come out in pieces for
  banks on end.
- **No pulse at all:** a wedge left it wedged through 18 opens, about 70 s.
- **The pulse once a run had ended wedged:** it was running again 0.3 s and 1.8 s after the two wedges it had
  (2 and 10 failed starts).

So that last one is the rule. On the Pixel 8 Pro the first wedge comes within a second and costs the one new
start.

## Lateness judged per part

At 40 MS/s the LG's banks all end ~194 us after due. A bank used to be late, and dropped, once it ended more
than period - guard after due (185 us): when the ADC comes back to its first byte. Every bank was dropped,
every run ended within ten banks, and the app opened the adapter 13 times in 2.5 minutes.

But the ADC overwrites a part only when it gets back to that part, and the chip takes a packet faster than the
ADC writes one (16376 bytes in 38 to 49 us, against 51 at 40 MS/s). So what matters is when each part starts.
The trailer now carries when each of the four data packets was taken (`TrailerStamps`). A bank is late if a
part started within 20 us of the ADC's return to it, or if the bank ended so late that the sentinel written
after it would land on the next revolution (`adc_back5` in `Stream8822b::run`).

With that, on the LG at 40 MS/s:

- a stream test of 127 s had one new start and 4 banks dropped;
- watches of 157 s and 187 s had no new start, and two banks dropped in all.

## Skipping a bank that starts too late

Before, after the host stopped reading for a few ms, the onboard program shipped bank after bank late until it
caught up (at 40 MS/s ~34 us a bank on the Pixel 8 Pro), and the run ended on the burst rule. Now the program
does not ship a bank that starts too late. It skips it, the trailer's seq jumps and counts the skipped banks,
and the next bank goes on time. The stream counts them as dropped banks but not as a burst
(`StreamCfg::skip_late`; `WSDR_SKIP=0` turns it off).

### The first limit did not save the run

The first limit was part 0's deadline, 165 us at 40 MS/s. On the Pixel 8 Pro, of 17 skips in 8 minutes at
40 MS/s, 16 were followed by a new start within 0 to 4 banks. The pause that makes the chip late also breaks
packets (the onboard pulse falls on them while the host is not reading), 3 or 4 banks come short and are
dropped, and the next marginal bank is the fifth drop within 64.

Starting the burst count afresh after a skip, which is in the patch, did not change that by itself: 10 of 12
skips were still followed by a new start. The spells of broken packets last 0.1 to 0.4 s, with nearly every
bank in them coming short. So the count fills again after the skip, and the new starts come in clusters of
2 to 5.

On the LG that limit fired once a run, at the start.

### What a broken bank is

Sixteen spells were recorded raw on the Pixel 8 Pro, every transfer whole. The commonest broken piece is a
part the chip sends short (42 of the 127 odd transfers):

- it is cut at 4, 8 or 12 KiB plus 9 to 26 bytes, or runs 11 to 138 bytes past 16 KiB;
- its first 32 bytes are zero where the descriptor should be;
- by the chip's own clock it took only as long as its length, so the rest was never sent and the bank cannot be
  put back together.

Such parts nearly all went out more than ~185 us after due, once the ADC was back in their bank: 75 % of the
parts sent that late were in broken banks, against 4 % of those sent in time.

A spell typically starts with a packet freed by the pulse, which makes its bank ~70 to 100 us late. With
~35 us a bank to catch up at 40 MS/s, the banks after it keep sending their last parts after 185 us and lose
them. On the LG, parts go out past 185 us whole, so lateness alone does not cut a part there.

### The limit in the patch

So the limit became the time left for a whole bank to go out before the ADC is back at its first byte: the
period less the guard less 165 us. That is about a bank's send time at USB 3 (168 to 181 us for 90 % of the
Pixel 8 Pro's banks, median 173). It comes to 20 us at 40 MS/s and ~225 us at 20 MS/s.

Tried on the Pixel 8 Pro's 40 MS/s stream test, two minutes per arm, each limit twice, interleaved:

| Limit | New starts a minute | Banks dropped a minute | Banks delivered |
|---|---|---|---|
| 20 us | 3.6 | 14 | 97.6 % |
| 50 us | 7.1 | 39 | 95.9 % |
| 165 us (part 0's deadline) | 7.4 | 42 | 96.8 % |

Both 20 us arms had the fewest new starts of the six. But the arms varied a lot (4.4 and 10.4 a minute at
165 us), and the `LIBUSB_TRANSFER_NO_DEVICE` errors (below) were in every arm: 26 of the 71 run ends, with the
phone at 35 to 37 C.

### The built patch on the three phones

**Pixel 8 Pro,** with no USB errors at the time:

- 40 MS/s stream test, 4 minutes: one new start (the first wedge) and 8 skipped banks.
- 20 MS/s watch, 185 s: one new start, 397 wedges freed, one bank dropped, no skip.
- 40 MS/s watch, 155 s: two new starts and 8 skipped banks. It fell back to 20 MS/s after 150 s, on a spell of
  broken packets that lost 32 blocks in 5 s (25 s in on the build before).

**LG G7+.** The new limit could have cost more there: its banks take ~194 us to go out at 40 MS/s, so a pause
of more than 20 us costs a skipped bank where before the bank came late but whole. It did not:

- 40 MS/s watches of 150 s, alternating: the new limit skipped 3 and 2 banks and dropped nothing else; the old
  one dropped 12 and 0. No new start and no fall back to 20 MS/s in any of the four.
- 40 MS/s stream test, 150 s: no new start, and one skip, at the start.
- 20 MS/s watch, 120 s: the first wedge 2 s in (two new starts, the freed packet's pieces taken for strays),
  then none.

**Pixel 7a.** It wedges too, ~50 times a minute, each freed in place.

- 40 MS/s watches: they lost 10 blocks in 5 s as soon as the grace ended, and fell back to 20 MS/s 16 to 25 s
  in, with either skip limit.
- 40 MS/s stream test, 146 s: one new start (the first wedge), but ~400 banks dropped, nearly all short after a
  pulse.
- 20 MS/s watch, 150 s: one new start, 130 wedges freed, 10 banks dropped.

## USB errors and run-to-run variation on the Pixel 8 Pro

The Pixel 8 Pro at 20 MS/s for 247 s, once the pulse waited for the first wedge: the first wedge 3 s in cost
its one new start, then 489 wedges were freed and 5 banks dropped. There was one more new start, for a transfer
that came back `LIBUSB_TRANSFER_NO_DEVICE` with the adapter still there.

Those errors come and go. There were none in that day's earlier runs. After the adapter was moved from the LG
there were 12 in 13 minutes, none in the next 7, then 24 in 12 minutes, on that build and on the one before
alike. A spell of broken packets often follows one.

At 40 MS/s the phone's new starts went from 1 to 16 a minute between 2-minute spells, one build's as much as
another's, so a short A/B cannot tell builds apart there. And on one evening every 40 MS/s watch fell back to
20 MS/s within 15 to 33 s, where one had held for 150 s a few hours before.

## What else the patch changed

- **The trailer** is 24 bytes longer than upstream's: the pulse count (4), when each data packet was taken
  (16) and the banks skipped (4).
- **A wedge that gets past the onboard program** is recognised as before: a packet does not come and the 0x294
  request is still pending. The run pulses the RX DMA on its way out, and the Receiver gives it one new start.
  Only a wedge that comes back before a bank ends the stream, with `RxStatus::gave_up` saying which way it
  ended. `RxStatus::unwedged` counts both kinds of freed wedge.
- **A restart after a fault** pauses 10 ms between starts instead of 100, and a start drains for 10 ms instead
  of 30. The hole a restart leaves in the stream goes from ~200 ms to ~55 ms (measured on the Pixel 8 Pro,
  which restarted every few seconds at 40 MS/s), with no more failed starts.
- **Whether the link is SuperSpeed** comes from the descriptors when the kernel cannot say (`Session::open`).
  libusb asks with USBDEVFS_GET_SPEED, which older kernels lack (the LG on Android 10). Taken as USB 2, a
  SuperSpeed link got 4096-byte EP0 writes, which the chip stalls: LIBUSB_ERROR_PIPE at every start. A bulk
  endpoint's 1024-byte packets now say USB 3.
- **The USB 3 PHY value (tried, no effect, taken out again).** Realtek's drivers write the 8822B's USB 3 PHY
  register 0x01 to 0xA841 on cut D at bring-up (halmac `usb3_phy_param_8822b`, rtw88 `usb3_param_8822b`), and
  devourer does not. It goes through 0xFF0D/0xFF0E (data) and 0xFF0C (address; BIT(7) writes, BIT(6) reads,
  with the value then in 0xFF0C's bytes 1 and 2). Written on the Pixel 8 Pro, it made no difference to the
  wedge.

## The app's fallback from 40 to 20 MS/s

The rules are in `native/radio/fallback_policy.h`. What they were measured against:

- **Pixel 8 Pro:** 20 MS/s is the better rate there, now that the wedge is freed in place: one new start in
  ~15 minutes at 20 MS/s, against two to four a minute at 40 MS/s. Before, 20 MS/s wedged within ~0.2 s there
  and was far the worse.
- **Lost blocks, not restarts:** what cost a restart at 40 MS/s there now mostly costs one to four lost blocks.
  So it is the lost blocks that take such a watch to 20 MS/s (in one run of three, 20 s in), where the restarts
  seldom did.
- **LG G7+, with the old whole-bank deadline:** every 40 MS/s run ended within ten banks. The receiver gave up
  inside the 10 s grace, the app opened the adapter again, and that cleared the fallback: 13 opens in
  2.5 minutes. With the fallback held over reopens (`FallbackPolicy::receiver_stopped`, `StreamFailing`), it
  settled at 20 MS/s 3.6 s after the first failed start, with one reopen.
