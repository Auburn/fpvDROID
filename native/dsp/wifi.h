// fpvDROID: 5 GHz Wi-Fi networks on the Spectrum tab. The averaged band picture (BandMap) hides them: a network
// that only sends its beacons is on the air well under 1 % of the time, and one in use comes and goes. What gives
// them away is how the band behaves over time, so every sweep step's capture is also cut into 0.1 ms slices
// (measure_bursts), find_wifi reads the networks out of it, and a WifiMemory keeps them for a few minutes: a quiet
// network is only caught now and then.
//
// Ported from FPVDesktopWiFiReceiver (github.com/Pwn3dP4nd4/FPVDesktopWiFiReceiver, GPL-2.0): the survey's
// statistics (src/radio/survey.cpp), find_wifi, WifiMemory and group_wifi (src/channel_advisor.cpp, its PRs 6 and
// 14; the reasoning is in its docs/wiki/Channel-analyzer.md). Its thresholds were measured on the adapter with the
// same chip at the same rate, slice and bin, which is why those are its own here too.
//
// One thing is done differently. Upstream tells a burst by each bin on its own (its strongest slice well over its
// mean), and leaves out whatever a known pilot covers: its pilots pass a sync check. The sweep here has no such list
// (what the band picture shows on the air may be a VTX, a DJI link or a busy network), and analog video comes and
// goes in single bins too, wherever the picture dwells on one level for a while. So a channel is taken here for
// Wi-Fi only when its bins light up together and evenly: OFDM switches its 52 subcarriers on and off at once, across
// 16.6 MHz, each with the same power. FM video can light most of a channel within 0.1 ms (a line that sweeps every
// level), but its levels then stand some 30 dB over the bins its leaps cross, it is on one frequency at a time within
// each 6.4 us FFT, and a strong VTX is never off.
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "spectrum.h"

namespace kp {

// A 20 MHz channel of the Wi-Fi grid: 5000 + 5 ch MHz, its OFDM (52 subcarriers) 16.6 MHz wide.
constexpr double wifi_mhz(int ch) { return 5000 + 5.0 * ch; }
constexpr double kWifiOccupiedMhz = 16.6;

// What one capture (a sweep step's 12 ms at 40 MS/s) says, against its own noise floor (the chip's AGC picks a gain of
// its own at every retune, so only the floor makes two steps comparable).
struct BurstStep {
  // Each 156.25 kHz in [-reach, +reach) of the centre, for the BurstMap.
  struct Bin {
    long index;   // on the BurstMap's grid
    float peak;   // the strongest 0.1 ms slice, over the floor (linear)
    float duty;   // the share of slices more than 6 dB over the floor ("on")
  };
  // Each channel of the Wi-Fi grid nearly all inside the capture (kChannelPresent of its bins), judged on its bins
  // that are not on all the time (a spur's, a pilot's beside it), when those are at least kJudgedShare of it. Not one
  // that holds kSteadyRunMhz on all the time in one piece: it is under a transmitter that never stops. Analog video is
  // such a one: every line has its sync pulse, so what lies from the sync tip to the blanking level (2 to 5 MHz) is on
  // in every slice.
  struct Channel {
    int ch;
    int full;        // slices with kFullShare of those bins on at once, evenly (their 10th percentile within kFlatDb
                     // of their 90th: room for the ripple of a multipath channel indoors) and in each 6.4 us frame,
                     // counted up to kMinSlices (no more are needed)
    int empty;       // slices with no more than kEmptyShare of them on
    float level_db;  // their strongest slices (median), over the floor
  };
  double centre_hz = 0;
  std::vector<Bin> bins;
  std::vector<Channel> channels;
};
constexpr double kChannelPresent = 0.85, kJudgedShare = 0.5;
constexpr float kSteadyDuty = 0.9f;   // a bin on in 90 % of the slices is on all the time
constexpr double kSteadyRunMhz = 1.5;  // more than a strong spur lights through the window's sidelobes
constexpr double kFullShare = 0.9, kEmptyShare = 0.1;
constexpr float kFlatDb = 15;
constexpr int kMinSlices = 2;  // full slices and empty ones that make a channel a network (find_wifi)
// One capture of n samples at fs, centred on centre_hz: slices of 0.1 ms (16 FFTs of 256, Hann), and none of it
// within 0.75 MHz of the centre (the chip's DC notch). The channels are judged on what lies within span_hz of the
// centre; the bins handed on are those in [-reach_hz, +reach_hz), so that, with the sweep's steps 2 x reach_hz apart,
// a pass measures every frequency once. Empty under 8 slices.
BurstStep measure_bursts(const cf *x, size_t n, double fs, double centre_hz, double reach_hz, double span_hz);

// One bin of the burst picture (dB over the floor of the step that measured it).
struct BurstBin {
  double mhz = 0;
  float peak_db = 0;  // the strongest slice of the last passes (it fades)
  float busy = 0;     // the share of the time it has been on, over the last kBusyPasses passes
};

// The band (Sweep::kBandLo..kBandHi) in 156.25 kHz bins, each step of a sweep laid on it: what each bin did over the
// last passes. As in upstream's survey grid, the strongest slice fades kPeakFadeDb over kPasses passes. The share of
// time on is a sliding average over kBusyPasses passes, started from nothing, so that the beacons of one capture
// early on do not read as a network in use, while one on a quarter of the time does within three passes.
// Not thread-safe.
class BurstMap {
public:
  static constexpr double kLo = 5300e6, kHi = 6000e6, kBin = 40e6 / 256;
  static constexpr int kPasses = 20;        // ~40 s of a sweep over 300 MHz
  static constexpr float kPeakFadeDb = 20;  // over kPasses passes: 1 dB a pass
  static constexpr int kBusyPasses = 10;

  BurstMap();
  void add(const BurstStep &s, int64_t now_ms);
  // Bins in lo..hi last measured before older_than_ms are forgotten.
  void forget(double lo_hz, double hi_hz, int64_t older_than_ms);
  // The bins measured in lo..hi, in order.
  std::vector<BurstBin> get(double lo_hz, double hi_hz) const;

private:
  struct Acc {
    float peak = 0, busy = 0;
    int64_t t = 0;  // when last measured (0: never)
  };
  std::vector<Acc> _acc;
};

// A Wi-Fi network seen on one 20 MHz channel.
struct Wifi {
  int ch = 0;            // 149
  double mhz = 0;        // its centre (5745)
  float peak_db = 0;     // its strongest slices (median across it), over the floor
  bool traffic = false;  // on more than its beacons would make it: in use, not only announcing itself
};
// The networks one sweep step shows (step: its capture, measure_bursts), against the burst picture over the span
// swept (bins), the networks seen in the last few seconds (recent) and what the band picture has on the air
// (carriers: find_carriers). A channel counts when, in this capture, kMinSlices slices had its bins on together and
// kMinSlices had them off together, and its strongest slices reach kWifiOverNoise over the band's typical strongest
// slice (the 25th percentile of the picture's: nearly all of it is noise), never under kWifiPeakMin. Upstream
// measured the noise's strongest slices on the adapter (54 sweeps, 5480-5940 MHz): median 5.9 dB, 99 % under 9.3 dB,
// whole stretches at 7-8 dB; an AP on channel 100, 24-27 dB. Which channels are looked for: every region's (the US
// plan, a superset of the EU's, where 144 and 169-177 are not used), since a router only uses what its country allows
// and what is seen is what is there; 68-96 are no Wi-Fi channels anywhere (5340-5480 MHz), and 36-64 are mostly under
// the band the tab sweeps.
//
// What does not count:
//   - a channel in a strong pilot's sidebands (upstream's PR 14). Measured on the adapter, an analog VTX at 35-39 dB
//     very close by lights up in bursts 10 to 45 MHz either side of it, which read as Wi-Fi on four channels. Its reach
//     beyond its width grows kSidebandMhzPerDb a dB over kSidebandFromDb (6 to 40 MHz: at 37 dB, 50 MHz from its
//     centre), and only what is at least kSidebandBelowDb weaker than it is taken for its sidebands: a real network
//     next to a strong pilot, or any next to a weak one, still shows. The pilots are the carriers not on a network.
//   - a weak channel with a much stronger one close by (kWifiShoulderMhz, kWifiShoulderDb): its shoulders. Measured
//     upstream, a block of 80 MHz at 33 dB falls from 24 to 13 dB over the next 30 MHz.
// Traffic: its bins on kWifiTrafficShare of the time or more (BurstBin::busy). Beacons take ~0.4 ms every 102.4 ms:
// under 2 % even for an AP that announces four networks.
constexpr float kWifiPeakMin = 6, kWifiOverNoise = 7;
constexpr float kSidebandFromDb = 10, kSidebandBelowDb = 10;
constexpr double kSidebandMhzPerDb = 1.5, kSidebandMinMhz = 6, kSidebandMaxMhz = 40;
constexpr double kWifiShoulderMhz = 40;
constexpr float kWifiShoulderDb = 10;
constexpr float kWifiTrafficShare = 0.05f;
std::vector<Wifi> find_wifi(const BurstStep &step, const std::vector<BurstBin> &bins, const std::vector<Wifi> &recent,
                            const std::vector<Carrier> &carriers);

// The networks seen, each kept kWifiHoldMs from the last time: a step of the sweep measures 12 ms, an AP with no
// traffic sends a beacon every ~102 ms, and a channel is in one or two steps a pass, so a quiet one is caught in about
// one pass out of eight. A network once seen in use stays so while it is kept. One seen within kWifiRecentMs is
// recent (find_wifi weighs a new one against those).
constexpr int64_t kWifiHoldMs = 300000, kWifiRecentMs = 10000;
class WifiMemory {
public:
  void clear() { _nets.clear(); }
  // What a step showed at t_ms. A network kept that is now the shoulder of one of these is forgotten: a step lower in
  // the band may have seen the shoulder before the step of the network itself.
  void update(const std::vector<Wifi> &now, int64_t t_ms);
  std::vector<Wifi> known(int64_t t_ms, int64_t within_ms = kWifiHoldMs) const;  // by channel

private:
  struct Entry {
    Wifi w;
    int64_t last_ms = 0;
  };
  std::vector<Entry> _nets;
};

// One network as the Spectrum tab shows it: channels first..last of the grid next to one another, the way a 40, 80
// or 160 MHz network lights them, from the lower edge of the first to the upper edge of the last.
struct WifiNet {
  int first = 0, last = 0;
  double lo_mhz = 0, hi_mhz = 0;
  bool busy = false;  // one of them has been seen in use
  std::string label() const;  // "Wi-Fi 149", "Wi-Fi 100-112" (an en dash)
};
// Channels next to one another (4 apart) in one 80 MHz block of the grid (36-48, 52-64, 100-112, 116-128, 132-144,
// 149-161, 165-177) are taken for one network, and two such blocks lit whole within a 160 MHz one (36-64, 100-128,
// 149-177) too. Two networks of 20 MHz side by side read as one of 40: the tab only says that the span is Wi-Fi.
std::vector<WifiNet> group_wifi(const std::vector<Wifi> &wifi);

// The Spectrum tab's Wi-Fi, step by step of the sweep (Radio::handle_sweep): the burst picture and the networks
// remembered. Not thread-safe.
class WifiWatch {
public:
  // A sweep over lo..hi starts: what it finds older than older_than_ms there is forgotten (as BandMap::forget).
  void forget(double lo_hz, double hi_hz, int64_t older_than_ms) { _map.forget(lo_hz, hi_hz, older_than_ms); }
  // A step of a sweep over lo..hi (measure_bursts); carriers: what the band picture has on the air over lo..hi now.
  void add(const BurstStep &s, double lo_hz, double hi_hz, const std::vector<Carrier> &carriers, int64_t now_ms);
  // The networks remembered that reach into lo..hi.
  std::vector<WifiNet> networks(double lo_hz, double hi_hz, int64_t now_ms) const;

private:
  BurstMap _map;
  WifiMemory _memory;
};

}  // namespace kp
