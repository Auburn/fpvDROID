// fpvDROID: 5 GHz Wi-Fi networks on the Spectrum tab. The averaged band picture (BandMap) hides them: a network
// that only sends its beacons is on the air well under 1 % of the time, and one in use comes and goes. What gives
// them away is how the band behaves over time, so every sweep step's grid (StepGrid: its 12 ms in slices of 0.1 ms) is
// also read for bursts (measure_bursts), and a WifiWatch finds the networks in them and keeps them for a few minutes: a
// quiet network is only caught now and then.
//
// Ported from FPVDesktopWiFiReceiver (github.com/Pwn3dP4nd4/FPVDesktopWiFiReceiver, GPL-2.0): the survey's
// statistics (src/radio/survey.cpp), and how networks are found, kept and grouped (src/channel_advisor.cpp, its PRs 6
// and 14; the reasoning is in its docs/wiki/Channel-analyzer.md). Its thresholds were measured on the adapter with the
// same chip at the same rate, slice and bin, which is why those are its own here too.
//
// One thing is done differently. Upstream tells a burst by each bin on its own (its strongest slice well over its
// mean), and leaves out whatever a known pilot covers: its pilots pass a sync check. The sweep here has no such list
// (what the band picture shows on the air may be a VTX, a DJI link or a busy network), and analog video comes and
// goes in single bins too, wherever the picture dwells on one level for a while. So a channel is taken here for
// Wi-Fi only when its bins light up together and evenly: OFDM switches its 52 subcarriers on and off at once, across
// 16.6 MHz, each with the same power. FM video can light most of a channel within 0.1 ms (a line that sweeps every
// level), but its levels then stand some 30 dB over the bins its leaps cross, it is on one frequency at a time within
// each 6.4 us frame, and a strong VTX is never off. wifi.cpp has the rules and their thresholds.
#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "spectrum.h"
#include "step_grid.h"

namespace kp {

// A 20 MHz channel of the Wi-Fi grid: 5000 + 5 ch MHz.
constexpr double wifi_mhz(int ch) { return 5000 + 5.0 * ch; }

// What one sweep step's grid says about bursts, against the grid's own noise floor (the chip's AGC picks a gain of its
// own at every retune, so only the floor makes two steps comparable).
struct BurstStep {
  // A bin of the grid: what it did over the step's slices.
  struct Bin {
    double hz;   // its frequency
    float peak;  // the strongest slice, over the floor (linear)
    float duty;  // the share of slices more than 6 dB over the floor ("on")
  };
  // A channel of the Wi-Fi grid nearly all inside the step's span, judged slice by slice on its bins that are not on
  // all the time.
  struct Channel {
    int ch;
    int full;        // slices with those bins on together, evenly and in each 6.4 us frame (counted up to kMinSlices)
    int empty;       // slices with hardly any of them on
    float level_db;  // its strongest slices (the median across those bins), over the floor
  };
  std::vector<Bin> bins;  // from the lowest frequency up
  std::vector<Channel> channels;
};
// Full slices, and empty ones, that make a channel a network.
constexpr int kMinSlices = 2;
// One step's grid (StepGrid::build: the channels are judged within its span), the hardware centre on centre_hz. The
// bins handed on are those in [-reach_hz, +reach_hz) of the centre, so that, with the sweep's steps 2 x reach_hz apart,
// a pass measures every frequency once. Empty under 8 slices, or with no floor (a stream of zeros).
BurstStep measure_bursts(const StepGrid &g, double centre_hz, double reach_hz);

// A Wi-Fi network seen on one 20 MHz channel.
struct Wifi {
  int ch = 0;            // 149
  double mhz = 0;        // its centre (5745)
  float peak_db = 0;     // its strongest slices (median across it), over the floor
  bool traffic = false;  // on more than its beacons would make it: in use, not only announcing itself
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
// The network a carrier of the band picture (find_carriers) is, or nullptr: one whose span holds its centre, the carrier
// no wider than it (a busy network's bursts, averaged, read as a carrier as wide as the network).
const WifiNet *wifi_for(const std::vector<WifiNet> &nets, const Carrier &c);

// How long a network is kept after it was last seen: a step measures 12 ms, an AP with no traffic sends a beacon every
// ~102 ms, and a channel is in one or two steps a pass, so a quiet one is caught in about one pass out of eight.
constexpr int64_t kWifiHoldMs = 300000;

// The Spectrum tab's Wi-Fi, step by step of the sweep (BandPicture::add): what the bins have done over the last passes
// (the burst picture) and the networks seen, each kept kWifiHoldMs. Not thread-safe.
class WifiWatch {
public:
  // The burst picture over lo..hi in bins of bin_hz, as the band picture's (Sweep::kBandLo, kBandHi, kBandBin).
  WifiWatch(double lo_hz, double hi_hz, double bin_hz);
  ~WifiWatch();
  // A sweep over lo..hi starts: what the burst picture has there older than older_than_ms is forgotten (as
  // BandMap::forget). The networks are kept their own time.
  void forget(double lo_hz, double hi_hz, int64_t older_than_ms);
  // A step of a sweep over lo..hi (measure_bursts); carriers: what the band picture has on the air over lo..hi now.
  void add(const BurstStep &s, double lo_hz, double hi_hz, const std::vector<Carrier> &carriers, int64_t now_ms);
  // The networks kept that reach into lo..hi, as of the last step: once the sweep stops, they stay as last seen.
  std::vector<WifiNet> networks(double lo_hz, double hi_hz) const;

private:
  struct State;  // wifi.cpp: the burst picture and the networks seen
  std::unique_ptr<State> _s;
};

}  // namespace kp
