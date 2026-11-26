// kp_wifi_tests: the Spectrum tab's Wi-Fi networks (dsp/wifi.h) from sweep steps made here as raw IQ: noise, Wi-Fi
// networks as OFDM (802.11a/ac subcarriers, on when their schedule says), analog video as FM and DJI-like links as
// OFDM that never stops, swept on the radio's own plan (Sweep) and taken the way Radio::handle_sweep takes them
// (ctest: "wifi"). Plain checks, as in radio_tests.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "spectrum.h"
#include "sweep.h"
#include "wifi.h"

using namespace kp;

static int g_checks = 0, g_failed = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    g_checks++;                                                                   \
    if (!(c)) {                                                                   \
      g_failed++;                                                                 \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c);  \
    }                                                                             \
  } while (0)

// ---------------------------------------------------------------------------------------- the air
constexpr double kFs = 40e6, kEdge = 16.5e6, kSc = 0.3125;  // radio.cpp's kEdge; OFDM's subcarrier spacing (MHz)

// The same pseudo-random numbers for the same seed and time: a schedule says the same thing to every step.
static uint64_t mix(uint64_t x) {
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}
static double unit(uint64_t seed, uint64_t n) { return static_cast<double>(mix(seed * 0x100000001b3ull + n) >> 11) / 9007199254740992.0; }

using Schedule = std::function<bool(double t_us)>;
static Schedule always() { return [](double) { return true; }; }
// An AP with no traffic: a beacon of 0.4 ms every 102.4 ms.
static Schedule beacons(double phase_us) {
  return [phase_us](double t) { return std::fmod(std::fmod(t - phase_us, 102400.0) + 102400.0, 102400.0) < 400; };
}
// A network in use: on in `duty` of its 0.5 ms frames at random, besides its beacons.
static Schedule traffic(double duty, uint64_t seed) {
  return [duty, seed](double t) { return unit(seed, static_cast<uint64_t>(t / 500)) < duty || beacons(seed % 1000 * 100.0)(t); };
}
// Short bursts (0.1 ms) at random, `duty` of the time.
static Schedule crackle(double duty, uint64_t seed) {
  return [duty, seed](double t) { return unit(seed, static_cast<uint64_t>(t / 100)) < duty; };
}

// OFDM: subcarriers kSc apart from lo to hi (MHz, on the band's kSc grid), db_lo..db_hi over the noise (a slope, for a
// strong signal's shoulders), on in the symbols (4 us) its schedule says.
struct Ofdm {
  double lo, hi;
  float db_lo, db_hi;
  Schedule on;
  double null_mhz = 0;  // a Wi-Fi channel's DC subcarrier, left empty
};
// A Wi-Fi network of 20, 40 or 80 MHz around mhz: 52, 114 or 242 subcarriers (+-26, +-58, +-122).
static Ofdm wifi(double mhz, int width, float db, Schedule on) {
  const int half = width == 80 ? 122 : width == 40 ? 58 : 26;
  return Ofdm{mhz - half * kSc, mhz + half * kSc, db, db, std::move(on), mhz};
}

// Analog video, FM: sync tip at -dev/2, white at +dev/2, around mhz. The picture is the hardest kind for this: a bright
// top, a ramp across the middle and a dark bottom, so that the bins of the levels it dwells on come and go.
struct Fm {
  double mhz;
  float db;  // over the noise, as if spread over 9 MHz
  double dev_mhz = 12;
};

struct Air {
  std::vector<Ofdm> ofdm;
  std::vector<Fm> fm;
};

static double ire_at(double t_us) {
  const double tf = std::fmod(t_us, 16683.0);  // an NTSC field
  const int line = static_cast<int>(tf / 63.556);
  const double tl = tf - line * 63.556;
  if (line < 9) return tl < 27 ? -40 : 0;  // the vertical sync
  if (tl < 4.7) return -40;
  if (tl < 10.9 || line < 21) return 0;
  if (line < 100) return 90;
  if (line < 180) return 7.5 + 92.5 * (tl - 10.9) / 52.6;
  return 15;
}

// What a sweep step hears: n samples at 40 MS/s centred on centre_hz from t0_us on, noise of power 1.
static std::vector<cf> capture(const Air &air, double centre_hz, double t0_us, size_t n, uint64_t seed) {
  std::vector<cf> x(n);
  {
    uint64_t s = mix(seed);
    for (auto &v : x) {  // Gaussian enough: the FFT's bins are, whatever the samples are
      float a[2];
      for (float &c : a) {
        float u = 0;
        for (int k = 0; k < 4; k++) {
          s = s * 6364136223846793005ull + 1442695040888963407ull;
          u += static_cast<float>(s >> 40) / 16777216.f;
        }
        c = (u - 2.f) * 1.2247f;  // variance 4/12 x 1.5 = 0.5 a component
      }
      v = cf(a[0], a[1]);
    }
  }
  const double c_mhz = centre_hz / 1e6;
  for (const auto &o : air.ofdm) {
    // the subcarriers in the stream (+-19.5 MHz), as bins of a 128-point IFFT at 40 MS/s
    std::vector<int> bin;
    std::vector<float> amp;
    const int k0 = static_cast<int>(std::lround(o.lo / kSc)), k1 = static_cast<int>(std::lround(o.hi / kSc));
    for (int k = k0; k <= k1; k++) {
      const double f = k * kSc;
      if (o.null_mhz > 0 && std::fabs(f - o.null_mhz) < kSc / 2) continue;
      const long m = std::lround((f - c_mhz) / kSc);
      if (std::abs(m) > 62) continue;
      const float db = k1 > k0 ? o.db_lo + (o.db_hi - o.db_lo) * static_cast<float>(k - k0) / static_cast<float>(k1 - k0) : o.db_lo;
      bin.push_back(static_cast<int>((m + 128) % 128));
      amp.push_back(std::sqrt(std::pow(10.f, db / 10) / 128.f));
    }
    if (bin.empty()) continue;
    cf sym[128];
    const uint64_t os = mix(static_cast<uint64_t>(o.lo * 1000) + seed);
    // symbols on the absolute 4 us grid: 128 samples and a cyclic prefix of 32
    const double first = std::ceil(t0_us / 4.0);
    for (double j = first;; j++) {
      const size_t at = static_cast<size_t>(std::lround((j * 4.0 - t0_us) * 40));
      if (at >= n) break;
      if (!o.on(j * 4.0)) continue;
      std::fill(sym, sym + 128, cf(0, 0));
      for (size_t i = 0; i < bin.size(); i++) {
        const uint64_t r = mix(os + static_cast<uint64_t>(j) * 977 + i);
        sym[bin[i]] = cf(r & 1 ? amp[i] : -amp[i], r & 2 ? amp[i] : -amp[i]) * 0.7071f;
      }
      for (auto &v : sym) v = std::conj(v);
      fft_inplace(sym, 128);  // the inverse, by conjugates
      for (size_t i = 0; i < 160 && at + i < n; i++) {
        const cf v = std::conj(sym[(i + 96) % 128]);
        x[at + i] = cf(x[at + i].real() + v.real(), x[at + i].imag() + v.imag());
      }
    }
  }
  for (const auto &f : air.fm) {
    const double off = (f.mhz - c_mhz) * 1e6;
    if (std::fabs(off) > 20e6 + f.dev_mhz * 0.5e6) continue;
    const float a = std::sqrt(std::pow(10.f, f.db / 10) * 9e6f / 40e6f);
    double ph = 0;
    for (size_t i = 0; i < n; i++) {
      const double fi = off + f.dev_mhz * 1e6 * ((ire_at(t0_us + i / 40.0) + 40) / 140.0 - 0.5);
      ph += 2 * M_PI * fi / kFs;
      if (ph > M_PI) ph -= 2 * M_PI;
      if (ph < -M_PI) ph += 2 * M_PI;
      if (std::fabs(fi) > 19.5e6) continue;  // outside what the chip's filter passes
      x[i] = cf(x[i].real() + a * static_cast<float>(std::cos(ph)), x[i].imag() + a * static_cast<float>(std::sin(ph)));
    }
  }
  return x;
}

// ---------------------------------------------------------------------------------------- the sweep
// What Radio::handle_sweep does with each step: its spectrum into the band picture, its bursts (and what the band
// picture has on the air) into the Wi-Fi watch. A step every 180 ms, each capture 12 ms (radio.cpp's kPsdSpan).
struct Sim {
  Air air;
  bool carriers = true;  // false: the band picture's carriers are not handed on (no pilots)
  BandMap band{Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin};
  WifiWatch wifi;
  Sweep sweep;
  int64_t t_ms = 1000;
  uint64_t seed = 1;

  explicit Sim(double lo_mhz = 5700, double hi_mhz = 5850) { sweep.start(lo_mhz * 1e6, hi_mhz * 1e6); }
  void step(double c) {
    const auto x = capture(air, c, static_cast<double>(t_ms) * 1000, static_cast<size_t>(0.012 * kFs), seed++);
    Psd psd(1024);
    psd.add(x.data(), x.size());
    band.add(psd.db(), kFs, c, kEdge, t_ms);
    const BurstStep b = measure_bursts(x.data(), x.size(), kFs, c, Sweep::kStep / 2, kEdge);
    double first = 0;
    const auto db = band.get(sweep.lo(), sweep.hi(), &first);
    wifi.add(b, sweep.lo(), sweep.hi(), carriers ? find_carriers(db, first, band.bin_hz()) : std::vector<Carrier>(), t_ms);
    t_ms += 180;
  }
  void passes(int n) {
    for (int i = 0; i < n; i++, sweep.pass_done())
      for (double c : sweep.pass()) step(c);
  }
  std::vector<WifiNet> nets() const { return wifi.networks(sweep.lo(), sweep.hi(), t_ms); }
  std::vector<Carrier> on_air() const {
    double first = 0;
    const auto db = band.get(sweep.lo(), sweep.hi(), &first);
    return find_carriers(db, first, band.bin_hz());
  }
};

static std::string labels(const std::vector<WifiNet> &nets) {
  std::string s;
  for (const auto &n : nets) s += (s.empty() ? "" : ", ") + n.label() + (n.busy ? " (busy)" : "");
  return s.empty() ? "none" : s;
}

static bool is(const std::vector<WifiNet> &nets, std::vector<std::pair<int, int>> want) {
  if (nets.size() != want.size()) return false;
  for (size_t i = 0; i < want.size(); i++)
    if (nets[i].first != want[i].first || nets[i].last != want[i].second) return false;
  return true;
}

// ---------------------------------------------------------------------------------------- the tests
// One capture: noise reads 0 dB and lights nothing; a beacon lights every bin of its channel at once, and the rest of
// the time none; an OFDM link that never stops is never off; nothing is measured within 0.75 MHz of the centre.
static void a_capture_tells_bursts_from_steady_signals() {
  const size_t n = static_cast<size_t>(0.012 * kFs);
  BurstStep s = measure_bursts(capture({}, 5770e6, 0, n, 7).data(), n, kFs, 5770e6, 15e6, kEdge);
  double sum = 0;
  float peak = 0;
  bool dc = false;
  for (const auto &b : s.bins) {
    sum += b.duty;
    peak = std::max(peak, 10 * std::log10(b.peak));
    dc = dc || std::fabs(BurstMap::kLo + b.index * BurstMap::kBin - 5770e6) < 0.75e6;
  }
  CHECK(s.bins.size() == 183);  // +-15 MHz in 156.25 kHz, less the DC notch (9 bins)
  CHECK(!dc);
  CHECK(sum / s.bins.size() < 0.001 && peak < 6);
  bool none = true;
  for (const auto &c : s.channels) none = none && c.full == 0;
  CHECK(none && !s.channels.empty());

  Air beacon{{wifi(5765, 20, 18, beacons(4000))}, {}};  // channel 153, a beacon 4 ms into the capture
  s = measure_bursts(capture(beacon, 5770e6, 0, n, 7).data(), n, kFs, 5770e6, 15e6, kEdge);
  const BurstStep::Channel *c153 = nullptr;
  for (const auto &c : s.channels)
    if (c.ch == 153) c153 = &c;
  CHECK(c153 && c153->full == kMinSlices && c153->empty > 100 && std::fabs(c153->level_db - 18) < 3);

  Air link{{Ofdm{5755, 5775, 18, 18, always()}}, {}};  // a 20 MHz link on channel 153, never off
  s = measure_bursts(capture(link, 5770e6, 0, n, 7).data(), n, kFs, 5770e6, 15e6, kEdge);
  bool burst = false;
  for (const auto &c : s.channels) burst = burst || (c.full >= kMinSlices && c.empty >= kMinSlices);
  CHECK(!burst);
}

// A network in use on channel 157: found on the first pass, and in use within three.
static void a_network_in_use_is_found() {
  Sim sim;
  sim.air.ofdm.push_back(wifi(5785, 20, 25, traffic(0.3, 11)));
  sim.passes(1);
  CHECK(is(sim.nets(), {{157, 157}}));
  sim.passes(2);
  const auto nets = sim.nets();
  std::fprintf(stderr, "  in use on 157: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{157, 157}}));
  CHECK(nets.size() == 1 && nets[0].busy && nets[0].lo_mhz == 5775 && nets[0].hi_mhz == 5795 && nets[0].label() == "Wi-Fi 157");
}

// An AP with no traffic on channel 149: only its beacons, caught now and then. Once caught it is kept (not busy) while
// it goes on being missed, until kWifiHoldMs after the last time.
static void a_quiet_network_is_remembered_then_forgotten() {
  Sim sim;
  sim.air.ofdm.push_back(wifi(5745, 20, 20, beacons(31700)));
  int found_at = -1;
  for (int pass = 0; pass < 60 && found_at < 0; pass++) {
    sim.passes(1);
    if (!sim.nets().empty()) found_at = pass;
  }
  const auto nets = sim.nets();
  std::fprintf(stderr, "  quiet on 149: %s after %d passes\n", labels(nets).c_str(), found_at + 1);
  CHECK(is(nets, {{149, 149}}) && !nets[0].busy);
  // a pass or two misses it (most do): still there
  sim.passes(3);
  CHECK(is(sim.nets(), {{149, 149}}));
  // gone from the air: kept until kWifiHoldMs after it was last seen, which was within the last 4 passes
  sim.air.ofdm.clear();
  sim.passes(2);
  CHECK(is(sim.nets(), {{149, 149}}));
  sim.t_ms += kWifiHoldMs;
  sim.passes(1);
  CHECK(sim.nets().empty());
}

// A network of 40 MHz on 132+136 and one of 80 on 149-161, both in use: each read as one, over its whole block.
static void wide_networks_read_as_one() {
  Sim sim(5635, 5850);
  sim.air.ofdm.push_back(wifi(5670, 40, 22, traffic(0.25, 21)));
  sim.air.ofdm.push_back(wifi(5775, 80, 22, traffic(0.25, 22)));
  sim.passes(3);
  const auto nets = sim.nets();
  std::fprintf(stderr, "  40 MHz on 132+136, 80 MHz on 149-161: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{132, 136}, {149, 161}}));
  CHECK(nets.size() == 2 && nets[0].lo_mhz == 5650 && nets[0].hi_mhz == 5690 && nets[1].lo_mhz == 5735 &&
        nets[1].hi_mhz == 5815 && nets[1].label() == "Wi-Fi 149\xe2\x80\x93" "161" && nets[0].busy && nets[1].busy);
}

// Analog video (strong, middling and weak, its picture dwelling on levels that come and go), a 20 MHz digital link
// that never stops right on a Wi-Fi channel and a 40 MHz one: all on the air, none of them Wi-Fi.
static void analog_video_and_steady_digital_links_are_not_wifi() {
  Sim sim(5700, 5910);
  sim.air.fm = {{5769, 40}, {5745, 25}, {5806, 12, 8}};    // R4 (inside channel 153), A7 (on 149's centre), R5
  sim.air.ofdm.push_back(Ofdm{5835, 5855, 22, 22, always()});  // a 20 MHz link on channel 169's centre
  sim.air.ofdm.push_back(Ofdm{5860, 5900, 18, 18, always()});  // 40 MHz over channels 173 and 177
  sim.passes(6);
  const auto nets = sim.nets();
  const auto on_air = sim.on_air();
  std::fprintf(stderr, "  analog video and steady links: %zu on the air, Wi-Fi: %s\n", on_air.size(), labels(nets).c_str());
  CHECK(on_air.size() >= 4);
  CHECK(nets.empty());
}

// An analog VTX at 37 dB very close by, as measured upstream: besides its own channel it lights up in bursts from 10
// to 45 MHz either side. None of that is Wi-Fi; without its carrier to go by, the bursts would read as networks. A real
// network inside that reach, as strong as to matter beside it, is still seen.
static void a_strong_vtxs_sidebands_are_not_wifi() {
  Air air;
  air.fm.push_back({5658, 44});  // R1: 37 dB in the band picture
  air.ofdm.push_back(Ofdm{5613, 5648, 15, 22, crackle(0.04, 31)});
  air.ofdm.push_back(Ofdm{5668, 5703, 22, 15, crackle(0.04, 32)});
  Sim sim(5635, 5760);
  sim.air = air;
  sim.passes(6);
  auto nets = sim.nets();
  std::fprintf(stderr, "  strong VTX on R1: Wi-Fi %s\n", labels(nets).c_str());
  CHECK(nets.empty());

  Sim blind(5635, 5760);
  blind.air = air;
  blind.carriers = false;
  blind.passes(6);
  std::fprintf(stderr, "  ... with no carriers to go by: %s\n", labels(blind.nets()).c_str());
  CHECK(!blind.nets().empty());

  Sim real(5635, 5760);
  real.air = air;
  real.air.ofdm.push_back(wifi(5680, 20, 30, traffic(0.15, 33)));  // channel 136, 22 MHz from the VTX
  real.passes(6);
  nets = real.nets();
  std::fprintf(stderr, "  ... and a network on 136 at 30 dB: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{136, 136}}));
}

// A network of 80 MHz at 33 dB with its shoulders, as measured upstream (24 dB at its edge falling to 13 over the next
// 30 MHz, bursting with it): the network alone.
static void a_strong_networks_shoulders_are_not_wifi() {
  Sim sim(5700, 5880);
  const Schedule on = traffic(0.25, 41);
  sim.air.ofdm.push_back(wifi(5775, 80, 33, on));
  sim.air.ofdm.push_back(Ofdm{5815.3125, 5845, 24, 13, on});
  sim.air.ofdm.push_back(Ofdm{5705, 5734.6875, 13, 24, on});
  sim.passes(3);
  const auto nets = sim.nets();
  std::fprintf(stderr, "  80 MHz at 33 dB with its shoulders: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{149, 161}}));
}

// The networks as the tab names them: channels side by side within an 80 MHz block are one network, two whole 80 MHz
// blocks within a 160 MHz one too, but nothing across the grid's gaps.
static void networks_are_grouped_by_the_wifi_grid() {
  const auto w = [](std::vector<int> chs) {
    std::vector<Wifi> v;
    for (int c : chs) v.push_back(Wifi{c, wifi_mhz(c), 20, c == 104});
    return v;
  };
  auto g = group_wifi(w({100, 104, 108, 112, 116, 120, 124, 128}));
  CHECK(g.size() == 1 && g[0].first == 100 && g[0].last == 128 && g[0].lo_mhz == 5490 && g[0].hi_mhz == 5650 && g[0].busy);
  g = group_wifi(w({112, 116}));  // two networks: no 40 MHz channel spans them
  CHECK(g.size() == 2 && g[0].label() == "Wi-Fi 112" && g[1].label() == "Wi-Fi 116" && !g[0].busy);
  g = group_wifi(w({140, 144, 149}));  // 144 and 149 are not 4 apart
  CHECK(g.size() == 2 && g[0].first == 140 && g[0].last == 144 && g[1].first == 149);
  g = group_wifi(w({100, 104, 108, 112, 116, 120}));  // 80 + 40: not a 160
  CHECK(g.size() == 2 && g[0].last == 112 && g[1].first == 116 && g[1].last == 120);
}

// The band picture as the app reads it carries the networks in an array of their own.
static void the_band_picture_carries_the_networks() {
  WifiNet a{149, 149, 5735, 5755, false}, b{100, 112, 5490, 5570, true};
  const std::string j = band_json(std::vector<float>(3, 0.f), 5735e6, 0.5e6, 0, 33e6, 2, {a, b});
  CHECK(j.find("\"carriers\":[]") != std::string::npos);
  CHECK(j.find("\"wifi\":[{\"lo_mhz\":5735.0,\"hi_mhz\":5755.0,\"label\":\"Wi-Fi 149\",\"busy\":false},"
               "{\"lo_mhz\":5490.0,\"hi_mhz\":5570.0,\"label\":\"Wi-Fi 100\xe2\x80\x93" "112\",\"busy\":true}]}") !=
        std::string::npos);
  CHECK(band_json(std::vector<float>(3, 0.f), 5735e6, 0.5e6, 0, 33e6, 2).find("\"wifi\":[]") != std::string::npos);
}

int main() {
  a_capture_tells_bursts_from_steady_signals();
  networks_are_grouped_by_the_wifi_grid();
  the_band_picture_carries_the_networks();
  a_network_in_use_is_found();
  a_quiet_network_is_remembered_then_forgotten();
  wide_networks_read_as_one();
  analog_video_and_steady_digital_links_are_not_wifi();
  a_strong_vtxs_sidebands_are_not_wifi();
  a_strong_networks_shoulders_are_not_wifi();
  std::fprintf(stderr, "kp_wifi_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
