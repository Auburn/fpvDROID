// kp_wifi_tests: the Spectrum tab's Wi-Fi networks (dsp/wifi.h) from sweep steps made here as raw IQ: noise, Wi-Fi
// networks as OFDM on their schedules (tools/digital_tx.h), analog video as FM and digital links as OFDM that never
// stops, swept on the radio's own plan (Sweep), each step taken as Radio::handle_sweep takes it (StepAnalysers::step,
// then BandPicture::add: sweep.h) (ctest: "wifi"). Plain checks, as in radio_tests.cpp.
//
//   kp_wifi_tests          every case; exits 1 if any fails
//   kp_wifi_tests --time   what a sweep step's bursts cost: measured on its grid, against the whole step
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "digital_tx.h"
#include "spectrum.h"
#include "step_grid.h"
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
constexpr double kFs = 40e6, kEdge = 16.5e6;                       // radio.cpp's kEdge: the span a step uses
constexpr size_t kStepSamples = static_cast<size_t>(0.012 * kFs);  // radio.cpp's kPsdSpan
constexpr double kNoiseSigma = 20;                                 // the chip's noise (12-bit counts)

// Something on the air as OFDM (tools/digital_tx.h), centred on mhz: its offset and clock are set for each capture.
struct Ofdm {
  double mhz;
  tx::Ofdm tx;
};
// OFDM in Wi-Fi's subcarriers and symbols from lo to hi (MHz), db_lo over the noise at its lower edge to db_hi at its
// upper (a slope: a strong signal's shoulders), sending when `on` says.
static Ofdm block(double lo, double hi, double db_lo, double db_hi, tx::Schedule on) {
  tx::Ofdm o;
  o.width_hz = (hi - lo) * 1e6;
  o.snr_db = (db_lo + db_hi) / 2;
  o.tilt_db = db_hi - db_lo;
  o.wifi = true;
  o.on = std::move(on);
  o.seed = tx::mix(static_cast<uint64_t>(lo * 1000));
  return Ofdm{(lo + hi) / 2, o};
}
// A Wi-Fi network of 20, 40 or 80 MHz around mhz: 52, 114 or 242 subcarriers (+-26, +-58, +-122) 312.5 kHz apart.
static Ofdm wifi(double mhz, int width, double db, tx::Schedule on) {
  const double half = (width == 80 ? 122 : width == 40 ? 58 : 26) * 0.3125;
  return block(mhz - half, mhz + half, db, db, std::move(on));
}
// A digital link that never stops, width_mhz wide around mhz.
static Ofdm link(double mhz, double width_mhz, double db) {
  tx::Ofdm o;
  o.width_hz = width_mhz * 1e6;
  o.snr_db = db;
  return Ofdm{mhz, o};
}

// Analog video, FM: sync tip at -dev/2, white at +dev/2, around mhz.
struct Fm {
  enum Picture {
    // The hardest kind for this: a bright top, a ramp across the middle and a dark bottom, so that the bins of the
    // levels it dwells on come and go.
    kBusy,
    // A still picture, a flat grey of `grey` IRE: the spectrum's twin peaks, at the blanking level and at the grey.
    kStill,
  };
  double mhz;
  double db;  // over the noise, as if spread over 9 MHz
  double dev_mhz = 12;
  Picture picture = kBusy;
  double grey = 50;
};

// The receiver's front end, third-order (y = x + a3 |x|^2 x), on a pilot p and a network w: its product p^2 w* at
// 2 fp - fw, which bursts when the network does and is as flat as its OFDM (the other one away from them, w^2 p* at
// 2 fw - fp, is the network's spectrum doubled in width: no flat block). a3 is given by the product's level: db over
// (or, negative, under) the network's.
struct Intermod {
  Fm p;
  Ofdm w;
  double db;
};

struct Air {
  std::vector<Ofdm> ofdm;
  std::vector<Fm> fm;
  std::vector<Intermod> intermod;
};

// The level of an NTSC picture at t (s on the transmitter's clock).
static double ire_at(const Fm &f, double t) {
  const double tf = std::fmod(t * 1e6, 16683.0);  // an NTSC field, in us
  const int line = static_cast<int>(tf / 63.556);
  const double tl = tf - line * 63.556;
  if (line < 9) return tl < 27 ? -40 : 0;  // the vertical sync
  if (tl < 4.7) return -40;
  if (tl < 10.9 || line < 21) return 0;
  if (f.picture == Fm::kStill) return f.grey;
  if (line < 100) return 90;
  if (line < 180) return 7.5 + 92.5 * (tl - 10.9) / 52.6;
  return 15;
}

// f's amplitude, as its db says against the noise.
static float fm_amplitude(const Fm &f) {
  const double noise_density = 2 * kNoiseSigma * kNoiseSigma / kFs;
  return static_cast<float>(std::sqrt(noise_density * 9e6 * std::pow(10.0, f.db / 10)));
}

// f, offset_hz from the samples' centre, from t0_s on: x.size() samples, added to x. What would go past the chip's
// filter (19.5 MHz) is left out.
static void add_fm(std::vector<cf> &x, const Fm &f, double offset_hz, double t0_s) {
  const float a = fm_amplitude(f);
  double ph = 0;
  for (size_t i = 0; i < x.size(); i++) {
    const double fi = offset_hz + f.dev_mhz * 1e6 * ((ire_at(f, t0_s + i / kFs) + 40) / 140.0 - 0.5);
    ph += 2 * M_PI * fi / kFs;
    if (ph > M_PI) ph -= 2 * M_PI;
    if (ph < -M_PI) ph += 2 * M_PI;
    if (std::fabs(fi) > 19.5e6) continue;
    x[i] += a * cf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
  }
}

// im's product in a capture centred on centre_hz, made from the pilot and the network each centred on its own (so that
// both are whole) and moved to where it lands. Left out where its bulk (the network's width, and the pilot's levels
// doubled in frequency about the middle of the picture: the sync tip's rare excursions past that are the chip's
// filter's) would reach past the edge of the samples, rather than folded over.
static void add_intermod(std::vector<cf> &x, const Intermod &im, double centre_hz, double t0_s, std::mt19937 &rng) {
  const double at = (2 * im.p.mhz - im.w.mhz) * 1e6 - centre_hz;
  if (std::fabs(at) + im.w.tx.width_hz / 2 + im.p.dev_mhz * 0.5e6 > 20e6) return;
  std::vector<cf> p(x.size()), w(x.size());
  add_fm(p, im.p, 0, t0_s);
  tx::Ofdm o = im.w.tx;
  o.t0_s = t0_s;
  tx::add_ofdm(w, o, kNoiseSigma, rng);
  const float ap = fm_amplitude(im.p), a3 = static_cast<float>(std::pow(10.0, im.db / 20)) / (ap * ap);
  for (size_t i = 0; i < x.size(); i++)
    x[i] += a3 * p[i] * p[i] * std::conj(w[i]) * std::polar(1.f, static_cast<float>(2 * M_PI * std::fmod(at * i / kFs, 1.0)));
}

// What a sweep step hears: kStepSamples at 40 MS/s centred on centre_hz from t0_s on, noise of kNoiseSigma.
static std::vector<cf> capture(const Air &air, double centre_hz, double t0_s, uint64_t seed) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  std::normal_distribution<float> nd(0, static_cast<float>(kNoiseSigma));
  std::vector<cf> x(kStepSamples);
  for (auto &v : x) v = cf(nd(rng), nd(rng));
  for (const auto &o : air.ofdm) {
    tx::Ofdm s = o.tx;
    s.offset_hz = o.mhz * 1e6 - centre_hz;
    s.t0_s = t0_s;
    if (std::fabs(s.offset_hz) - s.width_hz / 2 < 18e6) tx::add_ofdm(x, s, kNoiseSigma, rng);
  }
  for (const auto &f : air.fm) {
    const double off = f.mhz * 1e6 - centre_hz;
    if (std::fabs(off) <= 20e6 + f.dev_mhz * 0.5e6) add_fm(x, f, off, t0_s);
  }
  for (const auto &im : air.intermod) add_intermod(x, im, centre_hz, t0_s, rng);
  return x;
}

static StepGrid grid_of(const std::vector<cf> &x) {
  StepGrid g;
  g.build(x.data(), x.size(), kFs, kEdge);
  return g;
}

// ---------------------------------------------------------------------------------------- the sweep
// The radio's sweep on the Spectrum tab, each step taken as Radio::handle_sweep takes it: measured
// (StepAnalysers::step), then into the band picture (BandPicture::add), which weighs its bursts against the carriers
// of the span. A step every 180 ms, each capture 12 ms. blind: the bursts go into a WifiWatch of its own with no
// carriers to go by (no pilots), to show what those carriers keep out.
struct Sim {
  Air air;
  bool blind = false;
  StepAnalysers analysers;
  BandPicture picture;
  WifiWatch alone{Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin};  // blind's
  Sweep sweep;
  int64_t t_ms = 1000;
  uint64_t seed = 1;

  explicit Sim(double lo_mhz = 5700, double hi_mhz = 5850) {
    sweep.start(lo_mhz * 1e6, hi_mhz * 1e6);
    picture.sweep_started(sweep.lo(), sweep.hi(), t_ms);
    analysers.sweep_started();
  }
  void step(double c) {
    const StepFindings f = analysers.step(capture(air, c, t_ms / 1e3, seed++), kFs, c, kEdge);
    if (blind) alone.add(f.bursts, sweep.lo(), sweep.hi(), {}, t_ms);
    else picture.add(f, t_ms);
    t_ms += 180;
  }
  void passes(int n) {
    for (int i = 0; i < n; i++, sweep.pass_done())
      for (double c : sweep.pass()) step(c);
  }
  std::vector<WifiNet> nets() const { return (blind ? alone : picture.wifi()).networks(sweep.lo(), sweep.hi()); }
  std::vector<Carrier> on_air() const {
    double first = 0;
    const auto db = picture.band().get(sweep.lo(), sweep.hi(), &first);
    return find_carriers(db, first, picture.band().bin_hz());
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

static const BurstStep::Channel *channel(const BurstStep &s, int ch) {
  for (const auto &c : s.channels)
    if (c.ch == ch) return &c;
  return nullptr;
}

// ---------------------------------------------------------------------------------------- the tests
// One step's grid: noise reads 0 dB and lights nothing; a beacon lights every bin of its channel at once, and the rest
// of the time none; an OFDM link that never stops is never off; nothing is measured within 0.75 MHz of the centre,
// and each bin is handed on at its own frequency, within reach of the centre.
static void a_step_tells_bursts_from_steady_signals() {
  BurstStep s = measure_bursts(grid_of(capture({}, 5770e6, 0, 7)), 5770e6, 15e6);
  double sum = 0;
  float peak = 0;
  bool dc = false, reach = true;
  for (const auto &b : s.bins) {
    sum += b.duty;
    peak = std::max(peak, 10 * std::log10(b.peak));
    dc = dc || std::fabs(b.hz - 5770e6) < 0.75e6;
    reach = reach && b.hz >= 5755e6 && b.hz < 5785e6;
  }
  CHECK(s.bins.size() == 183);  // +-15 MHz in 156.25 kHz, less the DC notch (9 bins)
  CHECK(!dc && reach);
  CHECK(sum / s.bins.size() < 0.001 && peak < 6);
  bool none = true;
  for (const auto &c : s.channels) none = none && c.full == 0;
  CHECK(none && !s.channels.empty());

  Air beacon{{wifi(5765, 20, 18, tx::beacons(4e-3))}, {}, {}};  // channel 153, a beacon 4 ms into the capture
  s = measure_bursts(grid_of(capture(beacon, 5770e6, 0, 7)), 5770e6, 15e6);
  const BurstStep::Channel *c153 = channel(s, 153);
  CHECK(c153 && c153->full == kMinSlices && c153->empty > 100 && std::fabs(c153->level_db - 18) < 3);

  Air steady{{link(5765, 20, 18)}, {}, {}};  // a 20 MHz link on channel 153, never off
  s = measure_bursts(grid_of(capture(steady, 5770e6, 0, 7)), 5770e6, 15e6);
  bool burst = false;
  for (const auto &c : s.channels) burst = burst || (c.full >= kMinSlices && c.empty >= kMinSlices);
  CHECK(!burst);
}

// A network in use on channel 157: found on the first pass, and in use within three; as the app reads it, a signal of
// kind "wifi", and the carrier its bursts make tagged with it.
static void a_network_in_use_is_found() {
  Sim sim;
  sim.air.ofdm.push_back(wifi(5785, 20, 25, tx::traffic(0.3, 11)));
  sim.passes(1);
  CHECK(is(sim.nets(), {{157, 157}}));
  sim.passes(2);
  const auto nets = sim.nets();
  std::fprintf(stderr, "  in use on 157: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{157, 157}}));
  CHECK(nets.size() == 1 && nets[0].busy && nets[0].lo_mhz == 5775 && nets[0].hi_mhz == 5795 && nets[0].label() == "Wi-Fi 157");
  const std::string j = sim.picture.json(sim.sweep.lo(), sim.sweep.hi(), 0, 2 * kEdge, 3);
  CHECK(j.find("\"signals\":[{\"lo_mhz\":5775.0,\"hi_mhz\":5795.0,\"kind\":\"wifi\",\"label\":\"Wi-Fi 157\","
               "\"active\":true}]") != std::string::npos);
  CHECK(j.find(",\"kind\":\"wifi\"}") != std::string::npos);
}

// An AP with no traffic on channel 149: only its beacons, caught now and then. Once caught it is kept (not busy) while
// it goes on being missed, until kWifiHoldMs after the last time; and with the sweep stopped, as last seen.
static void a_quiet_network_is_remembered_then_forgotten() {
  Sim sim;
  sim.air.ofdm.push_back(wifi(5745, 20, 20, tx::beacons(31.7e-3)));
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
  CHECK(is(sim.nets(), {{149, 149}}));  // no step since: as last seen
  sim.passes(1);
  CHECK(sim.nets().empty());
}

// A network of 40 MHz on 132+136 and one of 80 on 149-161, both in use: each read as one, over its whole block.
static void wide_networks_read_as_one() {
  Sim sim(5635, 5850);
  sim.air.ofdm.push_back(wifi(5670, 40, 22, tx::traffic(0.25, 21)));
  sim.air.ofdm.push_back(wifi(5775, 80, 22, tx::traffic(0.25, 22)));
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
  sim.air.fm = {{5769, 40}, {5745, 25}, {5806, 12, 8}};  // R4 (inside channel 153), A7 (on 149's centre), R5
  sim.air.ofdm.push_back(link(5845, 20, 22));            // a 20 MHz link on channel 169's centre
  sim.air.ofdm.push_back(link(5880, 40, 18));            // 40 MHz over channels 173 and 177
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
  air.ofdm.push_back(block(5613, 5648, 15, 22, tx::crackle(0.04, 31)));
  air.ofdm.push_back(block(5668, 5703, 22, 15, tx::crackle(0.04, 32)));
  Sim sim(5635, 5760);
  sim.air = air;
  sim.passes(6);
  auto nets = sim.nets();
  std::fprintf(stderr, "  strong VTX on R1: Wi-Fi %s\n", labels(nets).c_str());
  CHECK(nets.empty());

  Sim blind(5635, 5760);
  blind.air = air;
  blind.blind = true;
  blind.passes(6);
  std::fprintf(stderr, "  ... with no carriers to go by: %s\n", labels(blind.nets()).c_str());
  CHECK(!blind.nets().empty());

  Sim real(5635, 5760);
  real.air = air;
  real.air.ofdm.push_back(wifi(5680, 20, 30, tx::traffic(0.15, 33)));  // channel 136, 22 MHz from the VTX
  real.passes(6);
  nets = real.nets();
  std::fprintf(stderr, "  ... and a network on 136 at 30 dB: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{136, 136}}));
}

// A network of 80 MHz at 33 dB with its shoulders, as measured upstream (24 dB at its edge falling to 13 over the next
// 30 MHz, bursting with it): the network alone.
static void a_strong_networks_shoulders_are_not_wifi() {
  Sim sim(5700, 5880);
  const tx::Schedule on = tx::traffic(0.25, 41);
  sim.air.ofdm.push_back(wifi(5775, 80, 33, on));
  sim.air.ofdm.push_back(block(5815, 5845, 24, 13, on));
  sim.air.ofdm.push_back(block(5705, 5735, 13, 24, on));
  sim.passes(3);
  const auto nets = sim.nets();
  std::fprintf(stderr, "  80 MHz at 33 dB with its shoulders: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{149, 161}}));
}

// What the phone showed beside a weak VTX on R3 (5732.7 MHz, 23 dB, a still picture) with a busy network on 149: a
// network on 144, idle, at 17 dB: 6 dB under the VTX, so not its sidebands, and under the 10 dB that makes it 149's
// shoulder. The VTX alone does not light 144: its skirts reach only the top 1.3 MHz of the channel, at most a tenth of
// its bins at once. What lights all of it at once, flat and in bursts, is the front end's third-order product of the
// two, 2 x 5732.7 - 5745 = 5720.4 MHz: the network on 149 alone. A real network on 144, as strong as the VTX, is still
// seen beside them.
static void a_front_end_product_is_not_a_network() {
  const Fm r3{5732.7, 34, 8, Fm::kStill, 50};
  const Ofdm w149 = wifi(5745, 20, 25, tx::traffic(0.04, 51));
  Sim alone(5635, 5955);
  alone.air.fm = {r3};
  alone.passes(4);
  std::fprintf(stderr, "  a VTX on R3: %s\n", labels(alone.nets()).c_str());
  CHECK(alone.nets().empty());

  Sim sim(5635, 5955);
  sim.air.fm = {r3};
  sim.air.ofdm = {w149};
  sim.air.intermod = {{r3, w149, -8}};
  sim.passes(8);
  auto nets = sim.nets();
  std::fprintf(stderr, "  ... a network on 149, and their product on 144: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{149, 149}}));

  Sim real(5635, 5955);
  real.air = sim.air;
  real.air.ofdm.push_back(wifi(5720, 20, 26, tx::traffic(0.1, 52)));
  real.passes(8);
  nets = real.nets();
  std::fprintf(stderr, "  ... and a network on 144 at 26 dB: %s\n", labels(nets).c_str());
  CHECK(is(nets, {{144, 144}, {149, 149}}));
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

// A carrier of the band picture is a network's when the network's span holds its centre and it is no wider.
static void a_carrier_on_a_network_is_that_network() {
  const std::vector<WifiNet> nets = {{149, 149, 5735, 5755, false}, {100, 112, 5490, 5570, true}};
  CHECK(wifi_for(nets, Carrier{5745.2, 16.5, 18}) == &nets[0]);
  CHECK(wifi_for(nets, Carrier{5530, 70, 15}) == &nets[1]);
  CHECK(wifi_for(nets, Carrier{5732.7, 6.5, 23}) == nullptr);  // beside 149's span
  CHECK(wifi_for(nets, Carrier{5745, 30, 25}) == nullptr);     // run together with something else
}

// ---------------------------------------------------------------------------------------- the cost
template <class F>
static double ms_per(int runs, F f) {
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < runs; i++) f();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / runs;
}

static void time_it() {
  constexpr int kRuns = 50;
  struct Case {
    const char *name;
    Air air;
  };
  const Case cases[] = {
      {"noise only", {}},
      {"analog FM (R4)", {{}, {{5769, 30}}, {}}},
      {"Wi-Fi in use (153)", {{wifi(5765, 20, 25, tx::traffic(0.3, 11))}, {}, {}}},
      {"Wi-Fi 80 MHz in use", {{wifi(5775, 80, 22, tx::traffic(0.5, 22))}, {}, {}}},
  };
  std::printf("a sweep step: %zu samples (12 ms at 40 MS/s), ms\n", kStepSamples);
  std::printf("%-24s %8s %8s | %8s\n", "", "grid", "bursts", "step");
  for (const auto &c : cases) {
    const auto x = capture(c.air, 5770e6, 0.1, 9);
    StepGrid g;
    const double grid = ms_per(kRuns, [&] { g.build(x.data(), x.size(), kFs, kEdge); });
    BurstStep b;
    const double bursts = ms_per(kRuns, [&] { b = measure_bursts(g, 5770e6, Sweep::kStep / 2); });
    StepAnalysers a;
    const double step = ms_per(kRuns, [&] { a.step(x, kFs, 5770e6, kEdge); });
    std::printf("%-24s %8.2f %8.2f | %8.2f\n", c.name, grid, bursts, step);
  }
}

int main(int argc, char **argv) {
  if (argc > 1 && std::strcmp(argv[1], "--time") == 0) {
    time_it();
    return 0;
  }
  a_step_tells_bursts_from_steady_signals();
  networks_are_grouped_by_the_wifi_grid();
  a_carrier_on_a_network_is_that_network();
  a_network_in_use_is_found();
  a_quiet_network_is_remembered_then_forgotten();
  wide_networks_read_as_one();
  analog_video_and_steady_digital_links_are_not_wifi();
  a_strong_vtxs_sidebands_are_not_wifi();
  a_strong_networks_shoulders_are_not_wifi();
  a_front_end_product_is_not_a_network();
  std::fprintf(stderr, "kp_wifi_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
