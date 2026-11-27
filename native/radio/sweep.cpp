#include "sweep.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace kp {

std::vector<double> Sweep::plan(double lo_hz, double hi_hz, int grid) {
  std::vector<double> p;
  const double first = std::round((lo_hz + (grid ? 0 : kStep / 2)) / 5e6) * 5e6;
  for (double c = first; p.empty() || c - kStep / 2 < hi_hz; c += kStep) p.push_back(c);
  return p;
}

void Sweep::start(double lo_hz, double hi_hz) {
  _lo = std::clamp(std::min(lo_hz, hi_hz), kBandLo, kBandHi);
  _hi = std::clamp(std::max(lo_hz, hi_hz), kBandLo, kBandHi);
  _grid = 0;
  _passes = 0;
}

StepFindings StepAnalysers::step(const std::vector<cf> &iq, double fs, double centre_hz, double edge_hz) {
  StepFindings f;
  f.centre_hz = centre_hz;
  f.fs = fs;
  f.edge_hz = edge_hz;
  _psd.reset();
  _psd.add(iq.data(), iq.size());
  f.spectrum = smooth_db(_psd.db(), fs / _psd.size());
  _grid.build(iq.data(), iq.size(), fs, edge_hz);
  f.digital = _digital.step(f.spectrum, _grid, centre_hz, edge_hz);
  // the bins handed on reach half a step either side: a pass measures every frequency once
  f.bursts = measure_bursts(_grid, centre_hz, Sweep::kStep / 2);
  return f;
}

BandPicture::BandPicture()
    : _band(Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin), _wifi(Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin) {}

void BandPicture::sweep_started(double lo_hz, double hi_hz, int64_t now_ms) {
  _band.forget(lo_hz, hi_hz, now_ms - Sweep::kStaleMs);
  _digital.forget(lo_hz, hi_hz, now_ms - Sweep::kStaleMs);
  _wifi.forget(lo_hz, hi_hz, now_ms - Sweep::kStaleMs);
  _lo = lo_hz;
  _hi = hi_hz;
}

void BandPicture::add(const StepFindings &f, int64_t now_ms) {
  _band.add(f.spectrum, f.fs, f.centre_hz, f.edge_hz, now_ms);
  _digital.add(f.digital, now_ms);
  // Over the step's whole reach, which goes past the ends of the sweep's span: a signal centred just outside it is
  // judged by the step at that end, and goes the same way.
  _digital.forget(f.centre_hz - f.edge_hz, f.centre_hz + f.edge_hz, now_ms - DigitalMap::kKeepMs);
  double first_hz = 0;
  const std::vector<float> db = _band.get(_lo, _hi, &first_hz);
  _wifi.add(f.bursts, _lo, _hi, find_carriers(db, first_hz, _band.bin_hz()), now_ms);
}

void BandPicture::add_levels(const std::vector<float> &sm, double fs, double centre_hz, double edge_hz, int64_t now_ms) {
  _band.add(sm, fs, centre_hz, edge_hz, now_ms);
}

std::string BandPicture::json(double lo_hz, double hi_hz, double at_hz, double at_width_hz, uint64_t sweeps) const {
  double first_hz = 0;
  const std::vector<float> db = _band.get(lo_hz, hi_hz, &first_hz);
  const auto carriers = find_carriers(db, first_hz, _band.bin_hz());
  std::vector<const char *> kind(carriers.size(), nullptr);  // the kind of the signal each carrier is on, if any
  std::vector<Signal> signals;
  // digital video: each verdict's block, and the carrier digital_for gives it to
  for (const auto &d : _digital.all()) {
    const double lo = d.centre_hz - d.width_hz / 2, hi = d.centre_hz + d.width_hz / 2;
    // A video link sends all the time it is on (DJI's frame with no goggles linked, which upstream measured too, is
    // not told apart from its frame with video).
    if (hi > lo_hz && lo < hi_hz) signals.push_back(Signal{lo, hi, digital_code(d.kind), digital_label(d.kind), true});
  }
  for (size_t i = 0; i < carriers.size(); i++)
    if (const DigitalSignal *d = digital_for(_digital.all(), carriers[i])) kind[i] = digital_code(d->kind);
  // Wi-Fi: each network that reaches into lo..hi, active once seen in use, and the carrier a busy one makes
  const std::vector<WifiNet> nets = _wifi.networks(lo_hz, hi_hz);
  for (const auto &n : nets) signals.push_back(Signal{n.lo_mhz * 1e6, n.hi_mhz * 1e6, "wifi", n.label(), n.busy});
  for (size_t i = 0; i < carriers.size(); i++)
    if (!kind[i] && wifi_for(nets, carriers[i])) kind[i] = "wifi";
  std::sort(signals.begin(), signals.end(), [](const Signal &a, const Signal &b) { return a.lo_hz < b.lo_hz; });

  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(1);
  o << "{\"first_mhz\":" << first_hz / 1e6 << ",\"bin_mhz\":" << _band.bin_hz() / 1e6 << ",\"at_mhz\":" << at_hz / 1e6
    << ",\"at_width_mhz\":" << at_width_hz / 1e6 << ",\"sweeps\":" << sweeps << ",\"db\":[";
  for (size_t i = 0; i < db.size(); i++) {
    if (i) o << ',';
    if (db[i] == BandMap::kNone || !std::isfinite(db[i])) o << "null";
    else o << db[i];
  }
  o << "],\"carriers\":[";
  for (size_t i = 0; i < carriers.size(); i++) {
    const auto &c = carriers[i];
    o << (i ? "," : "") << "{\"mhz\":" << c.mhz << ",\"width\":" << c.width_mhz << ",\"snr\":" << c.snr_db;
    if (kind[i]) o << ",\"kind\":\"" << kind[i] << "\"";
    o << "}";
  }
  o << "],\"signals\":[";
  for (size_t i = 0; i < signals.size(); i++) {
    const auto &s = signals[i];
    o << (i ? "," : "") << "{\"lo_mhz\":" << s.lo_hz / 1e6 << ",\"hi_mhz\":" << s.hi_hz / 1e6 << ",\"kind\":\""
      << s.kind << "\",\"label\":\"" << s.label << "\",\"active\":" << (s.active ? "true" : "false") << "}";
  }
  o << "]}";
  return o.str();
}

}  // namespace kp
