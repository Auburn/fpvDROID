#include "sweep.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "spectrum.h"

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

std::string band_json(const std::vector<float> &db, double first_hz, double bin_hz, double at_hz, double at_width_hz,
                      uint64_t sweeps, const std::vector<DigitalSignal> &digital) {
  const auto carriers = find_carriers(db, first_hz, bin_hz);
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(1);
  o << "{\"first_mhz\":" << first_hz / 1e6 << ",\"bin_mhz\":" << bin_hz / 1e6 << ",\"at_mhz\":" << at_hz / 1e6
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
    if (const DigitalSignal *d = digital_for(digital, c)) o << ",\"kind\":\"" << digital_code(d->kind) << "\"";
    o << "}";
  }
  o << "]}";
  return o.str();
}

}  // namespace kp
