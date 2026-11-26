#include "steps.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace kp {

void Steps::start(std::vector<double> plan, size_t need) {
  _plan = std::move(plan);
  _at = 0;
  _first_need = need;
  _settle = -1;
}

void Steps::insert(const std::vector<double> &hz) {
  const size_t at = std::min(_at + 1, _plan.size());
  _plan.insert(_plan.begin() + static_cast<std::ptrdiff_t>(at), hz.begin(), hz.end());
}

bool Steps::next() {
  _settle = -1;
  return ++_at < _plan.size();
}

bool Steps::take(const std::vector<cf> &bank, uint64_t seq, uint32_t gen, double centre_hz) {
  if (_settle < 0) {
    _settle = kSettleBanks;
    _restarts = 0;
    _cap.clear();
    _need = _first_need;
  }
  if (_settle > 0) {
    _settle--;
    _last_seq = seq;
    _gen = gen;
    return false;
  }
  const bool gap = seq != _last_seq + 1 || gen != _gen;
  if (!_cap.empty() && gap && _restarts++ < kRestarts) _cap.clear();
  _last_seq = seq;
  _gen = gen;
  _centre = centre_hz;
  for (size_t i = 0; i < bank.size() && _cap.size() < _need; i++)
    _cap.emplace_back(static_cast<int16_t>(bank[i].real()), static_cast<int16_t>(bank[i].imag()));
  return _cap.size() >= _need;
}

std::vector<cf> Steps::iq() const {
  std::vector<cf> x(_cap.size());
  for (size_t i = 0; i < _cap.size(); i++) x[i] = cf(_cap[i].real(), _cap[i].imag());
  return x;
}

}  // namespace kp
