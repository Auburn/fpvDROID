#include "radio.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "log.h"
#include "receiver.h"
#include "spectrum.h"
#include "stream88xx.h"
#include "thread_placement.h"

namespace kp {

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

namespace {
constexpr double kFs = 40e6;
constexpr double kEdge = 16.5e6;     // usable +- span at 40 MS/s (BW40 DFIR is flat to ~17 MHz)
constexpr double kStep = 20e6;       // a hump up to ~13 MHz wide is fully inside some step's span
constexpr double kCapture = 0.09;    // s of IQ per scan step (60 ms acquisition + ~300 lines)
constexpr double kPsdSpan = 0.012;   // s of it for the spectrum
constexpr int kSettleBanks = 40;     // ~8 ms after a retune (chip AGC)
constexpr int kCapRestarts = 8;      // times a scan step's capture starts over after a gap; then gaps stay in
constexpr int64_t kSnrEveryMs = 500;  // live SNR while watching (~2 % of a core at 20 MS/s)
constexpr double kSnrSpan = 8e6;      // +- span the BW20 RX filter passes flat (-3 dB at ~9.2 MHz)
// Off-centre tuning at 40 MS/s: how far beside the pilot the hardware centre goes (Radio::offset_centre). The
// video swings about +-2.5 MHz around its carrier; this keeps that clear of the DC point and inside the flat band.
constexpr double kOffsetAim40 = 6e6;

std::vector<double> scan_plan() {
  std::vector<double> p;
  // the common analog bands first (A, B, E, F, R: 5645..5945), then the low band (L: 5362..5621)
  for (double c = 5652e6; c <= 5940e6; c += kStep) p.push_back(c);
  for (double c = 5370e6; c <= 5630e6; c += kStep) p.push_back(c);
  return p;
}
}  // namespace

Radio::Radio(FrameSink frames) : _gate(std::move(frames)) {}

Radio::~Radio() { close(); }

bool Radio::is_open() const {
  std::lock_guard<std::mutex> l(_m);
  return _open;
}

bool Radio::superspeed() const {
  std::lock_guard<std::mutex> l(_m);
  return _superspeed;
}

bool Radio::dead() const {
  std::lock_guard<std::mutex> l(_m);
  return _dead;
}

void Radio::open(int fd) {
  close();
  auto log = std::make_shared<Logger>();
  auto rx = std::make_unique<wsdr::Receiver>(log);
  wsdr::RxSettings s;
  s.freq = 5769e6;
  s.rate_sel = 1;  // 40 MS/s
  s.wide = true;
  s.agc = true;
  KP_LOGI("radio: opening adapter (%s)", fd >= 0 ? "Android fd" : "bus scan");
  rx->open(wsdr::kPid8812bu, s, -1, {}, fd);
  const auto caps = rx->caps();
  KP_LOGI("radio: %s, USB %s, fastest continuous rate %.1f MS/s", caps.chip.c_str(), caps.superspeed ? "3" : "2",
          wsdr::Receiver::rate_of(caps.min_rate_sel) / 1e6);
  if (!caps.superspeed) {
    // 40 MS/s does not stream on USB 2; open at a rate it can, so the stream still comes up for diagnostics
    s.rate_sel = caps.min_rate_sel;
    rx->set(s);
  }
  {
    std::lock_guard<std::mutex> l(_m);
    _superspeed = caps.superspeed;
    _chip = caps.chip;
    _open = true;
    _mode = Mode::Idle;
    _mode_epoch++;
    _req_hz = s.freq;
    _req_rate = s.rate_sel;
    _fb.clear();
    _dead = false;
    _wedged = false;
  }
  _banks = 0;
  _dropped = 0;
  _stop = false;
  {
    std::lock_guard<std::mutex> l(_rxm);
    _rx = std::move(rx);
  }
  _decq.reset_dropped();
  _gate.reset();
  _faults_seen = 0;
  _started = false;
  _dsp = std::thread([this] { dsp_loop(); });
  _decoder = std::thread([this] { decode_loop(); });
  _rx->start([this](const uint8_t *b, size_t n, const wsdr::BankMeta &m) {
    on_bank(b, n, m.seq, m.gen, m.fs, m.centre_hz, m.shift_hz);
  });
  _started = true;
}

void Radio::close() {
  _stop = true;
  _qcv.notify_all();
  if (_rx) _rx->stop();
  if (_dsp.joinable()) _dsp.join();
  _decq.wake();
  if (_decoder.joinable()) _decoder.join();
  _decq.clear();
  _fe.reset();
  // a recording cut short by the close: what was written stays (no .json: kp_replay then needs --fs)
  if (_rec) std::fclose(_rec), _rec = nullptr;
  if (_rec_b) std::fclose(_rec_b), _rec_b = nullptr;
  // out of status_json()'s sight first, then closed: closing can take a while, and a status poll (the app's
  // main thread) must not wait for it
  std::unique_ptr<wsdr::Receiver> rx;
  {
    std::lock_guard<std::mutex> l(_rxm);
    rx = std::move(_rx);
  }
  if (rx) {
    rx->close();
    rx.reset();
    KP_LOGI("radio: closed");
  }
  {
    std::lock_guard<std::mutex> l(_m);
    _open = false;
    _mode = Mode::Idle;
  }
  std::lock_guard<std::mutex> q(_qm);
  _q.clear();
}

// Receiver stream thread: copy only.
void Radio::on_bank(const uint8_t *b, size_t n, uint64_t seq, uint32_t gen, double fs, double centre, double shift) {
  thread_local bool boosted = false;
  if (!boosted) {
    boosted = true;
    boost_thread("usb stream");
  }
  std::vector<uint8_t> v;
  {
    std::lock_guard<std::mutex> l(_qm);
    if (!_pool.empty()) {
      v = std::move(_pool.back());
      _pool.pop_back();
    }
  }
  v.assign(b, b + n);
  {
    std::lock_guard<std::mutex> l(_qm);
    if (_q.size() >= _qcap) {
      _pool.push_back(std::move(_q.front().data));
      _q.pop_front();
      _dropped++;
    }
    _q.push_back(Bank{std::move(v), seq, gen, fs, centre, shift});
  }
  _banks++;
  _qcv.notify_one();
}

// The hardware centre on the 5 MHz grid (where the chip's 40 MHz channels put it). At 40 MS/s: the one whose
// distance from the pilot is nearest the aim, either side.
// At 20 MS/s: the one nearest the pilot, which is where the watch is without off-centre tuning. The stream ends
// 10 MHz from its centre, and a signal ~18 MHz wide loses one side as soon as it sits off the middle: up to 2 MHz
// out the picture is the centred one, at 2.5 to 3 MHz PAL already suffers, at 4 to 5 MHz bright colours wash out
// (measured with the --as20 filter of kp_replay). The grid offers the pilot one centre at most 2.5 MHz away and
// the next one at least 2.5 MHz away on the other side, so the near one it is; a pilot 2 MHz off the grid has the
// DC point at the edge of its video's swing that way, one on the grid has to live with it in the middle.
double Radio::offset_centre(double hz, double fs) {
  const double c0 = std::round(hz / 5e6) * 5e6;
  if (fs < 40e6) return c0;
  double best = c0, err = 1e18;
  for (int k = -3; k <= 3; k++) {
    const double c = c0 + k * 5e6, e = std::fabs(std::fabs(hz - c) - kOffsetAim40);
    if (e < err) err = e, best = c;
  }
  return best;
}

bool Radio::set_offset_tuning(bool on) {
  std::lock_guard<std::mutex> l(_m);
  if (_offset == on) return false;
  _offset = on;
  return true;
}

void Radio::tune(double hz, int rate_sel, bool watching) {
  wsdr::RxSettings s = _rx->settings();
  s.freq = hz;
  {
    std::lock_guard<std::mutex> l(_m);
    // recomputed on every watching tune, so a 20 MS/s fallback gets the centre for its own rate
    s.centre = watching && _offset ? offset_centre(hz, wsdr::Receiver::rate_of(rate_sel)) : 0;
  }
  if (s.centre > 0)
    KP_LOGI("radio: hardware centre %.0f MHz (pilot %+.1f MHz beside it)", s.centre / 1e6, (hz - s.centre) / 1e6);
  s.rate_sel = rate_sel;
  s.wide = true;
  {
    std::lock_guard<std::mutex> l(_m);
    s.agc = _gain_db < 0;
    if (_gain_db >= 0) s.gain_db = _gain_db;
  }
  _rx->set(s);
  std::lock_guard<std::mutex> l(_m);
  _req_hz = hz;
  _req_rate = rate_sel;
}

// The four mode calls do nothing on a radio that is not open, and that is decided under the lock open() and
// close() set _open under: a call that races a close cannot leave the closed radio in its mode.
void Radio::scan(double prefer_hz) {
  std::lock_guard<std::mutex> l(_m);
  if (!_open) return;
  _resume_hz = prefer_hz > 0 ? prefer_hz : _mode == Mode::Watch ? _watch_hz : 0;
  _mode = Mode::Scan;
  _mode_epoch++;
  // the status says "scan" from here on: its progress with it, not what the scan before left (the DSP thread
  // sets both again when it takes the scan up, and moves the step on only for the scan it is working on)
  _step = 0;
  _plan_n = scan_plan().size();
}

void Radio::watch(double hz) {
  std::lock_guard<std::mutex> l(_m);
  if (!_open) return;
  _watch_hz = hz;
  _mode = Mode::Watch;
  _mode_epoch++;
}

void Radio::stream_test(int rate_sel, double seconds) {
  std::lock_guard<std::mutex> l(_m);
  if (!_open) return;
  _mode = Mode::Test;
  _mode_epoch++;
  _req_rate = rate_sel;
  _test_end_ms = static_cast<int64_t>(seconds * 1000);
}

void Radio::idle() {
  std::lock_guard<std::mutex> l(_m);
  if (!_open) return;
  _mode = Mode::Idle;
  _mode_epoch++;
}

void Radio::set_watch_rate(int rate_sel) {
  std::lock_guard<std::mutex> l(_m);
  _watch_rate = rate_sel;
  _fb.clear();
}

void Radio::set_gain(double db) {
  std::lock_guard<std::mutex> l(_m);
  _gain_db = db;
}

void Radio::set_comb(bool on) {
  std::lock_guard<std::mutex> l(_m);
  _comb = on;
}

void Radio::set_combine(bool on) { _combine = on; }

void Radio::set_conceal(bool on) { _gate.set_enabled(on); }

void Radio::set_fast_cores(bool on) { _fast_cores = on; }

Radio::ComboStatus Radio::combine_status() {
  std::lock_guard<std::mutex> l(_m);
  return _comb_status;
}

void Radio::record(double seconds, const std::string &prefix) {
  std::lock_guard<std::mutex> l(_m);
  _rec_req_s = seconds;
  _rec_req_prefix = prefix;
}

std::vector<Pilot> Radio::pilots() {
  std::lock_guard<std::mutex> l(_m);
  return _pilots;
}

// One path's samples out of a bank's 64-bit words. Path A: I [11:0], Q [23:12] (la_i / la_q); path B the same
// 24 bits up: I [35:24], Q [47:36] (kDmaBothPaths8822b).
void Radio::unpack(const Bank &b, Path path, std::vector<cf> &out) {
  const int shift = path == kPathB ? 24 : 0;
  const size_t ns = b.data.size() / 8;
  out.resize(ns);
  const uint8_t *p = b.data.data();
  for (size_t i = 0; i < ns; i++) {
    uint64_t w;
    std::memcpy(&w, p + 8 * i, 8);
    w >>= shift;
    out[i] = cf(static_cast<float>(wsdr::la_i(w)), static_cast<float>(wsdr::la_q(w)));
  }
}

// Path B of the bank in hand, unpacked the first time something asks for it (combining, a recording).
const std::vector<cf> &Radio::path_b(const Bank &b) {
  if (!_iqb_fresh) {
    unpack(b, kPathB, _iqb);
    _iqb_fresh = true;
  }
  return _iqb;
}

void Radio::dsp_loop() {
  boost_thread("dsp");
  keep_off_slow_cores("dsp", _fast_cores);
  Bank b;
  _load_t0_ms = now_ms();
  while (!_stop) {
    bool none;
    {
      std::unique_lock<std::mutex> l(_qm);
      _qcv.wait_for(l, std::chrono::milliseconds(200), [&] { return !_q.empty() || _stop; });
      if (_stop) break;  // close(): stops the receiver too, which is not a death
      none = _q.empty();
      if (!none) {
        b = std::move(_q.front());
        _q.pop_front();
      }
    }
    if (none) {
      // Outside the queue's lock: _m is never taken while holding it. And only once open() has the receiver
      // started: this thread runs before that call, and a receiver not yet running is not one that stopped.
      if (_started && _rx) {
        const wsdr::RxStatus st = _rx->status();
        if (_stop) break;  // close() got in between: it stopped the receiver itself
        note_faults(st);
        if (!st.running) {
          KP_LOGE("radio: receiver stopped%s: %s", st.wedged ? " (C2H wedged)" : "", st.error.c_str());
          // nothing restarts it from here: say so, so the app can reopen the adapter (at once, if it wedged)
          std::lock_guard<std::mutex> m(_m);
          _dead = true;
          _wedged = st.wedged;
          if (st.wedged) _fb.receiver_wedged(_mode == Mode::Watch);
          break;
        }
      }
      continue;
    }
    const auto t0 = std::chrono::steady_clock::now();
    Mode mode;
    uint64_t epoch;
    {
      std::lock_guard<std::mutex> l(_m);
      mode = _mode;
      epoch = _mode_epoch;
    }
    const bool fresh = epoch != _epoch_seen;
    _epoch_seen = epoch;
    if (fresh) {
      // a mode change: set the radio up for it (a new watch builds a new decoder)
      _fe.reset();
      _fe_gen = ~0u;
      if (mode != Mode::Watch) _fb.watch_ended();
      switch (mode) {
      case Mode::Watch: begin_watch(); break;
      case Mode::Scan: begin_scan(); break;
      case Mode::Test: begin_test(); break;
      case Mode::Idle: break;
      }
    }
    switch (mode) {
    case Mode::Watch: handle_watch(b); break;
    case Mode::Scan: handle_scan(b); break;
    case Mode::Test: handle_test(b); break;
    case Mode::Idle: break;
    }
    {
      std::lock_guard<std::mutex> l(_qm);
      if (_pool.size() < _qcap) _pool.push_back(std::move(b.data));
    }
    _busy_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const int64_t t = now_ms();
    if (t - _load_t0_ms >= 1000 && _rx) note_faults(_rx->status());
    if (t - _load_t0_ms >= 1000) {
      std::lock_guard<std::mutex> l(_m);
      _load = _busy_s / ((t - _load_t0_ms) / 1000.0);
      _busy_s = 0;
      _load_t0_ms = t;
      if (++_status_ticks % 5 == 0 && _mode == Mode::Watch)
        KP_LOGI("status: watch %.1f MHz, cpu demod %.0f%% decode %.0f%%, dropped stream %llu usb %llu decode %llu, "
                "restarts %llu, %s %s lock %.2f fps %.0f, acq %llu lost %llu rephase %llu, torn frames hidden %llu",
                _watch_hz / 1e6,
                _load * 100, _dload * 100, (unsigned long long)(_rx ? _rx->status().mid_drops : 0),
                (unsigned long long)_dropped.load(), (unsigned long long)_decq.dropped(),
                (unsigned long long)(_rx ? _rx->status().restarts : 0), _vstats.std_name,
                _vstats.colour ? "colour" : "grey", _vstats.lock_recent, _fps,
                (unsigned long long)_vstats.acquisitions, (unsigned long long)_vstats.losses,
                (unsigned long long)_vstats.rephases, (unsigned long long)_gate.hidden());
    }
  }
}

// The three ways into a mode, on the DSP thread, with the first bank after the mode call.

void Radio::begin_watch() {
  double hz;
  int rate;
  {
    std::lock_guard<std::mutex> l(_m);
    hz = _watch_hz;
    // 40 MS/s with the chip's AGC, or 20 MS/s after a fallback. Both on a 40 MHz channel: at 20 MHz width the
    // OFDM CCA/AGC froze the LA in 120-sample runs (wifi-sdr RxSettings::wide).
    rate = _fb.rate_for(_watch_rate);
    _watch_comb = _comb;  // a decoder rebuilt later in this watch (fallback, shift) keeps it
    _snr_db = -1;
  }
  KP_LOGI("radio: watch %.3f MHz at %.0f MS/s", hz / 1e6, wsdr::Receiver::rate_of(rate) / 1e6);
  _fb.watch_started(now_ms());
  _snr_psd.reset();
  _snr_t_ms = 0;
  tune(hz, rate, true);
}

void Radio::begin_scan() {
  _plan = scan_plan();
  _plan_n = _plan.size();
  _at = 0;
  _step = 0;
  _cap.clear();
  _settle = -1;
  KP_LOGI("radio: scan, %zu steps", _plan.size());
  tune(_plan[0], 1);
}

void Radio::begin_test() {
  int rate;
  {
    std::lock_guard<std::mutex> l(_m);
    rate = _req_rate;
    _test_result.clear();
  }
  _test_banks = _test_gaps = 0;
  _test_last_seq = 0;
  _test_t0_ms = 0;
  _test_fs = wsdr::Receiver::rate_of(rate);
  KP_LOGI("radio: stream test at %.1f MS/s", _test_fs / 1e6);
  tune(5769e6, rate);
}

void Radio::note_faults(const wsdr::RxStatus &st) {
  if (st.restarts == _faults_seen) return;
  _faults_seen = st.restarts;
  KP_LOGW("radio: stream fault %llu (%llu banks so far, %u repeated starts): %s",
          static_cast<unsigned long long>(st.restarts), static_cast<unsigned long long>(st.banks), st.start_retries,
          st.last_fault.c_str());
}

// A new carrier or rate: a new decoder. No weights or noise estimates from the last one either, and the paths'
// time offset is measured again (it is a number of samples, so it depends on the rate).
void Radio::new_front_end(const Bank &b) {
  DecoderOpts o;
  o.comb = _watch_comb;
  _fe = std::make_shared<FrontEnd>(b.fs, b.shift_hz, _gate.sink(), o);
  _fe_fs = b.fs;
  _fe_shift = b.shift_hz;
  _combiner.reset(b.fs);
}

// The stream restarted with the same settings: the decoder is kept, and re-phases onto the new stream within a
// few lines. True: one restart too many for 40 MS/s (FallbackPolicy).
bool Radio::stream_restarted(const Bank &b, int64_t t_ms) {
  std::string why;
  if (_rx) why = _rx->status().last_fault;
  KP_LOGW("radio: stream restarted (gen %u, %s), decoder keeps going", b.gen, why.c_str());
  return _fb.stream_restarted(t_ms, b.fs);
}

// The receiver's count of the blocks its stream gave up on, read every FallbackPolicy::kPollMs. True: too many
// lately for 40 MS/s.
bool Radio::blocks_lost(const Bank &b, int64_t t_ms) {
  return _rx && _fb.poll_due(t_ms) && _fb.blocks_lost(t_ms, b.fs, _rx->status().mid_drops);
}

// The watch does not hold up at 40 MS/s (FallbackPolicy has it at 20 MS/s from here on): the bank in hand is
// from the old stream, the next ones come at 20 MS/s to a new decoder.
void Radio::fall_back(double hz) {
  _fe.reset();
  _fe_gen = ~0u;
  tune(hz, 2, true);  // as a watch: off-centre tuning picks the centre for 20 MS/s
}

void Radio::measure_snr(const std::vector<cf> &iq, double fs, double shift_hz) {
  if (!_snr_psd) {
    if (now_ms() - _snr_t_ms < kSnrEveryMs) return;
    _snr_psd = std::make_unique<Psd>(1024);
    _snr_have = 0;
    _snr_need = static_cast<size_t>(kPsdSpan * fs);
  }
  _snr_psd->add(iq.data(), iq.size());
  _snr_have += iq.size();
  if (_snr_have < _snr_need) return;
  // Around the carrier (at shift_hz: beside the centre with off-centre tuning), within the band the RX filter
  // passes: the BW40 DFIR's +-kEdge at 40 MS/s, the BW20 set's +-kSnrSpan at 20 MS/s.
  const double pass = fs >= 40e6 ? kEdge : std::min(kSnrSpan, 0.4 * fs);
  const float snr = carrier_snr(_snr_psd->db(), fs, std::min(kSnrSpan, 0.4 * fs), shift_hz, pass);
  _snr_psd.reset();
  _snr_t_ms = now_ms();
  std::lock_guard<std::mutex> l(_m);
  _snr_db = snr;
}

// One bank of a watch: both antennas' samples in, a block of discriminator output on to the decode thread.
void Radio::handle_watch(Bank &b) {
  double req;
  int req_rate;
  {
    std::lock_guard<std::mutex> l(_m);
    req = _req_hz;
    req_rate = _req_rate;
  }
  // ignore banks from before the retune (or the rate change of a fallback)
  if (std::fabs(b.centre_hz + b.shift_hz - req) > 1e3 || b.fs != wsdr::Receiver::rate_of(req_rate)) return;
  const int64_t t = now_ms();
  // the decoder for this stream, and whether 40 MS/s is holding up (a restart now, blocks lost lately)
  bool give_up = false;
  if (!_fe || b.fs != _fe_fs || b.shift_hz != _fe_shift) new_front_end(b);
  else if (b.gen != _fe_gen) give_up = stream_restarted(b, t);
  if (give_up || blocks_lost(b, t)) {
    fall_back(req);
    return;
  }
  // After a restart, or banks lost on the way here, this bank does not follow on from the last one.
  const bool gap = b.gen != _fe_gen || b.seq != _watch_seq + 1;
  _fe_gen = b.gen;
  _watch_seq = b.seq;
  unpack(b, kPathA, _iq);
  _iqb_fresh = false;
  measure_snr(_iq, b.fs, b.shift_hz);  // path A, as without combining
  record_bank(b);                      // both paths as they came in
  combine_antennas(b, gap, t);
  demod_for_decoder(gap);
}

// Two-antenna video: path B lined up and turned into step with A and both weighted by their SNR, in place of A
// in _iq. gap: the combiner must not delay path B with samples from before the hole.
void Radio::combine_antennas(const Bank &b, bool gap, int64_t t_ms) {
  if (gap) _combiner.cut();
  const bool comb = _combine.load();
  if (comb != _combine_was) {
    _combiner.reset(b.fs);
    _combine_was = comb;
    KP_LOGI("radio: video antennas %s", comb ? "A + B combined" : "A only");
  }
  if (comb) {
    const std::vector<cf> &iqb = path_b(b);
    _combiner.process(_iq.data(), iqb.data(), std::min(_iq.size(), iqb.size()));
  }
  if (t_ms - _comb_t0_ms < 1000) return;
  // once a second: the combiner's numbers for the status
  _comb_t0_ms = t_ms;
  ComboStatus cs;
  cs.on = comb;
  if (comb) cs.s = _combiner.take_stats();
  bool news;
  {
    std::lock_guard<std::mutex> l(_m);
    news = comb && (!_comb_status.on || cs.s.lag != _comb_status.s.lag ||
                    cs.s.lag_measured != _comb_status.s.lag_measured);
    _comb_status = cs;
  }
  if (news)
    KP_LOGI("radio: antenna B taken as %d samples (%.0f ns) ahead of A, %s", cs.s.lag, cs.s.lag / b.fs * 1e9,
            cs.s.lag_measured ? "measured" : "the adapter's usual figure");
}

// _iq through the discriminator (this thread's half of the work), then on to the decode thread. gap: the frames
// around this block are torn (FrameGate).
void Radio::demod_for_decoder(bool gap) {
  DemodBlock blk = _decq.blank();
  blk.fe = _fe;
  blk.gap = gap;
  _fe->demod(_iq.data(), _iq.size(), blk.d, &blk.bad);  // bad stays empty while sparkle repair is off
  _decq.push(std::move(blk));
}

namespace {
// Samples as int16 pairs, LA value << 4, as wsdr_probe writes .cs16.
void write_cs16(FILE *f, const std::vector<cf> &iq, size_t ns, std::vector<int16_t> &buf) {
  buf.resize(2 * ns);
  for (size_t i = 0; i < ns; i++) {
    buf[2 * i] = static_cast<int16_t>(static_cast<int>(iq[i].real()) * 16);
    buf[2 * i + 1] = static_cast<int16_t>(static_cast<int>(iq[i].imag()) * 16);
  }
  std::fwrite(buf.data(), 4, ns, f);
}
}  // namespace

// Debug recording: the watched IQ of both paths, whole banks, until the seconds asked for are written.
void Radio::record_bank(const Bank &b) {
  if (!_rec && !begin_recording(b)) return;
  const size_t ns = std::min<uint64_t>(_iq.size(), _rec_left);
  std::vector<int16_t> buf;
  write_cs16(_rec, _iq, ns, buf);
  if (_rec_b) write_cs16(_rec_b, path_b(b), ns, buf);
  _rec_left -= ns;
  _rec_samples += ns;
  if (_rec_left == 0) end_recording();
}

// Takes up a request made with record(). False: there is none, or its file cannot be written.
bool Radio::begin_recording(const Bank &b) {
  double secs;
  std::string prefix;
  {
    std::lock_guard<std::mutex> l(_m);
    secs = _rec_req_s;
    prefix = _rec_req_prefix;
    _rec_req_s = 0;
  }
  if (secs <= 0) return false;
  _rec = std::fopen((prefix + ".cs16").c_str(), "wb");
  if (!_rec) {
    KP_LOGE("record: cannot open %s.cs16", prefix.c_str());
    return false;
  }
  std::setvbuf(_rec, nullptr, _IOFBF, 1 << 20);
  _rec_b = std::fopen((prefix + "_b.cs16").c_str(), "wb");
  if (_rec_b) std::setvbuf(_rec_b, nullptr, _IOFBF, 1 << 20);
  _rec_prefix = prefix;
  _rec_left = static_cast<uint64_t>(secs * b.fs);
  _rec_samples = 0;
  _rec_fs = b.fs;
  _rec_centre = b.centre_hz;
  _rec_shift = b.shift_hz;
  KP_LOGI("record: %.1f s at %.0f MS/s -> %s.cs16", secs, b.fs / 1e6, prefix.c_str());
  return true;
}

// Closes the files and writes PREFIX.json beside them (what kp_replay reads the rate and centre from).
void Radio::end_recording() {
  std::fclose(_rec);
  _rec = nullptr;
  const bool with_b = _rec_b != nullptr;
  if (_rec_b) std::fclose(_rec_b), _rec_b = nullptr;
  const std::string base = _rec_prefix.substr(_rec_prefix.find_last_of("/\\") + 1);
  if (FILE *j = std::fopen((_rec_prefix + ".json").c_str(), "w")) {
    std::fprintf(j, "{\"format\":\"cs16\",\"sample_rate\":%.0f,\"centre_hz\":%.0f,\"shift_hz\":%.0f,\"samples\":%llu,"
                 "\"source\":\"kepyur watch\"", _rec_fs, _rec_centre, _rec_shift, (unsigned long long)_rec_samples);
    if (with_b) std::fprintf(j, ",\"path_b\":\"%s_b.cs16\"", base.c_str());  // beside it, as Play RX writes it
    std::fprintf(j, "}\n");
    std::fclose(j);
  }
  KP_LOGI("record: done, %llu samples -> %s.cs16", (unsigned long long)_rec_samples, _rec_prefix.c_str());
}

void Radio::decode_loop() {
  boost_thread("decode");
  keep_off_slow_cores("decode", _fast_cores);
  std::shared_ptr<FrontEnd> cur;
  double busy = 0;
  int64_t t0 = now_ms();
  while (!_stop) {
    DemodBlock blk;
    if (!_decq.pop(blk, 200, _stop)) continue;
    const auto ta = std::chrono::steady_clock::now();
    if (blk.fe != cur) {
      cur = blk.fe;
      _frames_at_last = 0;
      _fps_t0_ms = now_ms();
    }
    if (blk.gap) _gate.gap(now_ms());  // the frame this block is in is torn, and the next while the decoder re-phases
    cur->decode(blk.d.data(), blk.d.size(), blk.bad.size() == blk.d.size() ? blk.bad.data() : nullptr);
    busy += std::chrono::duration<double>(std::chrono::steady_clock::now() - ta).count();
    _decq.recycle(std::move(blk));
    const int64_t t = now_ms();
    if (t - _fps_t0_ms >= 1000) {
      const auto s = cur->stats();
      std::lock_guard<std::mutex> l(_m);
      _vstats = s;
      _fps = static_cast<float>((s.frames - _frames_at_last) * 1000.0 / (t - _fps_t0_ms));
      _frames_at_last = s.frames;
      _fps_t0_ms = t;
    }
    if (t - t0 >= 1000) {
      std::lock_guard<std::mutex> l(_m);
      _dload = busy / ((t - t0) / 1000.0);
      busy = 0;
      t0 = t;
    }
  }
}

void Radio::handle_scan(Bank &b) {
  if (_at >= _plan.size()) return;
  const double req = _plan[_at];
  if (std::fabs(b.centre_hz + b.shift_hz - req) > 1e3 || b.fs != kFs) return;
  if (_settle < 0) {
    _settle = kSettleBanks;
    _cap_restarts = 0;
    _cap.clear();
    _cap_need = static_cast<size_t>(kPsdSpan * kFs);  // first just enough for the spectrum
    _humps.clear();
  }
  if (_settle > 0) {
    _settle--;
    _cap_last_seq = b.seq;
    _cap_gen = b.gen;
    return;
  }
  // A gap (banks dropped on the way, or the stream restarted): start the capture over. Not for ever: on a link
  // that loses a bank every few ms no capture would ever be whole and the scan would stay on this step, so after
  // kCapRestarts tries it is taken with its gaps (the sync check loses a few lines at each).
  const bool gap = b.seq != _cap_last_seq + 1 || b.gen != _cap_gen;
  if (!_cap.empty() && gap && _cap_restarts++ < kCapRestarts) _cap.clear();
  _cap_last_seq = b.seq;
  _cap_gen = b.gen;
  _cap_centre = b.centre_hz;
  const size_t ns = b.data.size() / 8;
  const uint8_t *p = b.data.data();
  for (size_t i = 0; i < ns && _cap.size() < _cap_need; i++) {
    uint64_t w;
    std::memcpy(&w, p + 8 * i, 8);
    _cap.emplace_back(static_cast<int16_t>(wsdr::la_i(w)), static_cast<int16_t>(wsdr::la_q(w)));
  }
  if (_cap.size() < _cap_need) return;
  if (_cap_need < static_cast<size_t>(kCapture * kFs)) {
    // spectrum first: a step with no candidate is done now; otherwise keep capturing for the sync check
    std::vector<cf> x(_cap.size());
    for (size_t i = 0; i < _cap.size(); i++) x[i] = cf(_cap[i].real(), _cap[i].imag());
    Psd psd(1024);
    psd.add(x.data(), x.size());
    _humps = find_humps(psd.db(), kFs, kEdge);
    if (!_humps.empty()) {
      _cap_need = static_cast<size_t>(kCapture * kFs);
      return;
    }
  }
  scan_step_done();
}

void Radio::scan_step_done() {
  const double centre = _cap_centre;  // the samples are centred on the hardware centre, not the request
  // start the next retune now; the chip settles while this step is analysed
  _at++;
  if (_at < _plan.size()) {
    tune(_plan[_at], 1);
    _settle = -1;
  }
  std::vector<cf> x(_humps.empty() ? 0 : _cap.size());
  for (size_t i = 0; i < x.size(); i++) x[i] = cf(_cap[i].real(), _cap[i].imag());
  const auto humps = _humps;
  std::vector<Pilot> found;
  for (const auto &h : humps) {
    const auto vc = check_video(x.data(), x.size(), kFs, h.offset_hz);
    const double mhz = (centre + h.offset_hz) / 1e6;
    KP_LOGI("scan: %.1f MHz hump %+.2f MHz (%.1f MHz wide, %.1f dB) -> %s %s lock %.2f", centre / 1e6,
            h.offset_hz / 1e6, h.width_hz / 1e6, h.snr_db, vc.video ? "VIDEO" : "no video", vc.std_name, vc.lock);
    if (!vc.video) continue;
    Pilot pl;
    pl.mhz = mhz;
    const auto *ch = nearest_channel(mhz);
    pl.name = ch ? ch->name : "";
    pl.std_name = vc.std_name;
    pl.snr_db = h.snr_db;
    pl.lock = vc.lock;
    pl.seen_ms = now_ms();
    found.push_back(pl);
  }
  const bool done = _at >= _plan.size();
  double next_watch = 0;
  {
    std::lock_guard<std::mutex> l(_m);
    // A mode call made since this bank's mode was read (from another thread, while this step was analysed)
    // has the last word: the status shows its progress (a scan called again starts at 0), and the end of
    // this scan does not put a watch of its own choosing over it. The next bank takes that call up.
    const bool current = _mode_epoch == _epoch_seen;
    if (current) _step = _at;
    for (const auto &f : found) {
      auto it = std::find_if(_pilots.begin(), _pilots.end(), [&](const Pilot &p) { return std::fabs(p.mhz - f.mhz) < 4; });
      if (it == _pilots.end()) _pilots.push_back(f);
      else if (f.snr_db >= it->snr_db - 3) *it = f;  // prefer the newer sighting unless much weaker
      else it->seen_ms = f.seen_ms;
    }
    std::sort(_pilots.begin(), _pilots.end(), [](const Pilot &a, const Pilot &b) { return a.snr_db > b.snr_db; });
    if (done && current) {
      // after a scan: the pilot watched before, if still there, else the strongest seen in this scan
      const int64_t scan_start = now_ms() - 60000;
      for (const auto &p : _pilots)
        if (_resume_hz > 0 && std::fabs(p.mhz * 1e6 - _resume_hz) < 4e6) next_watch = p.mhz * 1e6;
      if (next_watch == 0)
        for (const auto &p : _pilots)
          if (p.seen_ms >= scan_start) {
            next_watch = p.mhz * 1e6;
            break;
          }
      if (next_watch == 0 && _resume_hz > 0) next_watch = _resume_hz;
      KP_LOGI("scan: done, %zu pilot(s)%s", _pilots.size(), next_watch > 0 ? ", watching" : "");
      if (next_watch > 0) {
        _watch_hz = next_watch;
        _mode = Mode::Watch;
      } else {
        _mode = Mode::Idle;
      }
      _mode_epoch++;
    }
  }
  _cap.clear();
}

void Radio::handle_test(Bank &b) {
  if (std::fabs(b.fs - _test_fs) > 1) return;
  const int64_t t = now_ms();
  if (_test_t0_ms == 0) {
    _test_t0_ms = t;
    _test_last_seq = b.seq;
    _test_gen = b.gen;
    _test_runs = 1;
    KP_LOGI("test: first bank at %.1f MS/s (gen %u)", b.fs / 1e6, b.gen);
  } else if (b.gen != _test_gen) {
    _test_gen = b.gen;  // a stream restart (seq starts over): counted as a run, not as a gap
    _test_runs++;
    KP_LOGW("test: stream restarted (gen %u): %s", b.gen, _rx->status().last_fault.c_str());
  } else if (b.seq != _test_last_seq + 1) {
    _test_gaps++;
  }
  _test_last_seq = b.seq;
  _test_banks++;
  const double el = (t - _test_t0_ms) / 1000.0;
  const double expect = el * b.fs / wsdr::Stream8822b::kBankSamples;
  std::ostringstream o;
  o << "{\"rate_msps\":" << b.fs / 1e6 << ",\"seconds\":" << el << ",\"banks\":" << _test_banks
    << ",\"expected\":" << static_cast<uint64_t>(expect) << ",\"seq_gaps\":" << _test_gaps
    << ",\"stream_runs\":" << _test_runs
    << ",\"restarts\":" << _rx->status().restarts << ",\"queue_dropped\":" << _dropped.load()
    << ",\"done\":" << (el * 1000 >= _test_end_ms ? "true" : "false") << "}";
  std::lock_guard<std::mutex> l(_m);
  _test_result = o.str();
  // (not over a mode call made since this bank's mode was read: as at the end of a scan)
  if (el * 1000 >= _test_end_ms && _mode_epoch == _epoch_seen) {
    KP_LOGI("test: result %s", _test_result.c_str());
    _mode = Mode::Idle;
    _mode_epoch++;
  }
}

static std::string jstr(const std::string &s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    if (static_cast<unsigned char>(c) < 0x20) continue;
    o += c;
  }
  return o + "\"";
}

std::string Radio::status_json() {
  std::ostringstream o;
  wsdr::RxStatus rs;
  {
    std::lock_guard<std::mutex> l(_rxm);
    if (_rx) rs = _rx->status();
  }
  std::lock_guard<std::mutex> l(_m);
  // A JSON number or nothing: a nan or inf printed as it comes is not JSON, and a reader may choke on it or,
  // worse, take it for a string (Android's does).
  const auto num = [](double v) { return std::isfinite(v) ? v : 0.0; };
  static const char *modes[] = {"idle", "scan", "watch", "test"};
  const int64_t t = now_ms();
  // late_recent: of stream_dropped (every block the adapter's stream gave up on since open()), those of the
  // watch now running and of its last FallbackPolicy::kWindowMs: the host is not keeping up when they add up
  const uint64_t late_recent = _mode == Mode::Watch ? _fb.late_recent(t) : 0;
  o << "{\"open\":" << (_open ? "true" : "false") << ",\"usb3\":" << (_superspeed ? "true" : "false")
    << ",\"chip\":" << jstr(_chip) << ",\"rx_state\":" << jstr(wsdr::rx_state_name(rs.state))
    << ",\"rx_error\":" << jstr(rs.error) << ",\"last_fault\":" << jstr(rs.last_fault)
    << ",\"mode\":\"" << modes[static_cast<int>(_mode)] << "\",\"freq_hz\":" << num(_req_hz)
    << ",\"watch_hz\":" << num(_watch_hz) << ",\"fs\":" << num(rs.fs) << ",\"banks\":" << _banks.load()
    << ",\"dropped\":" << _dropped.load() << ",\"decode_dropped\":" << _decq.dropped()
    << ",\"stream_dropped\":" << rs.mid_drops << ",\"late_recent\":" << late_recent
    << ",\"restarts\":" << rs.restarts << ",\"dsp_load\":" << num(_load) << ",\"decode_load\":" << num(_dload)
    << ",\"watch_msps\":" << (_mode == Mode::Watch ? wsdr::Receiver::rate_of(_req_rate) / 1e6 : 0)
    << ",\"fallback\":" << (_fb.active() ? "true" : "false") << ",\"dead\":" << (_dead ? "true" : "false")
    << ",\"wedged\":" << (_wedged ? "true" : "false")
    << ",\"offset_tuning\":" << (_offset ? "true" : "false") << ",\"centre_hz\":" << num(rs.centre_hz);
  if (_mode == Mode::Watch && _snr_db >= 0) o << ",\"snr_db\":" << num(_snr_db);
  if (_mode == Mode::Scan) o << ",\"scan_step\":" << _step.load() << ",\"scan_steps\":" << _plan_n.load();
  o << ",\"video\":{\"locked\":" << (!_vstats.acquiring && _vstats.lock_recent > 0.5f ? "true" : "false")
    << ",\"lock\":" << num(_vstats.lock_recent) << ",\"std\":" << jstr(_vstats.std_name) << ",\"fps\":" << num(_fps)
    << ",\"acquisitions\":" << _vstats.acquisitions << ",\"losses\":" << _vstats.losses
    << ",\"rephases\":" << _vstats.rephases << ",\"clicks_repaired\":" << _vstats.clicks_repaired
    << ",\"lines_mended\":" << _vstats.lines_mended << ",\"colour\":" << (_vstats.colour ? "true" : "false") << "}";
  {
    const auto &d = _comb_status.s;
    o << ",\"antennas\":{\"combine\":" << (_comb_status.on ? "true" : "false") << ",\"snr_a\":" << num(d.snr_a_db)
      << ",\"snr_b\":" << num(d.snr_b_db) << ",\"snr_c\":" << num(d.snr_c_db) << ",\"b_share\":" << num(d.b_share)
      << ",\"a_fade\":" << num(d.a_fade) << ",\"rescued\":" << num(d.rescued) << ",\"coherence\":" << num(d.coherence)
      << ",\"lag\":" << d.lag << ",\"lag_measured\":" << (d.lag_measured ? "true" : "false") << "}";
  }
  o << ",\"test\":" << (_test_result.empty() ? "null" : _test_result);
  o << ",\"pilots\":[";
  for (size_t i = 0; i < _pilots.size(); i++) {
    const auto &p = _pilots[i];
    o << (i ? "," : "") << "{\"mhz\":" << num(p.mhz) << ",\"name\":" << jstr(p.name) << ",\"std\":" << jstr(p.std_name)
      << ",\"snr\":" << num(p.snr_db) << ",\"age_s\":" << (t - p.seen_ms) / 1000 << "}";
  }
  o << "]}";
  return o.str();
}

}  // namespace kp
