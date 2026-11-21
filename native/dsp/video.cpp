// kepyur video decoder. See video.h. The algorithm follows vtx-spike/viewer/vtxview.cpp (measured on a
// real R4 NTSC VTX: 93-96 % of lines locked); changes are marked "kepyur:".
#include "video.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "biquad.h"
#include "deinterlace.h"
#include "line_draw.h"
#include "log.h"
#include "rings.h"
#include "sparkle.h"

namespace kp {

const VideoStd kNtsc{63.5556, 9.4, 52.6, 240, 18, "NTSC"};
const VideoStd kPal{64.0, 10.5, 52.0, 288, 22, "PAL"};

// ---- picture switches (every decoder in the process; see video.h) ----
namespace {
std::atomic<bool> g_luma_comb{false}, g_acomb{false}, g_repair{false}, g_smooth{false};
}
void set_luma_comb(bool on) { g_luma_comb = on; }
bool luma_comb_on() { return g_luma_comb.load(std::memory_order_relaxed); }
void set_adaptive_comb(bool on) { g_acomb = on; }
bool adaptive_comb_on() { return g_acomb.load(std::memory_order_relaxed); }
void set_repair(bool on) { g_repair = on; }
bool repair_on() { return g_repair.load(std::memory_order_relaxed); }
void set_smooth_lines(bool on) { g_smooth = on; }
bool smooth_lines_on() { return g_smooth.load(std::memory_order_relaxed); }

// ---- the decoder (streaming, one sample of discriminator output at a time, MHz) ----------------------
class Decoder {
public:
  Decoder(double fs, FrameSink sink, const DecoderOpts &o) : fs(fs), sink(std::move(sink)), o(o), drawer(o.width, fs), spark(fs) {
    // chroma: mixed down by the subcarrier, 4th-order Butterworth low-pass at 1.2 MHz (U/V bandwidth)
    for (Biquad *b : {&cl_r1, &cl_i1}) b->design(1.2e6, fs, 0.54);
    for (Biquad *b : {&cl_r2, &cl_i2}) b->design(1.2e6, fs, 1.31);
    lp_luma.design(3.0e6, fs, 0.54);
    lp_luma2.design(3.0e6, fs, 1.31);
    lp_sync.design(1.0e6, fs);
    // luma comb: one gentle section (Q 0.6: no peaking, ~1 % overshoot), -3 dB near 4 MHz
    lp_wide.design(4.8e6, fs, 0.6);
    lp_vert.design(1.0e6, fs);  // adaptive comb: brightness below the colour band, line against line
    // colour / brightness registration: the notch chain was aligned by hand (0.21 us); the comb's luma filter
    // is faster, so colour must come that much later relative to it
    {
      const auto delay = [](Biquad a, Biquad *b) {  // group delay at low frequency: the impulse's centroid
        double s = 0, sw = 0;
        for (int i = 0; i < 2048; i++) {
          float x = i == 0 ? 1.f : 0.f;
          x = a.run(x);
          if (b) x = b->run(x);
          s += x;
          sw += x * static_cast<double>(i);
        }
        return s != 0 ? sw / s : 0;
      };
      Biquad l2 = lp_luma2;
      cdel_comb = 0.21e-6 * fs + (delay(lp_luma, &l2) - delay(lp_wide, nullptr));
    }
    acq.resize(static_cast<size_t>(fs * 0.06));  // 60 ms
    hist.assign(800, 0);
    set_std(o.std_force == 2 ? kPal : kNtsc);
  }

  DecoderStats stats() const {
    DecoderStats s;
    s.acquiring = acquiring;
    s.sign = sign;
    s.tip = tip;
    s.blank = blank;
    s.lines = lines;
    s.lines_locked = lines_locked;
    s.fields = fields_seen;
    s.frames = frames_out;
    s.acquisitions = acquisitions;
    s.losses = losses;
    s.line_us = T / fs * 1e6;
    s.lock_recent = recent_n ? static_cast<float>(recent_locked) / recent_n : 0.f;
    s.std_name = std_->name;
    s.rephases = rephases;
    s.colour = colour_on;
    s.burst_err = c_err;
    s.clicks_repaired = spark.clicks_repaired;
    s.lines_mended = spark.lines_mended;
    return s;
  }

  // A block of discriminator output, MHz; bad: the front end's click marks for it (sparkle repair), or null.
  // Each piece of up to kPiece samples goes through in two passes. First the sync path (a 1 MHz low-pass and a
  // 3-point median), whose filter never changes, over the whole piece. Then the pulse tracking, a sample at a
  // time (track), with the luma and colour paths following behind (filter_to): they are brought up to the
  // sample in hand before anything that reads their rings or changes their switches (an hsync, a line that
  // ends, acquisition), and at the end of the piece. So their filters run in tight loops with the state in
  // registers (sample by sample, through everything, it went through memory for every filter and every
  // sample: a third of the decode thread on a Pixel 8 Pro), and every sample still gets the same arithmetic,
  // in the same order, with the switches as they were when it came.
  void push_block(const float *d, const uint8_t *bad, size_t cnt) {
    float mb[kPiece];
    while (cnt > 0) {
      const size_t pn = std::min(cnt, kPiece);
      piece_d = d;
      piece_bad = bad;
      piece_n0 = n;
      {
        Biquad ls = lp_sync;
        float a = med_a, b = med_b;
        for (size_t k = 0; k < pn; k++) {
          const float s = ls.run(d[k]);
          mb[k] = std::max(std::min(a, b), std::min(std::max(a, b), s));
          b = a;
          a = s;
        }
        lp_sync = ls;
        med_a = a;
        med_b = b;
      }
      for (size_t k = 0; k < pn;) {
        k += quiet_run(mb + k, pn - k);
        if (k < pn) track(mb[k++]);
      }
      filter_to(n);
      d += pn;
      if (bad) bad += pn;
      cnt -= pn;
    }
  }

private:
  double fs;
  FrameSink sink;
  DecoderOpts o;
  const VideoStd *std_ = &kNtsc;
  float sign = +1, tip = -1, blank = 0, thr = -0.5f;
  uint64_t n = 0;  // samples taken (the pulse tracking's count)
  // push_block: the piece in hand (piece_n0: n at its first sample)
  static constexpr size_t kPiece = 512;
  const float *piece_d = nullptr;
  const uint8_t *piece_bad = nullptr;
  uint64_t piece_n0 = 0;
  // overdue_at: the line_start and T it was worked out for
  bool od_valid = false;
  double od_ls = 0, od_T = 0;
  uint64_t od_at = UINT64_MAX;
  SampleRings R;  // what the filters below have made of the samples so far (R.n <= n: filter_to)
  LineDrawer drawer;
  // NTSC chroma comb (DecoderOpts::comb; LineDrawer::draw does the combing): each line is drawn one line late,
  // once the line below is in, and combed with both neighbours (centred, so chroma stays on luma's line).
  // Luma keeps the notch. PAL (its subcarrier pattern repeats over two lines) is not combed.
  static constexpr float kCombMaxErr = 0.35f;  // rad: a burst further off means the stream jumped
  bool comb = false;
  double comb_w = 0;           // the mixing oscillator's frequency, rad per sample
  uint64_t comb_epoch = 0;     // bumped where the sample stream may have jumped: lines across it don't mix
  LineInfo comb_up, comb_mid;  // the two lines before the current one
  float burst_e = 0;           // |phase error| of the last burst against the PLL's prediction
  // kepyur: colour. A free-running oscillator at the nominal subcarrier mixes chroma to baseband; a
  // per-line PLL on the burst tracks the transmitter's subcarrier phase (and its small frequency error);
  // PAL: the burst's +-45 degree swing gives the V switch, and adjacent lines are averaged (PAL-D).
  Biquad luma_notch, cl_r1, cl_r2, cl_i1, cl_i2;
  cf lo{1, 0}, lo_step{1, 0};
  double c_phase = 0, c_dphi = 0;  // burst reference phase (rad) and its drift per line
  int64_t c_last_line = -1;
  float c_mag = 0, c_err = 1;      // burst amplitude (MHz), mean |phase error| (rad)
  bool colour_on = false;
  int c_good = 0;
  float med_a = 0, med_b = 0;
  Biquad lp_luma, lp_luma2, lp_sync;
  // Luma comb (DecoderOpts::luma_filter, set_luma_comb) and adaptive comb (set_adaptive_comb): see push().
  // The switches are read once per line (refresh_switches), not per sample.
  Biquad lp_wide, bp_chroma, lp_vert, luma_notch_w;
  bool luma_comb = false, adaptive = false, repair = false;
  int adaptive_lines = 0;  // lines since the adaptive comb came on: its filters and ring need one to fill
  uint64_t prev_back = 1;  // the line before: ring sample n - prev_back, and prev_fr of the way to the next one
  float prev_fr = 0, inv_depth = 1;  // inv_depth: 1 / sync depth
  float vdiff = 0;
  int chroma_quiet = 0;  // samples in a row with the colour faded out completely
  double cdel_comb = 0;
  SparkleRepair spark;  // sparkle repair (set_repair), on the click marks in R.bad
  bool low = false, longrun = false;
  uint64_t low_start = 0;
  double T = 0, line_start = -1;
  uint64_t lines = 0, lines_locked = 0, line_idx = 0, acquisitions = 0, losses = 0;
  uint64_t line_low = 0;
  bool line_has_longrun = false;
  uint64_t line_longrun_start = 0;
  bool prev_line_vsync = false;
  std::vector<float> acq;
  size_t acq_n = 0;
  bool acquiring = true;
  std::vector<uint8_t> fieldA, fieldB, frame;
  Deinterlacer deint;
  bool haveA = false, haveB = false;
  int cur_field = -1;
  int64_t field_line0 = 0;
  uint64_t frames_out = 0, fields_seen = 0;
  std::vector<uint32_t> hist;
  double hlo = -8, hhi = 8;
  int hdec = 0;
  uint64_t hist_count = 0;
  // kepyur: sync-gated levels: sync tip from the middle of each hsync pulse, blanking from the back porch,
  // measured on locked active lines and smoothed over ~64 lines. Picture content cannot move them (the
  // spike's histogram peaks could, and the threshold wandered with the scene).
  uint64_t gated_lines = 0;
  // kepyur: re-phase instead of re-acquire. After a jump in the signal (stream restart, a dropped bank)
  // the hsyncs keep coming, one line apart, but outside the PLL's window. A run of them on a steady grid
  // while the PLL freewheels moves the line grid onto them within a few lines.
  int miss_run = 0, stray_run = 0;
  double last_stray = -1;
  uint64_t rephases = 0;
  // kepyur: lock quality over a sliding window of lines (the spike counted since acquisition)
  static constexpr int kRecent = 1024;
  uint8_t recent[kRecent] = {};
  int recent_pos = 0, recent_n = 0, recent_locked = 0;

  // The pulse tracking for the next sample (m: its sync-path value). The luma and colour paths are behind it
  // (see push_block): filter_to(n) brings them up to and including this sample before anything that uses them.
  inline void track(float m) {
    const uint64_t now = n++;
    if (acquiring) {
      if (acq_n < acq.size()) {
        acq[acq_n++] = m;
      } else {
        filter_to(n);
        acquire();
      }
      return;
    }
    if (gated_lines < 64) track_levels(m);  // until the sync-gated measurement has settled
    const float w = sign * m;
    if (!low && w < thr) {
      low = true;
      low_start = now;
      longrun = false;
    } else if (low) {
      if (!longrun && now - low_start > 0.3 * T) {
        longrun = true;
        if (!line_has_longrun) {
          line_has_longrun = true;
          line_longrun_start = low_start;
        }
      }
      if (w >= thr) {
        low = false;
        const double wid = (now - low_start) / fs * 1e6;
        if (wid > 3.0 && wid < 6.5) {
          filter_to(n);
          on_hsync(static_cast<double>(low_start));
        }
      }
      line_low++;
    }
    // freewheel: line overdue by > 8 % with no edge
    if (now > overdue_at()) {
      filter_to(n);
      const double ls = line_start;
      miss_run++;
      count_line(false);
      render_line(ls, false);
      close_line(ls + T);
      check_lost();
    }
  }

  // How many of the next samples (sync-path values m[0..cnt)) track() would do nothing with: locked on (not
  // acquiring, the levels settled), between pulses, still above the threshold, and the line not yet overdue.
  // They are passed over here, n moved on past them; the first that would do something is left to track().
  // ~90 % of the samples, looked at eight at a time (a vector compare), where track() took each on its own.
  size_t quiet_run(const float *m, size_t cnt) {
    if (acquiring || gated_lines < 64 || low) return 0;
    const uint64_t od = overdue_at();
    if (od < n) return 0;  // the next sample is overdue
    if (od - n < cnt) cnt = static_cast<size_t>(od - n) + 1;  // samples up to and including od are not
    const float s = sign, t = thr;
    size_t j = 0;
    for (; j + 8 <= cnt; j += 8) {
      bool hit = false;
      for (int q = 0; q < 8; q++) hit |= s * m[j + q] < t;
      if (hit) break;
    }
    while (j < cnt && !(s * m[j] < t)) j++;
    n += j;
    return j;
  }

  // The first sample past which the line is overdue (line_start + 8 % of T; none while there is no line), as
  // an integer, worked out again only when line_start or T has changed: for a whole number now, now > x exactly
  // when now > floor(x). Per sample in doubles it cost ~10 % of the decode thread on a phone.
  uint64_t overdue_at() {
    if (!od_valid || line_start != od_ls || T != od_T) {
      od_valid = true;
      od_ls = line_start;
      od_T = T;
      const double x = line_start + T * 1.08;
      od_at = line_start < 0 || !(x < 1.8e19) ? UINT64_MAX : static_cast<uint64_t>(std::floor(x));
    }
    return od_at;
  }

  // The luma and colour paths (and the click marks' ring) up to sample `to`, not included (see push_block).
  void filter_to(uint64_t to) {
    if (R.n >= to) return;
    const size_t k0 = static_cast<size_t>(R.n - piece_n0), cnt = static_cast<size_t>(to - R.n);
    const uint8_t *bd = piece_bad ? piece_bad + k0 : nullptr;
    if (luma_comb) filter_comb(piece_d + k0, bd, cnt);
    else filter_notch(piece_d + k0, bd, cnt);
    R.n = to;
  }

  // Luma through the 3 MHz low-pass (two sections) and the subcarrier notch. Colour (here and in filter_comb):
  // mixed down by the oscillator, whose phase belongs to the absolute sample index (burst and picture share
  // it), then a 4th-order low-pass for each of U and V. The two sections of luma, U and V go side by side
  // (Biquad4: lanes 0-2, the first sections in s1 and the second in s2, so s1's output is s2's input lane for
  // lane), the notch on its own. All in locals for the loop, so that their state stays in registers.
  void filter_notch(const float *d, const uint8_t *bd, size_t cnt) {
    Biquad4 s1, s2;
    s1.load(0, lp_luma);
    s1.load(1, cl_r1);
    s1.load(2, cl_i1);
    s2.load(0, lp_luma2);
    s2.load(1, cl_r2);
    s2.load(2, cl_i2);
    Biquad nt = luma_notch;
    cf osc = lo;
    const cf step = lo_step;
    const bool col = o.colour;
    const size_t mask = R.mask;
    float *lr = R.luma.data();
    cf *cr = R.chroma.data();
    uint8_t *br = R.bad.data();
    uint64_t idx = R.n;
    for (size_t k = 0; k < cnt; k++, idx++) {
      const size_t at = idx & mask;
      const float x = d[k];
      br[at] = bd ? bd[k] : 0;
      cf c{};
      if (col) {
        c = x * std::conj(osc);
        osc *= step;
        if ((idx & 4095) == 0) osc /= std::abs(osc);
      }
      const v4f y = s2.run(s1.run(v4f{x, c.real(), c.imag(), 0.f}));
      lr[at] = nt.run(y[0]);
      if (col) cr[at] = cf(y[1], y[2]);
    }
    s1.store(0, lp_luma);
    s2.store(0, lp_luma2);
    luma_notch = nt;
    if (col) {  // without colour its filters stay as they were (only ever fed zeros here)
      s1.store(1, cl_r1);
      s1.store(2, cl_i1);
      s2.store(1, cl_r2);
      s2.store(2, cl_i2);
      lo = osc;
    }
  }

  // The NTSC luma comb, and the adaptive comb on top of it (refresh_switches: the switches are read once per
  // line, so they hold for a whole call). Locals as in filter_notch, and side by side the same way: after the
  // wide low-pass, its three filters (the colour band of the line difference, and the adaptive comb's low band
  // and notch) in s2's lanes 0-2, then U and V's two sections in c1 and c2.
  void filter_comb(const float *d, const uint8_t *bd, size_t cnt) {
    Biquad wide = lp_wide;
    Biquad4 s2;
    s2.load(0, bp_chroma);
    s2.load(1, lp_vert);
    s2.load(2, luma_notch_w);
    Biquad2 c1, c2;
    c1.load(0, cl_r1);
    c1.load(1, cl_i1);
    c2.load(0, cl_r2);
    c2.load(1, cl_i2);
    cf osc = lo;
    const cf step = lo_step;
    const bool col = o.colour, adapt = adaptive, adapt_full = adaptive_lines > 1, comb3 = comb;
    const uint64_t back = prev_back;
    const float fr = prev_fr, idepth = inv_depth;
    float vd = vdiff;
    int quiet = chroma_quiet;
    const size_t mask = R.mask;
    float *lr = R.luma.data(), *fdr = R.fade.data();
    cf *cr = R.chroma.data();
    uint8_t *br = R.bad.data();
    CombTap *ring = R.comb.data();
    uint64_t idx = R.n;
    for (size_t k = 0; k < cnt; k++, idx++) {
      const size_t at = idx & mask;
      const float x = d[k];
      br[at] = bd ? bd[k] : 0;
      float csrc = x;  // what the colour is demodulated from
      // NTSC colour flips phase from one line to the next. Half the difference with the same point one line
      // earlier (T samples, tracked) is colour plus vertical detail; band-passing it at the subcarrier leaves
      // the colour, and the brightness is the wide signal minus that. No notch, so no ringing.
      const float w = wide.run(x);
      CombTap &now_tap = ring[at];
      now_tap.wide = w;
      now_tap.d = x;
      // the same point on the line before: T samples back, between ring samples q0 and q0 + 1 (prev_back and
      // prev_fr are worked out once per line, see refresh_switches)
      const bool have_prev = idx > back;
      const CombTap &p0 = ring[(idx - back) & mask], &p1 = ring[(idx - back + 1) & mask];
      const v4f y2 = s2.run(v4f{0.5f * (w - (have_prev ? p0.wide + (p1.wide - p0.wide) * fr : w)), w, w, 0.f});
      float luma = w - y2[0];
      if (adapt) {
        // Adaptive comb: how different the line before is, below the colour band (so colour itself, which
        // flips from line to line, doesn't count), in sync depths; past ~0.25 fall back to the notch, fully by
        // ~0.6.
        const float lf = y2[1];
        now_tap.low = lf;
        const float ln = y2[2];
        if (have_prev && adapt_full) {
          const float dv = std::fabs(lf - (p0.low + (p1.low - p0.low) * fr));
          vd += (dv - vd) * 0.35f;  // ~3 samples: a whole edge, not one noisy sample
          const float fade = std::clamp((vd * idepth - 0.25f) * (1.f / 0.35f), 0.f, 1.f);
          luma += (ln - luma) * fade;
          // And the colour combed too: half the difference with the line before keeps the colour (it flips
          // from line to line) and cancels brightness detail that continues down the picture (the strokes of
          // OSD characters); what is left at a sharp edge is faded out. With the 3-line chroma comb on, that
          // comb does the cancelling (centred on this line), and the fade is only noted here: draw_line
          // applies it where the comb found no line to cancel with. Fading here took the colour off the
          // line under every horizontal edge, which the comb draws well from the line below, and off part of
          // the next line, which is combed with that one.
          if (comb3) {
            fdr[at] = fade;
          } else {
            csrc = 0.5f * (x - (p0.d + (p1.d - p0.d) * fr));
            csrc *= 1.f - fade;
            // While the fade is complete (a whole line under a horizontal edge; the first picture line of every
            // field too: top_line() works that line's colour out again afterwards, with filters of its own,
            // but this one still runs through the line here) the chroma low-pass is fed exact zeros, and
            // within a few hundred samples its state decays into denormal numbers, which x86 computes some
            // hundred times slower: 9 % of the decode time on the bench's test picture. Looked at every 64
            // samples, it never gets that far.
            if (fade < 1.f) {
              quiet = 0;
            } else if ((++quiet & 63) == 0) {
              c1.settle(0);
              c1.settle(1);
              c2.settle(0);
              c2.settle(1);
            }
          }
        }
      }
      lr[at] = luma;
      if (col) {
        const cf c = csrc * std::conj(osc);
        osc *= step;
        if ((idx & 4095) == 0) osc /= std::abs(osc);
        const v2f yc = c2.run(c1.run(v2f{c.real(), c.imag()}));
        cr[at] = cf(yc[0], yc[1]);
      }
    }
    lp_wide = wide;
    s2.store(0, bp_chroma);
    if (adapt) {  // without the adaptive comb its filters stay as they were
      s2.store(1, lp_vert);
      s2.store(2, luma_notch_w);
    }
    if (col) {
      c1.store(0, cl_r1);
      c1.store(1, cl_i1);
      c2.store(0, cl_r2);
      c2.store(1, cl_i2);
      lo = osc;
    }
    vdiff = vd;
    chroma_quiet = quiet;
  }

  void set_std(const VideoStd &s) {
    std_ = &s;
    T = s.T_us * 1e-6 * fs;
    fieldA.assign(static_cast<size_t>(o.width) * s.active * 4, 0);
    fieldB = fieldA;
    frame.assign(static_cast<size_t>(o.width) * s.active * 2 * 4, 0);
    const double fsc = &s == &kPal ? 4.43361875e6 : 315e6 / 88;
    const double a = 2 * M_PI * fsc / fs;
    lo_step = cf(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
    luma_notch.notch(fsc, fs, 1.5);  // the subcarrier out of the picture (also the B/W herringbone)
    luma_notch_w.notch(fsc, fs, 1.5);  // adaptive comb: the same, on the comb's wide signal
    bp_chroma.bandpass(fsc, fs, 1.0);  // luma comb: the colour band of the line difference
    refresh_switches();
    comb = o.comb && &s != &kPal;
    comb_w = a;
    comb_epoch++;
    drawer.set_std(s, a);
    colour_on = false;
    c_good = 0;
    c_last_line = -1;
  }

  // The process-wide picture switches, as of now. PAL's colour repeats over two lines, not one: it keeps the
  // notch.
  void refresh_switches() {
    luma_comb = (o.luma_filter >= 0 ? o.luma_filter == 1 : luma_comb_on()) && std_ != &kPal;
    adaptive = luma_comb && adaptive_comb_on();
    adaptive_lines = adaptive ? std::min(adaptive_lines + 1, 2) : 0;
    repair = repair_on();
    // n - T = (n - ceil(T)) + (ceil(T) - T)
    const double up = std::ceil(T);
    prev_back = static_cast<uint64_t>(std::max(up, 1.0));
    prev_fr = static_cast<float>(up - T);
    inv_depth = 1.f / std::max(blank - tip, 0.2f);
  }

  void count_line(bool locked) {
    lines++;
    if (locked) lines_locked++;
    if (recent_n == kRecent) recent_locked -= recent[recent_pos];
    else recent_n++;
    recent[recent_pos] = locked;
    recent_locked += locked;
    recent_pos = (recent_pos + 1) % kRecent;
  }

  // kepyur: lost when fewer than 40 % of the last ~2 fields' lines found their hsync (a healthy picture
  // locks ~90 %: only the vertical interval freewheels). The spike used the ratio since acquisition.
  void check_lost() {
    const int window = std::min(kRecent, 2 * (std_ == &kPal ? 312 : 262));
    if (recent_n < window) return;
    if (recent_locked * 10 < recent_n * 4) {
      losses++;
      KP_LOGI("video: lost lock (%d/%d recent lines), re-acquiring", recent_locked, recent_n);
      restart_acquisition();
    }
  }

  void restart_acquisition() {
    acquiring = true;
    acq_n = 0;
    lines = lines_locked = 0;
    cur_field = -1;
    haveA = haveB = false;
    recent_n = recent_locked = recent_pos = 0;
    line_start = -1;
    gated_lines = 0;
    miss_run = stray_run = 0;
    last_stray = -1;
    comb_epoch++;
  }

  void track_levels(float m) {
    // level tracking histogram (every 8th sample), refreshed every ~40 ms
    if ((++hdec & 7) != 0) return;
    const float v = sign * m;
    const int nb = static_cast<int>(hist.size());
    const int b = static_cast<int>((v - hlo) / (hhi - hlo) * nb);
    if (b >= 0 && b < nb) {
      hist[b]++;
      hist_count++;
    }
    if (hist_count >= static_cast<uint64_t>(fs * 0.04 / 8)) {
      float t, bl;
      if (levels_from_hist(static_cast<uint32_t>(hist_count), t, bl)) {
        tip = 0.8f * tip + 0.2f * t;
        blank = 0.8f * blank + 0.2f * bl;
        thr = 0.5f * (tip + blank);
      }
      std::fill(hist.begin(), hist.end(), 0);
      hist_count = 0;
    }
  }

  bool levels_from(const std::vector<float> &buf, size_t cnt, float sgn, float &tip_o, float &blank_o) {
    std::fill(hist.begin(), hist.end(), 0);
    const int nb = static_cast<int>(hist.size());
    uint32_t total = 0;
    for (size_t i = 0; i < cnt; i += 4) {
      const float v = sgn * buf[i];
      const int b = static_cast<int>((v - hlo) / (hhi - hlo) * nb);
      if (b < 0 || b >= nb) continue;
      hist[b]++;
      total++;
    }
    const bool ok = levels_from_hist(total, tip_o, blank_o);
    std::fill(hist.begin(), hist.end(), 0);
    return ok;
  }

  bool levels_from_hist(uint32_t total, float &tip_o, float &blank_o) {
    const int nb = static_cast<int>(hist.size());
    if (total < 1000) return false;
    uint32_t c = 0;
    int p0 = 0, p1 = nb - 1;
    for (int i = 0; i < nb; i++) {
      c += hist[i];
      if (c >= total * 0.005) { p0 = i; break; }
    }
    c = 0;
    for (int i = nb - 1; i >= 0; i--) {
      c += hist[i];
      if (c >= total * 0.005) { p1 = i; break; }
    }
    if (p1 - p0 < 20) return false;
    std::vector<float> sm(nb, 0);
    for (int i = 0; i < nb; i++) {
      float s = 0;
      for (int k = -4; k <= 4; k++) {
        const int j = i + k;
        if (j >= 0 && j < nb) s += hist[j];
      }
      sm[i] = s / 9;
    }
    float mx = 0;
    for (int i = p0; i <= p1; i++) mx = std::max(mx, sm[i]);
    std::vector<int> peaks;
    const int dist = std::max(3, (p1 - p0) / 40);
    for (int i = p0; i <= p1; i++) {
      if (sm[i] < mx * 0.02) continue;
      bool ok = true;
      for (int k = -dist; k <= dist && ok; k++) {
        const int j = i + k;
        if (k != 0 && j >= 0 && j < nb && sm[j] > sm[i]) ok = false;
      }
      if (ok && (peaks.empty() || i - peaks.back() > dist)) peaks.push_back(i);
    }
    if (peaks.size() < 2) return false;
    const float bw = static_cast<float>((hhi - hlo) / nb);
    const float tipv = static_cast<float>(hlo) + (peaks[0] + 0.5f) * bw;
    const float span = (p1 - p0) * bw;
    for (size_t k = 1; k < peaks.size(); k++) {
      const float v = static_cast<float>(hlo) + (peaks[k] + 0.5f) * bw;
      if (v - tipv > 0.12f * span) {
        tip_o = tipv;
        blank_o = v;
        return true;
      }
    }
    return false;
  }

  // acquisition: pick polarity + levels + line period from a 60 ms buffer
  void acquire() {
    float best = 0, bt = 0, bb = 0, bs = 0;
    for (float sgn : {+1.f, -1.f}) {
      if (o.force_invert && sgn > 0) continue;
      float t, b;
      if (!levels_from(acq, acq_n, sgn, t, b)) continue;
      const float th = 0.5f * (t + b);
      std::vector<double> edges;
      bool lo = false;
      size_t ls = 0;
      for (size_t i = 0; i < acq_n; i++) {
        const float v = sgn * acq[i];
        if (!lo && v < th) {
          lo = true;
          ls = i;
        } else if (lo && v >= th) {
          lo = false;
          const double w = (i - ls) / fs * 1e6;
          if (w > 3.0 && w < 6.5) edges.push_back(static_cast<double>(ls));
        }
      }
      if (edges.size() < 30) continue;
      std::vector<double> good;
      for (size_t k = 1; k < edges.size(); k++) {
        const double iv = (edges[k] - edges[k - 1]) / fs * 1e6;
        if (iv > 62.5 && iv < 65.0) good.push_back(iv);
      }
      const float score = static_cast<float>(good.size()) / (edges.size() - 1);
      KP_LOGD("video acq: sign %+.0f tip %.2f blank %.2f, %zu pulses, score %.2f", sgn, t, b, edges.size(), score);
      if (score > best && !good.empty()) {
        best = score;
        bs = sgn;
        bt = t;
        bb = b;
        std::sort(good.begin(), good.end());
        const double Tus = good[good.size() / 2];
        if (!o.std_force) {
          const VideoStd &s = std::fabs(Tus - kNtsc.T_us) < std::fabs(Tus - kPal.T_us) ? kNtsc : kPal;
          if (&s != std_) set_std(s);
        }
        T = Tus * 1e-6 * fs;
      }
    }
    if (best > 0.3f) {
      sign = bs;
      tip = bt;
      blank = bb;
      thr = 0.5f * (tip + blank);
      acquiring = false;
      line_start = -1;
      acquisitions++;
      KP_LOGI("video: acquired, sync %s end, tip %.2f blank %.2f MHz, line %.3f us -> %s, score %.2f",
              sign > 0 ? "low" : "high", tip, blank, T / fs * 1e6, std_->name, best);
    }
    acq_n = 0;
  }

  // Burst: chroma over the burst window (NTSC 5.3-7.8 us, PAL 5.6-7.85 us after the hsync edge; the
  // chroma filter delays it ~0.35 us). Updates the reference phase; returns false when there is none.
  bool burst(double ls, bool &pal_v) {
    const double us = 1e-6 * fs;
    const int64_t i0 = static_cast<int64_t>(ls + 6.0 * us), i1 = static_cast<int64_t>(ls + 7.8 * us);
    if (!R.has(i0, i1)) return false;
    cf b{};
    for (int64_t i = i0; i < i1; i++) b += R.chroma[i & R.mask];
    b /= static_cast<float>(i1 - i0);
    const float mag = std::abs(b);
    const int64_t li = static_cast<int64_t>(line_idx);
    const double pred = c_last_line < 0 ? std::arg(b) : c_phase + c_dphi * (li - c_last_line);
    double e = std::arg(b * std::polar(1.f, static_cast<float>(-pred)));
    pal_v = false;
    if (std_ == &kPal) {
      // The burst sits 45 degrees either side of the reference: at baseband a normal line's burst (135 deg
      // in U/V) lands at -45, a V-switched line's (225 deg) at +45.
      pal_v = e > 0;
      e += pal_v ? -M_PI / 4 : M_PI / 4;
    }
    burst_e = static_cast<float>(std::fabs(e));
    if (c_last_line < 0) {
      c_phase = pred + e;
      c_dphi = 0;
    } else {
      c_phase = pred + 0.2 * e;
      c_dphi += 0.01 * e / std::max<int64_t>(1, li - c_last_line);
    }
    c_phase = std::remainder(c_phase, 2 * M_PI);
    c_last_line = li;
    c_mag += 0.05f * (mag - c_mag);
    c_err += 0.05f * (static_cast<float>(std::fabs(e)) - c_err);
    // colour killer, with hysteresis: a burst near the expected size (~1/4 of the sync depth) and steady
    const bool ok = c_mag > 0.08f * (blank - tip) && c_err < 0.5f;
    c_good = ok ? std::min(c_good + 1, 60) : std::max(c_good - 2, 0);
    if (!colour_on && c_good >= 40) colour_on = true;
    if (colour_on && c_good == 0) colour_on = false;
    return colour_on;
  }

  void render_line(double ls, bool locked_line) {
    if (cur_field < 0) return;
    const int64_t k = static_cast<int64_t>(line_idx) - field_line0 - std_->skip;
    if (comb) {
      comb_line(ls, k, locked_line);
      return;
    }
    if (k < 0 || k >= std_->active) return;
    if (locked_line) measure_levels(ls);
    if (luma_comb) top_line(ls, k);
    const bool hit = repair && patch_line(ls);
    bool pal_v = false;
    const bool col = o.colour && locked_line && burst(ls, pal_v);
    LineInfo l;
    l.ls = ls;
    l.k = k;
    l.col = col;
    l.pal_v = pal_v;
    l.phase = c_phase;
    l.mag = c_mag;
    draw_line(l, false, false);
    if (repair) mend_rows(k, hit);
  }

  // NTSC comb: line k is measured now (levels, burst) and the previous one, k - 1, is drawn, combed with
  // k - 2 and k. A neighbour is used only if both lines are locked and in colour, in the same field and on
  // the same side of any jump in the stream (comb_epoch, and a burst on its predicted phase).
  void comb_line(double ls, int64_t k, bool locked_line) {
    LineInfo cur;
    cur.ls = ls;
    cur.k = k;
    cur.field = cur_field;
    cur.epoch = comb_epoch;
    if (k >= 0 && k < std_->active) {
      if (locked_line) measure_levels(ls);
      if (luma_comb) top_line(ls, k);
      cur.hit = repair && patch_line(ls);  // before this line is anyone's neighbour
      bool pal_v = false;
      cur.col = o.colour && locked_line && burst(ls, pal_v);
      cur.ok = cur.col && burst_e < kCombMaxErr;
      cur.phase = c_phase;
      cur.mag = c_mag;
      cur.fade = adaptive && adaptive_lines > 1;  // as push() had them over this line's picture
    }
    const LineInfo &m = comb_mid;
    if (m.field == cur_field && m.k == k - 1 && m.k >= 0 && m.k < std_->active) {
      const auto pair = [&](const LineInfo &x, int64_t dk) {
        return m.ok && x.ok && x.field == m.field && x.k == m.k + dk && x.epoch == m.epoch;
      };
      draw_line(m, pair(comb_up, -1), pair(cur, 1));
      if (repair) mend_rows(m.k, m.hit);
    }
    comb_up = comb_mid;
    comb_mid = cur;
  }

  // The line into its row of the field being built (line_draw.h). up / dn: comb its chroma with the line above /
  // below (comb_line).
  void draw_line(const LineInfo &l, bool up, bool dn) {
    uint32_t *row = reinterpret_cast<uint32_t *>((cur_field == 0 ? fieldA : fieldB).data()) + static_cast<size_t>(l.k) * o.width;
    const double cdel = luma_comb ? cdel_comb : 0.21e-6 * fs;  // chroma filter delay minus luma filter delay
    drawer.draw(row, R, l, up, dn, DrawParams{sign, tip, blank, T, cdel});
  }

  // The top of the picture with the luma comb. The first picture line of a field has blanking above it, not
  // picture, and the luma comb works each line out against the line before. Against blanking it leaves half of
  // the colour in the brightness as dots, and the adaptive comb, which sees a line that differs all the way
  // along, fades the colour out as it does under any horizontal edge: the top rows of every frame came out grey.
  // There is nothing above that line to keep rainbows off, so its picture is worked out again here, from what
  // the comb's ring holds of it, the way the decoder does without the luma comb: brightness through the notch
  // (which the adaptive comb falls back to there anyway), colour straight from the signal, not faded. Only the
  // picture's own stretch of that one line is replaced and nothing in push() changes: levels, burst and every
  // other line come out exactly as they did.
  // Which line that is depends on the camera and on where the field was found to start: the first drawn row
  // (k = 0) or one of the next few, with blank rows above it. So it is looked for: the first of rows 0..kTopRows
  // that has picture (it averages kTopPicture sync depths above blanking or more: the level from which the
  // adaptive comb starts to see a difference) while every row above it, and the line before row 0, averages
  // within kTopBlank sync depths of blanking. The second limit is the tighter one on purpose: black picture
  // sits 7.5 IRE (0.19 sync depths) above blanking, and a row under black rows of picture is a line inside the
  // picture, which keeps the comb's fade. (A camera that sends black at the blanking level cannot be told
  // apart this way: under up to kTopRows black rows at the top of its picture, the first brighter row is taken
  // for the top line and drawn as without the luma comb.) A field that begins with
  // the standard's half line has it just above row 0 here: then row 0 has blanking above its left half only,
  // and only that half is done (to the middle, where the half line's picture has begun and the comb is right
  // again). A line further down that follows a dark line is not touched: there the fade is Play RX's trade.
  static constexpr int kTopRows = 4;
  static constexpr float kTopPicture = 0.25f, kTopBlank = 0.10f;
  void top_line(double ls, int64_t k) {
    if (k > kTopRows) return;
    const double a0 = std_->a0_us * 1e-6 * fs, aw = std_->aw_us * 1e-6 * fs;
    // no picture over the part [x0, x1) of the width of the line that starts at l?
    const auto blanking = [&](double l, double x0, double x1, float limit = kTopBlank) {
      float m;
      return ring_mean(l + a0 + aw * x0, l + a0 + aw * x1, m) && (m - blank) * inv_depth < limit;
    };
    if (blanking(ls, 0, 1, kTopPicture)) return;
    for (int64_t up = 2; up <= k + 1; up++)  // the rows above the line before, and the line before row 0
      if (!blanking(ls - static_cast<double>(up) * T, 0, 1)) return;
    const bool whole = blanking(ls - T, 0, 1);
    if (!whole && (k > 0 || !blanking(ls - T, 0, 0.4))) return;
    // The whole line: from the first good point patch_line may take (8 samples ahead of the picture) to its
    // last one in luma (20 after it; those of its chroma patches further on lie in blanking, where there is no
    // colour either way). The left half: to the middle, the last kBlend samples blended into what push() made.
    // The filters start kWarm samples ahead, in the sync pulse, and have settled long before the picture.
    constexpr int kWarm = 128, kBlend = 32;
    const int64_t s0 = static_cast<int64_t>(ls + a0) - 8;
    const int64_t s1 = whole ? static_cast<int64_t>(ls + a0 + aw) + 21 : static_cast<int64_t>(ls + a0 + 0.5 * aw);
    const int64_t back = static_cast<int64_t>(n) - (s0 - kWarm);
    if (!R.has(s0 - kWarm, s1, 32)) return;
    const auto fresh = [](Biquad b) {
      b.z1 = b.z2 = 0;
      return b;
    };
    Biquad nt = fresh(luma_notch_w), r1 = fresh(cl_r1), r2 = fresh(cl_r2), i1 = fresh(cl_i1), i2 = fresh(cl_i2);
    // Colour: only the adaptive comb on its own takes it from the line difference and fades it (push). Otherwise
    // R.chroma already has it straight from the signal; with the 3-line comb the fade is only noted, and
    // draw_line uses it where that comb finds no matching line below either, which is left as it is. The
    // oscillator turns by comb_w per sample: back to where it was at the first sample.
    const bool chroma = o.colour && adaptive && !comb;
    const std::complex<double> was = std::polar(1.0, -comb_w * static_cast<double>(back));
    cf osc = lo * cf(static_cast<float>(was.real()), static_cast<float>(was.imag()));
    // push()'s line difference reads the line before between two samples, which takes 3 % off the subcarrier
    // and turns it by half a degree. The burst went through it and the picture's colour is measured against
    // the burst, so this line's colour gets the same, or it would come out that much more saturated than the
    // rest: half of (1 - the subcarrier as read prev_back samples back, prev_fr of the way to the next sample).
    const std::complex<double> prev = std::polar(1.0, -comb_w * static_cast<double>(prev_back)) *
                                      ((1.0 - prev_fr) + static_cast<double>(prev_fr) * std::polar(1.0, comb_w));
    const cf gain(static_cast<float>(0.5 * (1.0 - prev.real())), static_cast<float>(-0.5 * prev.imag()));
    for (int64_t i = s0 - kWarm; i < s1; i++) {
      const CombTap &t = R.comb[i & R.mask];
      const float y = nt.run(t.wide);
      const float mix = whole ? 1.f : std::min(1.f, static_cast<float>(s1 - i) * (1.f / kBlend));
      if (i >= s0) R.luma[i & R.mask] += (y - R.luma[i & R.mask]) * mix;
      if (!chroma) continue;
      const cf c = t.d * std::conj(osc);
      osc *= lo_step;
      const cf uv(r2.run(r1.run(c.real())), i2.run(i1.run(c.imag())));
      if (i >= s0) R.chroma[i & R.mask] += (uv * gain - R.chroma[i & R.mask]) * mix;
    }
  }

  // Sparkle repair (sparkle.h): the picture of the line that starts at ls patched in the rings, before it is
  // drawn; true when it is still badly hit, which mend_rows is told once the row (k) is drawn.
  bool patch_line(double ls) {
    const double a0 = std_->a0_us * 1e-6 * fs, aw = std_->aw_us * 1e-6 * fs;
    return spark.patch_line(R, ls + a0, aw, T, luma_comb, adaptive && !comb);
  }
  void mend_rows(int64_t k, bool hit) {
    spark.mend_rows((cur_field == 0 ? fieldA : fieldB).data(), o.width, fields_seen, cur_field, k, hit);
  }

  // The mean of R.luma over [from, to) into out; false when that stretch is not in the ring. (Not a NaN
  // return: this file is built with -ffast-math, under which a test for NaN may be compiled away.)
  bool ring_mean(double from, double to, float &out) {
    const int64_t i0 = static_cast<int64_t>(from), i1 = static_cast<int64_t>(to);
    if (!R.has(i0, i1)) return false;
    float s = 0;
    for (int64_t i = i0; i < i1; i++) s += R.luma[i & R.mask];
    out = sign * s / static_cast<float>(i1 - i0);
    return true;
  }

  void measure_levels(double ls) {
    const double us = 1e-6 * fs;
    float t, b;
    if (!ring_mean(ls + 1.2 * us, ls + 3.6 * us, t)) return;  // hsync is 4.7 us
    if (!ring_mean(ls + 5.8 * us, ls + 8.8 * us, b)) return;  // back porch (burst averages out)
    if (b - t < 0.05f) return;
    const float a = gated_lines < 64 ? 0.1f : 1.f / 64;
    tip += a * (t - tip);
    blank += a * (b - blank);
    thr = 0.5f * (tip + blank);
    gated_lines++;
  }

  // A frame to the sink: from one field with its lines interpolated (smooth lines), or from two (the same one
  // twice: each line doubled).
  void emit_field(const std::vector<uint8_t> &f, bool upper) {
    deint.smooth(f.data(), upper, o.width, std_->active, frame.data());
    emit_frame();
  }
  void emit(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
    weave(a.data(), b.data(), o.width, std_->active, frame.data());
    emit_frame();
  }
  void emit_frame() {
    frames_out++;
    if (sink) sink(Frame{frame.data(), o.width, 2 * std_->active, std_});
  }

  void close_line(double next_start) {
    const bool vs = line_low > 0.5 * T;
    if (vs && !prev_line_vsync && line_has_longrun) {
      // field start: phase of the broad-pulse start inside this line
      double ph = (static_cast<double>(line_longrun_start) - line_start) / T;
      ph -= std::floor(ph + 0.25);  // -> [-0.25, 0.75)
      const bool odd = std::fabs(ph) < 0.25;
      // smooth lines: A is the lower field, B the upper one (B's line k is drawn above A's line k)
      const bool smooth = smooth_lines_on();
      if (cur_field == 0) {
        haveA = true;
        if (o.fields) smooth ? emit_field(fieldA, false) : emit(fieldA, fieldA);
      }
      if (cur_field == 1) {
        haveB = true;
        if (o.fields) smooth ? emit_field(fieldB, true) : emit(fieldB, fieldB);
      }
      if (!o.fields && haveA && haveB) {
        emit(fieldA, fieldB);
        haveA = haveB = false;
      }
      cur_field = odd ? 0 : 1;
      field_line0 = static_cast<int64_t>(line_idx);
      fields_seen++;
      if (odd) haveB = false;  // a frame is A then B
    }
    prev_line_vsync = vs;
    line_idx++;
    line_low = 0;
    line_has_longrun = false;
    line_start = next_start;
    refresh_switches();
  }

  void on_hsync(double edge) {
    if (line_start < 0) {
      line_start = edge;
      return;
    }
    const double pred = line_start + T;
    if (std::fabs(edge - pred) <= 0.08 * T) {
      const double ns = 0.6 * edge + 0.4 * pred;
      T = 0.995 * T + 0.005 * (ns - line_start);
      miss_run = 0;
      stray_run = 0;
      count_line(true);
      render_line(line_start, true);
      close_line(ns);
      check_lost();
      return;
    }
    // an hsync-width pulse off the grid: noise, an equalising pulse, or the picture has jumped
    if (last_stray >= 0 && std::fabs(edge - last_stray - T) < 0.03 * T) stray_run++;
    else stray_run = 1;
    last_stray = edge;
    if (stray_run >= 6 && miss_run >= 4) {
      rephases++;
      comb_epoch++;
      KP_LOGD("video: re-phased the line grid by %.1f us", (edge - line_start) / fs * 1e6);
      line_start = edge;
      line_low = 0;
      line_has_longrun = false;
      stray_run = 0;
      miss_run = 0;
    }
  }
};

// ---- front end ----------------------------------------------------------------------------------------
// kepyur: block-wise and branch-free so the compiler vectorises it (the spike's per-sample loop used a whole
// phone core at 40 MS/s). Per chunk of <= kTab samples: DC removal (block mean, smoothed), mix by a
// precomputed rotation table, halfband /2 over a work array with 22 samples of history, discriminator.
FrontEnd::FrontEnd(double fs_in, double shift_hz, FrameSink sink, const DecoderOpts &o)
    : _fs_in(fs_in), _fs_out(fs_in >= 40e6 ? fs_in / 2 : fs_in), _decimate(fs_in >= 40e6) {
  for (int i = 0; i < kHb; i++) {
    const int k = i - kHb / 2;
    const double w = 0.5 - 0.5 * std::cos(2 * M_PI * (i + 0.5) / kHb);
    _h[i] = k == 0 ? 0.5f : static_cast<float>(std::sin(M_PI * k / 2) / (M_PI * k) * w);
  }
  _wr.assign(kHb - 1 + kTab, 0.f);
  _wi.assign(kHb - 1 + kTab, 0.f);
  _dec = new Decoder(_fs_out, std::move(sink), o);
  _clicks = new ClickDetector(_fs_out);
  set_shift(shift_hz);
}

FrontEnd::~FrontEnd() {
  delete _clicks;
  delete _dec;
}

void FrontEnd::set_shift(double s) {
  _shift = s;
  _tr.resize(kTab);
  _ti.resize(kTab);
  for (int i = 0; i < kTab; i++) {
    const double a = -2 * M_PI * s / _fs_in * i;
    _tr[i] = static_cast<float>(std::cos(a));
    _ti[i] = static_cast<float>(std::sin(a));
  }
  const double a = -2 * M_PI * s / _fs_in * kTab;
  _stepN = cf(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
  _rot = cf(1, 0);
}

DecoderStats FrontEnd::stats() const { return _dec->stats(); }

namespace {
// branch-free atan2 (a polynomial, max error ~1e-4 rad) so the discriminator loop vectorises
inline float atan2_bf(float y, float x) {
  const float ax = std::fabs(x), ay = std::fabs(y);
  const float mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
  const float a = mn / (mx + 1e-20f);
  const float s = a * a;
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  r = ay > ax ? 1.57079637f - r : r;
  r = x < 0 ? 3.14159274f - r : r;
  return y < 0 ? -r : r;
}
}  // namespace

void FrontEnd::demod(const cf *xs, size_t n, std::vector<float> &out, std::vector<uint8_t> *bad) {
  out.clear();
  const bool rep = bad && repair_on();
  _clicks->begin(rep);
  if (bad) bad->clear();
  const float k = static_cast<float>(_fs_out / (2 * M_PI) / 1e6);  // rad/sample -> MHz
  constexpr int H = kHb - 1;
  while (n > 0) {
    const int m = static_cast<int>(std::min<size_t>(n, kTab));
    // DC: block mean, smoothed across blocks (the adapter's DC line)
    float sr = 0, si = 0;
    for (int i = 0; i < m; i++) {
      sr += xs[i].real();
      si += xs[i].imag();
    }
    _dc += (cf(sr / m, si / m) - _dc) * 0.3f;
    const float dr = _dc.real(), di = _dc.imag();
    const float pr = _rot.real(), pi = _rot.imag();
    float *wr = _wr.data() + H, *wi = _wi.data() + H;
    const float *tr = _tr.data(), *ti = _ti.data();
    for (int i = 0; i < m; i++) {
      const float xr = xs[i].real() - dr, xi = xs[i].imag() - di;
      const float cr = pr * tr[i] - pi * ti[i], ci = pr * ti[i] + pi * tr[i];  // rot * step^i
      wr[i] = xr * cr - xi * ci;
      wi[i] = xr * ci + xi * cr;
    }
    _rot *= _stepN;
    if (m < kTab) {  // a short chunk: the next one starts at step^m, not step^kTab
      _rot = cf(pr, pi) * cf(tr[m], ti[m]);
    }
    _rot /= std::abs(_rot);
    // halfband /2 (or pass through), then the discriminator
    const size_t o0 = out.size();
    if (_decimate) {
      const float *r = _wr.data(), *q = _wi.data();
      int j0 = _phase;
      const int nout = m - 1 - j0 >= 0 ? (m - 1 - j0) / 2 + 1 : 0;  // windows that end inside this chunk
      _yr.resize(nout);
      _yi.resize(nout);
      for (int j = 0; j < nout; j++) {
        const int s0 = j0 + 2 * j;
        float a = 0.5f * r[s0 + kHb / 2], b = 0.5f * q[s0 + kHb / 2];
        for (int t = 0; t < kHb; t += 2) {
          a += _h[t] * r[s0 + t];
          b += _h[t] * q[s0 + t];
        }
        _yr[j] = a;
        _yi[j] = b;
      }
      _phase = j0 + 2 * nout - m;  // where the next window starts, relative to the next chunk
      std::memmove(_wr.data(), _wr.data() + m, H * sizeof(float));
      std::memmove(_wi.data(), _wi.data() + m, H * sizeof(float));
    } else {
      _yr.assign(wr, wr + m);
      _yi.assign(wi, wi + m);
    }
    const int ny = static_cast<int>(_yr.size());
    out.resize(o0 + ny);
    float *d = out.data() + o0;
    float lr = _prev.real(), li = _prev.imag();
    if (ny > 0) {
      d[0] = atan2_bf(_yi[0] * lr - _yr[0] * li, _yr[0] * lr + _yi[0] * li) * k;
      for (int j = 1; j < ny; j++) {
        const float re = _yr[j] * _yr[j - 1] + _yi[j] * _yi[j - 1];
        const float im = _yi[j] * _yr[j - 1] - _yr[j] * _yi[j - 1];
        d[j] = atan2_bf(im, re) * k;
      }
      _prev = cf(_yr[ny - 1], _yi[ny - 1]);
    }
    if (rep) _clicks->envelope(_yr.data(), _yi.data(), ny);
    xs += m;
    n -= m;
  }
  if (rep) _clicks->mark(out, *bad);
}

void FrontEnd::decode(const float *d, size_t n, const uint8_t *bad) {
  _dec->push_block(d, bad, n);
}

void FrontEnd::feed(const cf *xs, size_t n) {
  demod(xs, n, _dbuf, &_badbuf);
  decode(_dbuf.data(), _dbuf.size(), _badbuf.size() == _dbuf.size() ? _badbuf.data() : nullptr);
}

}  // namespace kp
