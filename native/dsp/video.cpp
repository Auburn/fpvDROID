// fpvDROID video decoder. See video.h. The algorithm follows vtx-spike/viewer/vtxview.cpp (measured on a
// real R4 NTSC VTX: 93-96 % of lines locked); changes are marked "fpvDROID:".
#include "video.h"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <utility>

#include "biquad.h"
#include "deinterlace.h"
#include "field_rhythm.h"
#include "line_draw.h"
#include "log.h"
#include "picture_filters.h"
#include "rings.h"
#include "sparkle.h"
#include "sync_levels.h"

namespace kp {

const VideoStd kNtsc{63.5556, 9.4, 52.6, 240, 18, 262, "NTSC"};
const VideoStd kPal{64.0, 10.5, 52.0, 288, 22, 312, "PAL"};

// ---- the decoder: discriminator output (MHz) in, frames out ---------------------------------------------
// It finds the sync pulses (acquisition, a line PLL that freewheels and re-phases, the field starts, and for a weak
// signal the fold of the lines that holds the grid: kFoldLines), which is its own; for the rest it runs the modules: the levels the pulses are found by (sync_levels.h) and where a field begins
// when its vertical sync is not found (field_rhythm.h); the luma and colour paths behind the pulse tracking into
// the sample rings (picture_filters.h, rings.h), and as each line ends, its burst, sparkle repair (sparkle.h), the
// line into a row (line_draw.h) and, as each field ends, the frame (deinterlace.h); with strips
// (PictureOpts::strips), the frame a strip at a time as the rows come in.
class Decoder {
public:
  // picture: the switches, which whoever owns the decoder may change at any time (FrontEnd::set_picture)
  Decoder(double fs, FrameSink sink, const DecoderOpts &o, const std::atomic<PictureOpts> &picture)
      : fs(fs), sink(std::move(sink)), o(o), picture(picture), levels(fs), drawer(o.width, fs), filt(fs, o.colour),
        spark(fs) {
    lp_sync.design(1.0e6, fs);
    sring.assign(kSyncMask + 1, 0.f);
    fold_before = static_cast<int>(std::lround(kFoldBeforeUs * 1e-6 * fs));
    fold_ref = kFoldRefUs * 1e-6 * fs;
    fold.assign(static_cast<size_t>(std::lround(kFoldUs * 1e-6 * fs)), 0.f);
    fold_ps.assign(fold.size() + 1, 0.f);
    acq.resize(static_cast<size_t>(fs * 0.06));  // 60 ms
    set_std(o.std_force == 2 ? kPal : kNtsc);
  }

  DecoderStats stats() const {
    DecoderStats s;
    s.acquiring = acquiring;
    s.sign = levels.sign;
    s.tip = levels.tip;
    s.blank = levels.blank;
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
    s.level_jumps = levels.jumps;
    s.fields_coasted = fields_coasted;
    s.starts_put_right = starts_put_right;
    s.colour = colour_on;
    s.burst_err = c_err;
    s.clicks_repaired = spark.clicks_repaired;
    s.lines_mended = spark.lines_mended;
    s.holding = weak;
    s.holds = holds;
    // (settled: the levels are measured ones, on locked lines or in the fold, not the histogram's)
    if (!acquiring && sync_var_n >= kNoiseLines && levels.settled() && levels.blank - levels.tip > 0.05f)
      s.sync_snr = (levels.blank - levels.tip) / std::sqrt(std::max(sync_var(), 1e-6f));
    s.depth_ref = depth_ref;
    if (!acquiring && line_start >= 0) {
      s.grid = line_start;
      s.field = cur_field;
      if (cur_field >= 0) s.field0 = line_start - static_cast<double>(line_idx - field_line0) * T;
    }
    return s;
  }

  // A block of discriminator output, MHz; bad: the front end's click marks for it (sparkle repair), or null.
  // Each piece of up to kPiece samples goes through in two passes. First the sync path (a 1 MHz low-pass and a
  // 3-point median), whose filter never changes, over the whole piece. Then the pulse tracking, a sample at a
  // time (track), with the luma and colour paths following behind (filter_to): they are brought up to the
  // sample in hand before anything that reads their rings or changes their switches (an hsync, a line that
  // ends, acquisition), and at the end of the piece. So their filters run in tight loops with the state in
  // registers (a sample at a time through everything, each filter's state goes through memory for every
  // sample: a third of the decode thread on a Pixel 8 Pro), and every sample still gets the same arithmetic,
  // in the same order, with the switches as they were when it came.
  // t_end: when the block's last sample was taken (steady clock, s; 0: not known), for the frames' LineTimes.
  void push_block(const float *d, const uint8_t *bad, size_t cnt, double t_end) {
    blk_end = n + cnt;
    blk_t = t_end;
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
      if (o.hold) {  // the sync path's samples into their ring: one stretch, or two where it wraps
        const size_t at = static_cast<size_t>(n) & kSyncMask, first = std::min(pn, kSyncMask + 1 - at);
        std::memcpy(sring.data() + at, mb, first * sizeof(float));
        std::memcpy(sring.data(), mb + first, (pn - first) * sizeof(float));
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
  const std::atomic<PictureOpts> &picture;
  PictureOpts pic;  // ... as of the start of the line being received (refresh_switches)
  const VideoStd *std_ = &kNtsc;
  SyncLevels levels;  // the polarity, the sync tip and blanking levels, the threshold the pulses are found by
  uint64_t n = 0;     // samples taken (the pulse tracking's count)
  // push_block: the piece in hand (piece_n0: n at its first sample)
  static constexpr size_t kPiece = 512;
  const float *piece_d = nullptr;
  const uint8_t *piece_bad = nullptr;
  uint64_t piece_n0 = 0;
  // ... and the block: n after its last sample, and when that one was taken (0: not known)
  uint64_t blk_end = 0;
  double blk_t = 0;
  // overdue_at: the line_start and T it was worked out for
  bool od_valid = false;
  double od_ls = 0, od_T = 0;
  uint64_t od_at = UINT64_MAX;
  SampleRings R;  // what the filters below have made of the samples so far (R.n <= n: filter_to)
  LineDrawer drawer;
  // NTSC chroma comb (PictureOpts::chroma_comb; LineDrawer::draw does the combing): each line is drawn one line late,
  // once the line below is in, and combed with both neighbours (centred, so chroma stays on luma's line).
  // Luma keeps the notch. PAL (its subcarrier pattern repeats over two lines) is not combed.
  static constexpr float kCombMaxErr = 0.35f;  // rad: a burst further off means the stream jumped
  bool comb = false;
  uint64_t comb_epoch = 0;     // bumped where the sample stream may have jumped: lines across it don't mix
  LineInfo comb_up, comb_mid;  // the two lines before the current one
  float burst_e = 0;           // |phase error| of the last burst against the PLL's prediction
  // fpvDROID: colour. A per-line PLL on the burst tracks the transmitter's subcarrier phase (and its small
  // frequency error) against the filters' free-running oscillator. PAL: the burst's +-45 degree swing gives
  // the V switch, and adjacent lines are averaged (PAL-D).
  double c_phase = 0, c_dphi = 0;  // burst reference phase (rad) and its drift per line
  int64_t c_last_line = -1;
  float c_mag = 0, c_err = 1;      // burst amplitude (MHz), mean |phase error| (rad)
  float c_size = 0;                // the burst amplitude that the colour is drawn against (LineInfo::mag)
  bool colour_on = false;
  int c_good = 0;
  float med_a = 0, med_b = 0;
  Biquad lp_sync;  // the sync path: a 1 MHz low-pass, then a 3-point median (med_a, med_b)
  // The luma and colour paths, which follow behind the pulse tracking (filter_to). Their switches are set once
  // per line (refresh_switches).
  PictureFilters filt;
  float inv_depth = 1;  // 1 / sync depth, as of the line's start
  SparkleRepair spark;  // sparkle repair (PictureOpts::repair), on the click marks in R.bad
  bool low = false, longrun = false;
  uint64_t low_start = 0;
  static constexpr double kHsMinUs = 3.0, kHsMaxUs = 6.5;  // an hsync: a pulse this long (4.7 us nominal)
  double hs_len = 0;  // how long hsyncs have been lately, in samples (on_hsync)
  double T = 0, line_start = -1;
  uint64_t lines = 0, lines_locked = 0, acquisitions = 0, losses = 0;
  int64_t line_idx = 0;  // the line in hand, counted from construction
  uint64_t line_low = 0;
  bool line_has_longrun = false;
  uint64_t line_longrun_start = 0;
  bool prev_line_vsync = false;
  std::optional<FieldStart> pending_start;  // a field start found, which waits for the lines after it (close_line)
  FieldRhythm rhythm;  // fpvDROID: the vertical flywheel (DecoderOpts::flywheel): where a field is due to begin
  uint64_t fields_coasted = 0, starts_put_right = 0;
  std::vector<float> acq;
  size_t acq_n = 0;
  bool acquiring = true;
  std::vector<uint8_t> fieldA, fieldB, frame;
  Deinterlacer deint;
  bool haveA = false, haveB = false;
  int cur_field = -1;
  int64_t field_line0 = 0;
  // The field being drawn goes into the frame, and out, all at once when it ends, or with low latency
  // (PictureOpts::strips) a strip at a time
  bool field_strips = false;  // this field does (strips as the field began)
  int part_y = 0;             // the frame's lines [0, part_y) are in and out already
  std::vector<double> row_t;  // when the samples each of the field's rows was drawn from were taken (LineTimes)
  uint64_t frames_out = 0, fields_seen = 0;
  // fpvDROID: re-phase instead of re-acquire. After a jump in the signal (stream restart, a dropped bank)
  // the hsyncs keep coming, one line apart, but outside the PLL's window. A run of them on a steady grid
  // while the PLL freewheels moves the line grid onto them within a few lines.
  int miss_run = 0, stray_run = 0;
  double last_stray = -1;
  uint64_t rephases = 0;
  // fpvDROID: lock quality over a sliding window of lines (the spike counted since acquisition)
  static constexpr int kRecent = 1024;
  uint8_t recent[kRecent] = {};
  int recent_pos = 0, recent_n = 0, recent_locked = 0;
  // fpvDROID: holding the grid through a weak signal (DecoderOpts::hold). An FM signal that sinks into the noise
  // does two things to the sync pulses. Each line's own hsync is no longer found: a pulse of the right length
  // between two crossings of a threshold is a matter of chance once the noise is as large as the pulse is deep.
  // And the levels move: the discriminator is captured by the noise, whose mean is the middle of the band, and
  // tip and blanking both shrink towards it, until the threshold kept from better times is outside the pulses.
  // The lock was then lost and looked for again, 60 ms without a picture at a time, with the levels and the
  // fields' rhythm forgotten: four times a second in recordings of such a signal (the noise in the sync path as
  // large as the sync was deep). All of that for nothing: the camera's lines come from a crystal, and in those
  // recordings they stay where a line period says, to a fraction of a microsecond, for as long as each lasts.
  // So the lines are folded: the sync path's samples around where each line's hsync is due, a leaky mean over
  // kFoldLines lines, in which the pulse stands out of the noise eight times better than in any one line.
  //  - When fewer than half of the last 64 lines find their own hsync the fold takes over (weak). It says where
  //    the pulse is, and the grid is steered onto it; and at which levels, and tip and blanking follow them. A
  //    line's own hsync then only counts as found, and only within kTightUs of the grid. 60 of 64 lines finding
  //    theirs again ends it.
  //  - While the fold does not show the pulse either, the grid runs on at the line period. That is known from a
  //    long stretch of lines (grid_sure) far better than the lines' own tracking has it: to a few parts in ten
  //    million, a fraction of a microsecond over a second. The lock is lost only when that has lasted kHoldS.
  //  - The fields are found by what their half lines are (vert_line), and a signal that is weak from the start is
  //    acquired by the fold of its lines (acquire_by_fold).
  //  - With some noise in the signal but each line's hsync still steering, the fold guards the grid: a pulse far
  //    from where the fold shows the hsync is not taken for one (on_hsync).
  static constexpr int kFoldLines = 64, kFoldMin = 16;
  static constexpr double kFoldBeforeUs = 7, kFoldUs = 21;  // the fold: from so long before each line's start, so long
  static constexpr double kTightUs = 1.0, kNearUs = 1.5, kHoldS = 0.5;
  static constexpr double kSteer = 1.0 / 16;  // of the way to where the fold has the pulse, every fourth line
  static constexpr size_t kSyncMask = 8191;
  std::vector<float> sring;  // the sync path's samples, by absolute index (& kSyncMask)
  std::vector<float> fold, fold_ps;
  int fold_before = 0, fold_n = 0, fold_due = 0;
  float fold_var = 0;                // a line's samples about the fold, mean square: the noise
  // Where the fold has the hsync's edge, from the line's start (fold_look), and where that is while each line's own
  // hsync steers (fold_ref): a little later, because an hsync is found where the sync path's signal crosses the
  // threshold, the fold measures the edge by area, and behind that path's low-pass the bulk of an edge comes
  // after its half-way point. By how much is the transmitter's: 0.31 us in recordings of two, 0.13 us in those
  // of a third. So it is measured while nearly every line finds its hsync, and kept.
  static constexpr double kFoldRefUs = 0.2, kFoldRefMinUs = -0.1, kFoldRefMaxUs = 0.5;
  double fold_pos = 0, fold_ref = 0;
  int fold_ref_n = 0;
  float fold_tip = 0, fold_blank = 0;
  double fold_carry = 0;             // steering not yet passed on to the fold (less than a sample)
  bool fold_good = false;            // the fold shows the hsync where the grid is (looked at every 16 lines)
  bool weak = false;
  uint64_t found_bits = ~0ull;       // the last 64 lines, 1: its hsync was found
  int unseen = 0;                    // weak: lines since the fold last showed the pulse
  double sure_start = -1;            // the start of a line as it was when most lines still found their hsync ...
  uint64_t sure_idx = 0;             // ... and which line
  double base_start = -1, mid_start = -1;  // such a line long before, and one half as long before (grid_sure) ...
  uint64_t base_idx = 0, mid_idx = 0;
  double T_long = 0;                 // ... and the line period over the lines between
  bool long_ok = false;
  static constexpr uint64_t kLongMin = 2000, kLongSpan = 65536;  // lines: at least, and at most
  int weak_lines = 0;
  int weak_ago = 1 << 30;            // lines since the fold last steered
  uint64_t holds = 0;
  int found64() const { return std::popcount(found_bits); }
  // Is the signal a faint one: held by the fold now, or within the last two fields? Such a signal shows the
  // vertical sync of few of its fields (one in nine, in one recording): see FieldRhythm::doubt and seen.
  bool faint() const { return o.hold && (weak || weak_ago < 2 * std_->field_lines); }

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
    if (!levels.settled()) levels.track(m);  // until the sync-gated measurement has settled
    const float w = levels.sign * m, thr = levels.thr;
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
        if (wid > kHsMinUs && wid < kHsMaxUs) {
          filter_to(n);
          on_hsync(static_cast<double>(low_start), static_cast<double>(now - low_start));
        }
      }
      line_low++;
    }
    // freewheel: line overdue by > 8 % with no edge. A pulse that is low now and may yet be an hsync counts as
    // an edge: what it is shows only when it ends, 4.7 us after it began, and the line is overdue 5.1 us after
    // its hsync was due. Without the wait an hsync more than 0.4 us late was never taken, though the window is
    // 5.1 us either side: the line freewheeled, the pulse came out as a stray one line off, and so did every
    // hsync after it, until the run of strays re-phased the grid six lines on.
    if (now > overdue_at() && !(low && (now - low_start) / fs * 1e6 < kHsMaxUs)) {
      filter_to(n);
      miss_run++;
      end_line(line_start + T, false);
    }
  }

  // The line in hand has ended, on its hsync (locked) or overdue without one, and the next begins at next_start:
  // it is counted, its sync looked at for levels that have moved (DecoderOpts::follow_levels), drawn and closed.
  void end_line(double next_start, bool locked) {
    const double ls = line_start;
    count_line(locked);
    // most lines find their hsync: the grid as it is now is one to come back to, and one end of the stretch of
    // lines the line period is measured over (grid_sure)
    if (o.hold && locked && !weak && found64() >= 52) grid_sure(next_start, static_cast<uint64_t>(line_idx) + 1);
    LineLevels sync;
    if (o.follow_levels && !weak) {  // (held by the fold, the levels are the fold's)
      const float was = levels.tip;
      sync = levels.look(R, ls);
      if (sync.jumped) {
        KP_LOGD("video: the levels moved by %+.2f MHz, to tip %.2f blank %.2f, at line %lld of the field",
                levels.tip - was, levels.tip, levels.blank, line_in_field());
        pending_start.reset();  // the field start that waits (close_line) was the signal dropping
      }
    }
    render_line(ls, locked, sync.own);
    close_line(next_start, sync.own.has_value());
    check_lost();
  }

  // Which line of the field in hand the line in hand is, for the log (-1: no field)
  long long line_in_field() const { return cur_field < 0 ? -1LL : static_cast<long long>(line_idx - field_line0); }

  // How many of the next samples (sync-path values m[0..cnt)) track() would do nothing with: locked on (not
  // acquiring, the levels settled), between pulses, still above the threshold, and the line not yet overdue.
  // They are passed over here, n moved on past them; the first that would do something is left to track().
  // ~90 % of the samples, looked at eight at a time (a vector compare), where track() takes each on its own.
  size_t quiet_run(const float *m, size_t cnt) {
    if (acquiring || !levels.settled() || low) return 0;
    const uint64_t od = overdue_at();
    if (od < n) return 0;  // the next sample is overdue
    if (od - n < cnt) cnt = static_cast<size_t>(od - n) + 1;  // samples up to and including od are not
    const float s = levels.sign, t = levels.thr;
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
    filt.run(R, piece_d + k0, bd, cnt);
  }

  void set_std(const VideoStd &s) {
    std_ = &s;
    rhythm.reset();
    long_ok = false;
    T = s.T_us * 1e-6 * fs;
    wide.assign(static_cast<size_t>(T / kWideStep), 0.f);  // a line of it
    wide_ps.assign(3 * wide.size() + 1, 0.f);
    wide_n = 0;
    fieldA.assign(static_cast<size_t>(o.width) * s.active * 4, 0);
    fieldB = fieldA;
    frame.assign(static_cast<size_t>(o.width) * s.active * 2 * 4, 0);
    row_t.assign(s.active, 0);
    part_y = 0;
    filt.set_std(s);
    refresh_switches();
    comb_epoch++;
    drawer.set_std(s, filt.comb_w());
    colour_on = false;
    c_good = 0;
    c_last_line = -1;
  }

  // The picture switches as they now stand, and the timing, for the line that begins. PAL's colour repeats over
  // two lines, not one: it keeps the notch and is not combed.
  void refresh_switches() {
    pic = picture.load(std::memory_order_relaxed);
    const bool ntsc = std_ != &kPal;
    const bool luma_comb = pic.luma_comb && ntsc;
    if (comb != (pic.chroma_comb && ntsc)) {
      // The chroma comb comes on or goes off. Lines either side of the change do not mix, and the line the comb
      // was holding back (it draws each a line late) is not drawn: its row keeps what the frame before had.
      comb = !comb;
      comb_epoch++;
      comb_up = comb_mid = LineInfo{};
    }
    inv_depth = 1.f / std::max(levels.blank - levels.tip, 0.2f);
    filt.set_line(luma_comb, luma_comb && pic.adaptive_comb, comb, T, inv_depth);
  }

  void count_line(bool locked) {
    lines++;
    if (locked) lines_locked++;
    if (recent_n == kRecent) recent_locked -= recent[recent_pos];
    else recent_n++;
    recent[recent_pos] = locked;
    recent_locked += locked;
    recent_pos = (recent_pos + 1) % kRecent;
    found_bits = (found_bits << 1) | (locked ? 1u : 0u);
  }

  // fpvDROID: lost when fewer than 40 % of the last ~2 fields' lines found their hsync (a healthy picture
  // locks ~90 %: only the vertical interval freewheels). The spike used the ratio since acquisition. With the
  // hold (kFoldLines), which takes such a signal over long before: when the fold has not shown the hsync for
  // kHoldS.
  void check_lost() {
    if (weak) {
      // the fold has the say: lost when it has not shown the pulse for kHoldS
      if (unseen * T > kHoldS * fs) {
        losses++;
        KP_LOGI("video: lost lock (no hsync in the fold for %.1f s), re-acquiring", unseen * T / fs);
        restart_acquisition();
      }
      return;
    }
    // (with the hold, a signal whose lines do not find their hsync is one for the fold before it gets here)
    if (o.hold && lines >= 64) return;
    const int window = std::min(kRecent, 2 * std_->field_lines);
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
    field_strips = false;
    haveA = haveB = false;
    recent_n = recent_locked = recent_pos = 0;
    line_start = -1;
    levels.restart();
    pending_start.reset();
    rhythm.reset();
    miss_run = stray_run = 0;
    last_stray = -1;
    hs_len = 0;
    comb_epoch++;
    weak = false;
    weak_ago = 1 << 30;
    found_bits = ~0ull;
    fold_n = fold_due = unseen = 0;
    fold_good = false;
    sure_start = base_start = -1;
    vq_n = 0;
    sync_var_n = 0;
  }

  // acquisition: pick polarity + levels + line period from a 60 ms buffer
  void acquire() {
    float best = 0, bt = 0, bb = 0, bs = 0;
    for (float sgn : {+1.f, -1.f}) {
      if (o.force_invert && sgn > 0) continue;
      float t, b;
      if (!levels.survey(acq, acq_n, sgn, t, b)) continue;
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
      levels.acquired(bs, bt, bb);
      acquiring = false;
      line_start = -1;
      acquisitions++;
      // the transmitter's line period is what it was before, and known far better than from this buffer's median
      if (o.hold && long_ok && std::fabs(T - T_long) < 0.25) T = T_long;
      else long_ok = false;
      KP_LOGI("video: acquired, sync %s end, tip %.2f blank %.2f MHz, line %.3f us -> %s, score %.2f",
              bs > 0 ? "low" : "high", bt, bb, T / fs * 1e6, std_->name, best);
    }
    if (acquiring && o.hold) acquire_by_fold();
    acq_n = 0;
  }

  // fpvDROID: acquisition of a signal too weak for the above, which wants two peaks in a histogram for the levels
  // and then pulses between crossings of a threshold: in noise as large as the sync is deep it finds neither, at
  // any picture. The fold of the lines does (see kFoldLines). The 60 ms are folded at the standard's line period
  // (or the transmitter's, where it is known from before) in eight parts. In each the hsync is looked for as in
  // fold_look, and must be what lies lowest in the whole line. A transmitter's line period is a few tens of parts
  // in a million off the standard's, so the pulse moves along from part to part: at least five of the eight must
  // have it on one straight line, to 0.5 us, and that line's slope is the period. (Five places in a 63 us line
  // that agree so by chance: once in hours of pure noise, and then the fold does not show a pulse, and the lock is
  // given up again after kHoldS.) The polarity is the one in use: the one found last, the usual one before that.
  // Then the fold holds the grid from the first line on.
  void acquire_by_fold() {
    constexpr int kParts = 8, kNeed = 5;
    const double us = 1e-6 * fs;
    const int step = std::max(1, static_cast<int>(std::lround(0.2 * us)));  // every 0.2 us will do: 1 MHz of signal
    const size_t part = acq_n / kParts;
    const auto bins = [&](double t_us) { return static_cast<int>(std::lround(t_us * us / step)); };
    struct Got {
      const VideoStd *std = nullptr;
      int parts = 0;
      double T = 0, edge = 0;
      float tip = 0, blank = 0;
    } got;
    std::vector<float> f, ps;
    std::vector<int> cnt;
    for (const VideoStd *cand : {&kNtsc, &kPal}) {
      if ((o.std_force == 1 && cand != &kNtsc) || (o.std_force == 2 && cand != &kPal)) continue;
      const double T0 = long_ok && cand == std_ ? T_long : cand->T_us * us;
      const int K = static_cast<int>(T0 / step);
      if (K < bins(40.0) || part < static_cast<size_t>(32 * T0)) continue;
      double pos[kParts];
      float tips[kParts], blanks[kParts];
      bool ok[kParts];
      for (int p = 0; p < kParts; p++) {
        f.assign(static_cast<size_t>(K), 0.f);
        cnt.assign(static_cast<size_t>(K), 0);
        double ph = std::fmod(static_cast<double>(p * part), T0);
        for (size_t i = p * part; i < (p + 1) * part; i += static_cast<size_t>(step)) {
          const int k = std::min(K - 1, static_cast<int>(ph / step));
          f[static_cast<size_t>(k)] += levels.sign * acq[i];
          cnt[static_cast<size_t>(k)]++;
          ph += step;
          if (ph >= T0) ph -= T0;
        }
        // sums over the fold laid end to end three times, so that a stretch may run over either end
        ps.assign(static_cast<size_t>(3 * K + 1), 0.f);
        for (int k = 0; k < 3 * K; k++) {
          const size_t j = static_cast<size_t>(k % K);
          ps[static_cast<size_t>(k + 1)] = ps[static_cast<size_t>(k)] + f[j] / static_cast<float>(std::max(cnt[j], 1));
        }
        const auto mean = [&](int a, int b) {
          return (ps[static_cast<size_t>(K + b)] - ps[static_cast<size_t>(K + a)]) / static_cast<float>(b - a);
        };
        float best = 0, lowest = 1e30f, tp = 0, bk = 0;
        int e = -1;
        for (int k = 0; k < K; k++) {
          float t, b;
          const float r = pulse_at(mean, bins, k, t, b);
          lowest = std::min(lowest, t);
          if (r > best) best = r, e = k, tp = t, bk = b;
        }
        double edge;
        ok[p] = e >= 0 && best >= 0.02f && tp <= lowest + 0.25f * best && edge_of(mean, bins, e, tp, bk, edge);
        if (!ok[p]) continue;
        pos[p] = edge * step;
        tips[p] = tp;
        blanks[p] = bk;
      }
      // the pulse's place from part to part, unwrapped, and the straight line through it
      double u[kParts], last = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
      int n_ok = 0, prev = -1;
      for (int p = 0; p < kParts; p++) {
        if (!ok[p]) continue;
        last = prev < 0 ? pos[p] : last + std::remainder(pos[p] - pos[prev], T0);
        u[p] = last;
        prev = p;
        n_ok++;
        sx += p, sy += last, sxx += static_cast<double>(p) * p, sxy += p * last;
      }
      KP_LOGD("video acq by fold: %s: %d of %d parts show a pulse", cand->name, n_ok, kParts);
      if (n_ok < kNeed) continue;
      const double den = n_ok * sxx - sx * sx;
      const double slope = den > 0 ? (n_ok * sxy - sx * sy) / den : 0, at0 = (sy - slope * sx) / n_ok;
      int in = 0;
      float tip_m = 0, blank_m = 0;
      for (int p = 0; p < kParts; p++) {
        if (!ok[p] || std::fabs(u[p] - (at0 + slope * p)) > 0.5 * us) continue;
        in++;
        tip_m += tips[p];
        blank_m += blanks[p];
      }
      if (in < kNeed || in <= got.parts || std::fabs(slope) > 1.5 * us) continue;
      got.std = cand;
      got.parts = in;
      got.T = T0 + slope * T0 / static_cast<double>(part);
      // each part's place is its middle's: the line through them, on to the buffer's end
      const double at_end = at0 + slope * (kParts - 0.5);
      got.edge = at_end + T0 * std::floor((static_cast<double>(acq_n) - at_end) / T0);  // the last hsync in the buffer
      got.tip = tip_m / static_cast<float>(in);
      got.blank = blank_m / static_cast<float>(in);
    }
    if (!got.std) return;
    if (got.std != std_) set_std(*got.std);
    T = got.T;
    levels.set(got.tip, got.blank);  // (measured levels: none of the histogram's after this)
    acquiring = false;
    acquisitions++;
    // sample j of the buffer is sample n - 1 - acq_n + j of the signal; an hsync is found fold_ref before where
    // the fold has its edge
    line_start = static_cast<double>(n - 1 - acq_n) + got.edge - fold_ref;
    weak = true;
    holds++;
    found_bits = 0;
    weak_lines = unseen = fold_due = fold_n = 0;
    fold_carry = 0;
    wide_n = 0;
    wide_line = 0;
    KP_LOGI("video: acquired by the fold of the lines (%d of %d parts), tip %.2f blank %.2f MHz, line %.4f us -> %s",
            got.parts, kParts, levels.tip, levels.blank, T / fs * 1e6, std_->name);
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
    const int64_t li = line_idx;
    const double pred = c_last_line < 0 ? std::arg(b) : c_phase + c_dphi * (li - c_last_line);
    const bool faint = faint_colour();
    if (faint != c_faint) {
      c_faint = faint;
      c_z = cf{};
      c_zn = 0;
      c_zmag = c_nvar = 0;
    }
    if (faint) {
      // (how far the burst's samples lie from their mean: the colour path's noise, a sample)
      float nv = 0;
      for (int64_t i = i0; i < i1; i++) nv += std::norm(R.chroma[i & R.mask] - b);
      return faint_burst(b, mag, nv / static_cast<float>(i1 - i0), pred, li, pal_v);
    }
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
    c_size = c_mag;
    // colour killer, with hysteresis: a burst near the expected size (~1/4 of the sync depth) and steady, in a
    // signal whose sync is not too noisy (colour_quiet)
    const bool ok = colour_quiet && c_mag > 0.08f * (levels.blank - levels.tip) && c_err < 0.5f;
    c_good = ok ? std::min(c_good + 1, 60) : std::max(c_good - 2, 0);
    if (!colour_on && c_good >= 40) colour_on = true;
    if (colour_on && c_good == 0) colour_on = false;
    return colour_on;
  }

  // fpvDROID: colour from a noisy signal. One line's burst is then mostly noise (an FM signal's noise grows with
  // the frequency, and the burst sits at the top of the band): a reference that follows each burst a fifth of
  // the way swings with the noise, and the picture gets bands of green and purple. But the subcarrier is as
  // steady as the line period, and the hold has every line's start whether its hsync was found or not: so the
  // bursts of the last kFaintLines lines, each turned back by the reference predicted for its line, are averaged,
  // and the reference follows that mean slowly. How much of the bursts' size is left in the mean (c_coh: 1 without
  // noise, about 0.1 with nothing but noise) says whether there is colour to be had: under kFaintOff there is
  // none. Above it the picture's colour is drawn smoothed along the line and down the field (LineInfo::calm), and
  // its saturation is turned down by the share of signal in what the colour path then gives: the burst's size,
  // squared, against that and what is left of the path's noise (kFaintNoise of a sample's). The picture's colours
  // stay where they are, paler, and the coloured snow is as much paler.
  // NTSC only so far: PAL's swinging burst wants the lines' V switch from more than one line as well (PAL keeps
  // the cut: no colour while the sync is noisy).
  static constexpr int kFaintLines = 128, kFaintMin = 32;
  static constexpr float kFaintOn = 0.4f, kFaintOff = 0.25f;
  static constexpr double kFaintStep = 0.05, kFaintDrift = 0.0005;
  static constexpr float kFaintCalm = 0.1f, kFaintNoise = 0.3f;
  cf c_z{};           // the mean of those bursts, against the reference
  float c_zmag = 0;   // and of their sizes
  float c_nvar = 0;   // the colour path's noise, a sample: mean square
  int c_zn = 0;
  float c_coh = 0;
  bool c_faint = false;
  bool faint_colour() const { return o.hold && !colour_quiet && std_ != &kPal; }
  bool faint_burst(cf b, float mag, float nv, double pred, int64_t li, bool &pal_v) {
    pal_v = false;
    const cf d = b * std::polar(1.f, static_cast<float>(-pred));
    c_zn = std::min(c_zn + 1, kFaintLines);
    const float a = 1.f / static_cast<float>(c_zn);
    c_z += a * (d - c_z);
    c_zmag += a * (mag - c_zmag);
    c_nvar += a * (nv - c_nvar);
    const double ez = std::arg(c_z), step = kFaintStep * ez;
    c_dphi += kFaintDrift * ez / static_cast<double>(std::max<int64_t>(1, li - c_last_line));
    c_phase = std::remainder(pred + step, 2 * M_PI);
    c_z *= std::polar(1.f, static_cast<float>(-step));  // (the mean is against the reference, and turns with it)
    c_last_line = li;
    const float e = std::fabs(std::arg(d));
    burst_e = e;
    c_mag += 0.05f * (mag - c_mag);
    c_err += 0.05f * (e - c_err);
    if (c_zn < kFaintMin) return colour_on;  // (as it was, until there is a mean to go by)
    c_coh = c_zmag > 0 ? std::min(1.f, std::abs(c_z) / c_zmag) : 0;
    const bool ok = c_coh > (colour_on ? kFaintOff : kFaintOn);
    c_good = ok ? std::min(c_good + 1, 60) : std::max(c_good - 2, 0);
    if (!colour_on && c_good >= 40) colour_on = true;
    if (colour_on && c_good == 0) colour_on = false;
    // the burst's size as the mean has it, and the saturation down by the signal's share (LineInfo::mag divides)
    const float s = std::norm(c_z), share = s / std::max(s + kFaintNoise * c_nvar, 1e-12f);
    c_size = std::abs(c_z) / std::max(share, 0.02f);
    return colour_on;
  }

  bool in_picture(int64_t k) const { return k >= 0 && k < std_->active; }

  // The line that has just ended (ls: its start; k: its row; locked: its hsync was found; own: the levels of its
  // sync, where they are not the kept ones), ready to be drawn: levels measured on it, then its picture put right
  // in the rings (the top line with the luma comb, sparkle repair), then its burst. In that order: each reads what
  // the one before left.
  LineInfo measure(double ls, int64_t k, bool locked, const std::optional<LevelPair> &own) {
    LineInfo l;
    l.ls = ls;
    l.k = k;
    l.field = cur_field;
    l.epoch = comb_epoch;
    l.own = own;
    if (!in_picture(k)) return l;
    if (locked && !own && !weak) levels.measure(R, ls);
    if (filt.luma_comb()) top_line(ls, k);
    l.hit = pic.repair && patch_line(ls);
    // (in colour: a line whose hsync was found, or one the hold has the start of, for a noisy signal's colour)
    l.col = o.colour && (locked || (weak && faint_colour())) && burst(ls, l.pal_v);
    l.ok = l.col && burst_e < kCombMaxErr;
    l.phase = c_phase;
    l.mag = c_size;
    l.calm = l.col && c_faint ? kFaintCalm : 0;
    l.fade = filt.fade_noted();
    return l;
  }

  // A line has ended: it is drawn, or with the 3-line comb the one before it is.
  void render_line(double ls, bool locked, const std::optional<LevelPair> &own) {
    if (cur_field < 0) return;
    const LineInfo cur = measure(ls, line_idx - field_line0 - std_->skip, locked, own);
    if (comb) {
      comb_line(cur);
    } else if (in_picture(cur.k)) {
      draw_line(cur, false, false);
      if (pic.repair) mend_rows(cur.k, cur.hit);
      row_drawn(cur.k);
    }
  }

  // NTSC comb: the line before cur, k - 1, is drawn, combed with k - 2 and k. A neighbour is used only if both
  // lines are locked and in colour, in the same field and on the same side of any jump in the stream
  // (comb_epoch, and a burst on its predicted phase).
  void comb_line(const LineInfo &cur) {
    const LineInfo &m = comb_mid;
    if (m.field == cur_field && m.k == cur.k - 1 && in_picture(m.k)) {
      const auto pair = [&](const LineInfo &x, int64_t dk) {
        return m.ok && x.ok && x.field == m.field && x.k == m.k + dk && x.epoch == m.epoch;
      };
      draw_line(m, pair(comb_up, -1), pair(cur, 1));
      if (pic.repair) mend_rows(m.k, m.hit);
      row_drawn(m.k);
    }
    comb_up = comb_mid;
    comb_mid = cur;
  }

  // The line into its row of the field being built (line_draw.h). up / dn: comb its chroma with the line above /
  // below (comb_line).
  void draw_line(const LineInfo &l, bool up, bool dn) {
    uint32_t *row = reinterpret_cast<uint32_t *>((cur_field == 0 ? fieldA : fieldB).data()) + static_cast<size_t>(l.k) * o.width;
    const LevelPair lv = l.own.value_or(LevelPair{levels.tip, levels.blank});
    drawer.draw(row, R, l, up, dn, DrawParams{levels.sign, lv.tip, lv.blank, T, filt.cdel()});
  }

  // The top of the picture with the luma comb. The first picture line of a field has blanking above it, not
  // picture, and the luma comb works each line out against the line before. Against blanking it leaves half of
  // the colour in the brightness as dots, and the adaptive comb, which sees a line that differs all the way
  // along, fades the colour out as it does under any horizontal edge: the top rows of every frame would be grey.
  // There is nothing above that line to keep rainbows off, so its picture is worked out again, the way the
  // decoder does without the luma comb (PictureFilters::redo_without_comb). Only the picture's own stretch of
  // that one line is replaced: levels, burst and every other line are as the comb left them.
  // Which line that is depends on the camera and on where the field was found to start: the first drawn row
  // (k = 0) or one of the next few, with blank rows above it. So it is looked for: the first of rows 0..kTopRows
  // that has picture (it averages kTopPicture sync depths above blanking or more: the level from which the
  // adaptive comb starts to see a difference) while every row above it, and the line before row 0, averages
  // within kTopBlank sync depths of blanking. The second limit is the tighter one on purpose: black picture
  // sits 7.5 IRE (0.19 sync depths) above blanking, and a row under black rows of picture is a line inside the
  // picture, which keeps the comb's fade. (A camera that sends black at the blanking level cannot be told
  // apart this way: under up to kTopRows black rows at the top of its picture, the first brighter row is taken
  // for the top line and drawn as without the luma comb.) A field that begins with the standard's half line has
  // it just above row 0 here: then row 0 has blanking above its left half only, and only that half is done (to
  // the middle, where the half line's picture has begun and the comb is right again). A line further down that
  // follows a dark line is not touched: there the fade is Play RX's trade.
  static constexpr int kTopRows = 4;
  static constexpr float kTopPicture = 0.25f, kTopBlank = 0.10f;
  void top_line(double ls, int64_t k) {
    if (k > kTopRows) return;
    const double a0 = std_->a0_us * 1e-6 * fs, aw = std_->aw_us * 1e-6 * fs;
    // no picture over the part [x0, x1) of the width of the line that starts at l?
    const auto blanking = [&](double l, double x0, double x1, float limit = kTopBlank) {
      float m;
      return levels.mean(R, l + a0 + aw * x0, l + a0 + aw * x1, m) && (m - levels.blank) * inv_depth < limit;
    };
    if (blanking(ls, 0, 1, kTopPicture)) return;
    for (int64_t up = 2; up <= k + 1; up++)  // the rows above the line before, and the line before row 0
      if (!blanking(ls - static_cast<double>(up) * T, 0, 1)) return;
    const bool whole = blanking(ls - T, 0, 1);
    if (!whole && (k > 0 || !blanking(ls - T, 0, 0.4))) return;
    filt.redo_without_comb(R, ls + a0, aw, whole);
  }

  // Sparkle repair (sparkle.h): the picture of the line that starts at ls patched in the rings, before it is
  // drawn; true when it is still badly hit, which mend_rows is told once the row (k) is drawn.
  bool patch_line(double ls) {
    const double a0 = std_->a0_us * 1e-6 * fs, aw = std_->aw_us * 1e-6 * fs;
    return spark.patch_line(R, ls + a0, aw, T, filt.luma_comb(), filt.colour_fades());
  }
  void mend_rows(int64_t k, bool hit) {
    spark.mend_rows((cur_field == 0 ? fieldA : fieldB).data(), o.width, fields_seen, cur_field, k, hit);
  }

  // Row k of the field has been drawn, and sparkle repair has done what it can with it so far. With strips, the frame
  // lines that are now final go out once there are a strip's worth of them. Final: sparkle repair may yet redraw
  // this row and the two above it (a run of badly hit rows, mended once a good row comes below it: mend_rows), and
  // with smooth lines the frame line below an upper field's row is made with the row after it too.
  void row_drawn(int64_t k) {
    // the newest sample it was drawn from is the newest taken: that far before the end of the block in hand
    row_t[k] = blk_t > 0 ? blk_t - static_cast<double>(blk_end - n) / fs : 0;
    if (!field_strips) return;
    const int64_t fin = pic.repair ? k - 3 : k;
    if (fin < 0) return;
    const int y1 = static_cast<int>(pic.smooth_lines && cur_field == 1 ? 2 * fin + 1 : 2 * fin + 2);
    if (y1 - part_y >= 2 * kStripRows) put_part(y1);
  }

  // Frame lines [part_y, y1) of the field being drawn into the frame, interpolated (smooth lines) or each of the
  // field's lines twice (DecoderOpts::fields), and the frame to the sink with those lines new.
  void put_part(int y1) {
    const int W = o.width, A = std_->active;
    y1 = std::min(y1, 2 * A);
    if (y1 <= part_y) return;
    const bool upper = cur_field == 1;
    const uint8_t *f = (upper ? fieldB : fieldA).data();
    if (pic.smooth_lines) deint.smooth(f, upper, W, A, frame.data(), part_y, y1);
    else double_lines(f, W, A, frame.data(), part_y, y1);
    const int y0 = part_y;
    part_y = y1;
    emit_frame(y0, y1, times_of(y0, y1));
  }

  // When the field rows in frame lines [y0, y1) were taken (LineTimes: of the first and the last of them drawn)
  LineTimes times_of(int y0, int y1) const {
    LineTimes t;
    const int r0 = std::max(y0 >> 1, 0), r1 = std::min((y1 - 1) >> 1, std_->active - 1);
    for (int r = r0; r <= r1 && t.t_first <= 0; r++) t.t_first = row_t[r];
    for (int r = r1; r >= r0 && t.t_last <= 0; r--) t.t_last = row_t[r];
    return t;
  }

  // A field begins: strips or not, as the switches say now
  void begin_field() {
    field_strips = o.fields && pic.strips;
    part_y = 0;
    std::fill(row_t.begin(), row_t.end(), 0);
  }

  // The field being drawn has ended (DecoderOpts::fields): what is not out yet of its frame goes out. All of it, or
  // with strips the last one.
  void field_done() { put_part(2 * std_->active); }
  // A frame woven from two fields (not DecoderOpts::fields)
  void emit(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
    weave(a.data(), b.data(), o.width, std_->active, frame.data());
    emit_frame(0, 2 * std_->active, {});
  }
  // The frame to the sink, its lines [y0, y1) new
  void emit_frame(int y0, int y1, const LineTimes &t) {
    const int H = 2 * std_->active;
    if (y1 >= H) frames_out++;
    if (sink) sink(Frame{frame.data(), o.width, H, std_, y0, y1, t});
  }

  // The line in hand is closed, and the next begins at next_start. moved: its sync was at other levels than the
  // kept ones (end_line).
  void close_line(double next_start, bool moved) {
    // A line whose levels have moved was low against a threshold that was no longer right: it starts no field.
    // Nor does the line before the first of them, the one the signal dropped in, low from there to its end (the
    // picture's black under the old threshold). So a field start waits for the line after it, in a vertical sync
    // another of its lines, and while lines say the levels have moved: when they have (end_line), it is dropped.
    const bool vs = line_low > 0.5 * T;
    if (o.hold) vert_line(line_start);
    if (weak) {
      pending_start.reset();  // (a run under the threshold says nothing in such noise)
    } else if (!moved) {
      if (pending_start) {
        found_start(*std::exchange(pending_start, std::nullopt));
      } else if (vs && !prev_line_vsync && line_has_longrun) {
        // field start: phase of the broad-pulse start inside this line
        double ph = (static_cast<double>(line_longrun_start) - line_start) / T;
        ph -= std::floor(ph + 0.25);  // -> [-0.25, 0.75)
        pending_start = FieldStart{std::fabs(ph) < 0.25 ? 0 : 1, line_idx};
      }
    }
    // the flywheel: no vertical sync where the field in hand should end, and none waited on (with a weak signal a
    // vertical sync is known kVertWait lines later)
    if (!pending_start) {
      if (const std::optional<FieldStart> due =
              rhythm.overdue({cur_field, field_line0}, line_idx - (weak ? kVertWait : 0))) {
        fields_coasted++;
        start_field(*due);
      }
    }
    if (o.hold) {
      const bool broad = vs || (vq_n >= 2 && (vq[(vq_n - 1) & 15] < 0.5f || vq[(vq_n - 2) & 15] < 0.5f));
      next_start = hold_line(line_start, next_start, broad);
    }
    prev_line_vsync = vs;
    line_idx++;
    line_low = 0;
    line_has_longrun = false;
    line_start = next_start;
    refresh_switches();
  }

  // A field start has been found: the field begins there, or with the flywheel where the fields' rhythm says it
  // is due (the start is put right), or the start is passed over (field_rhythm.h).
  void found_start(FieldStart s) {
    const std::optional<FieldStart> at = o.flywheel ? rhythm.found({cur_field, field_line0}, s, std_->field_lines) : s;
    if (!at) return;
    if (*at != s) starts_put_right++;
    start_field(*at);
  }

  // fpvDROID: the vertical sync of a weak signal. The broad pulses are found by how long the signal stays under
  // the threshold in one run (close_line), and in noise as large as the sync is deep no run lasts. But the
  // signal's mean over the middle of a half line says what that half line is, noise or not: at the tip's level in
  // a broad pulse, at blanking's in an equalising one, at or above it in the picture. So each line's two half
  // lines are measured (vq: 0 the tip's level, 1 blanking's), and a vertical sync is the standard's six half
  // lines (PAL five) nearer the tip, with one nearer blanking either side. It is known three lines after the
  // line it began in, still well before the field's first line of picture (vert_start has what is done with it).
  static constexpr int kVertWait = 4;  // lines the flywheel waits for it, with a weak signal
  float vq[16] = {};
  uint64_t vq_n = 0;
  void vert_line(double ls) {
    const double us = 1e-6 * fs;
    const float depth = levels.blank - levels.tip;
    for (int half = 0; half < 2; half++) {
      const double h0 = ls + half * 0.5 * T;
      const int64_t i0 = static_cast<int64_t>(std::llround(h0 + 5.5 * us)), i1 = static_cast<int64_t>(std::llround(h0 + 26.0 * us));
      float q = 1;
      if (i0 >= 0 && i1 <= static_cast<int64_t>(n) && static_cast<int64_t>(n) - i0 <= static_cast<int64_t>(kSyncMask) &&
          depth > 0.02f) {
        float s = 0;
        int c = 0;
        for (int64_t i = i0; i < i1; i += 4, c++) s += sring[static_cast<size_t>(i) & kSyncMask];
        q = (levels.sign * s / static_cast<float>(c) - levels.tip) / depth;
      }
      vq[vq_n++ & 15] = q;
    }
    if (vq_n < 12) return;
    // the half lines so far end with this line's second; a vertical sync that began with the line three back
    // (odd) or in its middle (even) has all of its broad pulses, and the half line after them, in by now
    const int nb = std_ == &kPal ? 5 : 6;
    const auto broad = [&](uint64_t back) { return vq[(vq_n - 1 - back) & 15] < 0.5f; };
    for (int even = 0; even < 2; even++) {
      const uint64_t first = static_cast<uint64_t>(7 - even);  // half lines back from the newest
      bool ok = !broad(first + 1) && !broad(first - static_cast<uint64_t>(nb));
      for (int k = 0; k < nb && ok; k++) ok = broad(first - static_cast<uint64_t>(k));
      if (ok) {
        vert_start(FieldStart{even, line_idx - 3});
        return;
      }
    }
  }


  // A vertical sync found that way: the field begins there, or where the fields' rhythm has it, or nothing is done
  // (FieldRhythm::seen has when). Without the flywheel it begins a field wherever the other way has not.
  void vert_start(FieldStart s) {
    const FieldStart in_hand{cur_field, field_line0};
    if (!o.flywheel) {
      if (in_hand != s) start_field(s);
      return;
    }
    const std::optional<FieldStart> at = rhythm.seen(in_hand, s, std_->field_lines, weak, faint());
    if (!at) return;
    if (*at != s) starts_put_right++;
    start_field(*at);
  }

  // How much noise the sync path has: the samples of each line's back porch about their own mean (flat in every
  // kind of line; the burst does not get through that path), mean square, and of the last 32 lines' the one in
  // the middle (a jump of the levels inside a few lines' porches is not noise: one transmitter jumps 14 times a
  // second with a light flashing in the picture). And the sync depth of better times, while nearly every line
  // finds its hsync: a weak signal does not show it (the discriminator is then captured by the noise part of the
  // time, and tip and blanking both shrink towards the middle of the band). For the front end's channel filter
  // (DecoderStats::sync_snr, depth_ref).
  static constexpr int kNoiseLines = 32;
  // The burst of a signal that noisy is not one to take colour from: its phase is mostly the noise's (the noise of
  // an FM signal grows with the frequency, and the burst sits at the top of the band), and the picture gets bands
  // of green and purple over its snow, where grey snow is the easier to see through.
  static constexpr float kColourOff = 12, kColourOn = 16;
  bool colour_quiet = true;
  float colour_noisy = 0, colour_unclean = 0;  // how much of the time lately the sync was noisier than each
  float line_var[kNoiseLines] = {};
  int sync_var_n = 0, line_var_at = 0;
  float depth_ref = 0;
  float sync_var() const {
    float v[kNoiseLines];
    std::copy(line_var, line_var + kNoiseLines, v);
    std::nth_element(v, v + kNoiseLines / 2, v + kNoiseLines);
    return v[kNoiseLines / 2];
  }
  void noise_line(double ls) {
    const double us = 1e-6 * fs;
    const int64_t i0 = static_cast<int64_t>(std::llround(ls + 5.6 * us)), i1 = static_cast<int64_t>(std::llround(ls + 8.6 * us));
    if (i0 < 0 || i1 > static_cast<int64_t>(n) || static_cast<int64_t>(n) - i0 > static_cast<int64_t>(kSyncMask)) return;
    float s = 0, ss = 0;
    for (int64_t i = i0; i < i1; i++) {
      const float v = sring[static_cast<size_t>(i) & kSyncMask];
      s += v;
      ss += v * v;
    }
    const float cnt = static_cast<float>(i1 - i0);
    line_var[line_var_at] = std::max(0.f, ss / cnt - (s / cnt) * (s / cnt));
    line_var_at = (line_var_at + 1) % kNoiseLines;
    sync_var_n = std::min(sync_var_n + 1, kNoiseLines);
    if (line_var_at == 0 && sync_var_n == kNoiseLines && levels.blank - levels.tip > 0.05f) {
      // Colour only from a signal whose sync is not too noisy (burst). Judged over a tenth of a second or so, as
      // the front end's channel filter is: out when the sync was noisier than kColourOff for more than half of
      // it, back in when it was cleaner than kColourOn for four fifths.
      const float snr = (levels.blank - levels.tip) / std::sqrt(std::max(sync_var(), 1e-6f));
      colour_noisy += 0.02f * ((snr < kColourOff ? 1.f : 0.f) - colour_noisy);
      colour_unclean += 0.02f * ((snr < kColourOn ? 1.f : 0.f) - colour_unclean);
      colour_quiet = colour_quiet ? colour_noisy <= 0.5f : colour_unclean < 0.2f;
    }
    // (lines >= 128: the first 64 lines after an acquisition count as found before they are)
    if (!weak && lines >= 128 && found64() >= 58 && levels.blank - levels.tip > 0.2f)
      depth_ref += (depth_ref > 0 ? 1.f / 16 : 1.f) * (levels.blank - levels.tip - depth_ref);
  }

  // The line that began at ls into the fold.
  void fold_line(double ls) {
    const int64_t i0 = static_cast<int64_t>(std::llround(ls)) - fold_before;
    const size_t len = fold.size();
    if (i0 < 0 || static_cast<int64_t>(n) - i0 > static_cast<int64_t>(kSyncMask)) return;
    const float a = 1.f / static_cast<float>(std::min(fold_n + 1, kFoldLines)), s = levels.sign;
    float sq = 0;
    // (in one stretch of the ring, or two where it wraps: loops the compiler can vectorise)
    const size_t at = static_cast<size_t>(i0) & kSyncMask, first = std::min(len, kSyncMask + 1 - at);
    const float *src = sring.data() + at;
    float *f = fold.data();
    for (size_t j = 0; j < first; j++) {
      const float d = s * src[j] - f[j];
      sq += d * d;
      f[j] += a * d;
    }
    src = sring.data();
    f += first;
    for (size_t j = 0; j < len - first; j++) {
      const float d = s * src[j] - f[j];
      sq += d * d;
      f[j] += a * d;
    }
    sq /= static_cast<float>(len);
    fold_var = fold_n == 0 ? 0 : fold_n == 1 ? sq : fold_var + std::max(a, 1.f / 16) * (sq - fold_var);
    fold_n = std::min(fold_n + 1, kFoldLines);
  }

  // Is there an hsync in the fold? One by its shape: a stretch as long as a sync tip that lies lower than the
  // porches before and behind it, by more than the fold's noise would make it anywhere, with the two porches at
  // one level. Where its edges are is then measured by area (how much of a stretch around each edge is up at the
  // porch's level), which takes every sample of the edge into the count: fold_pos, with fold_tip and fold_blank.
  bool fold_look() {
    if (fold_n < kFoldMin) return false;
    const double us = 1e-6 * fs;
    const int len = static_cast<int>(fold.size());
    for (int j = 0; j < len; j++) fold_ps[j + 1] = fold_ps[j] + fold[j];
    const auto mean = [&](int a, int b) { return (fold_ps[b] - fold_ps[a]) / static_cast<float>(b - a); };
    const auto at = [&](double t) { return static_cast<int>(std::lround(t * us)); };
    const int w = at(4.5);
    float best = 0, tp = 0, bk = 0;
    int e = -1;
    for (int k = fold_before - w; k <= fold_before + w; k++) {
      float t, b;
      const float r = pulse_at(mean, at, k, t, b);
      if (r > best) best = r, e = k, tp = t, bk = b;
    }
    const float noise = fold_noise();
    double edge;
    if (e < 0 || best < 2.5f * noise || best < 0.02f || !edge_of(mean, at, e, tp, bk, edge)) return false;
    fold_pos = edge - fold_before;
    // (a pulse that is not where the grid has the hsync has to stand out further: in noise alone the fold shows
    // one somewhere in its 9 us every second or two, by chance)
    if (std::fabs(fold_pos - fold_ref) > kTightUs * us && best < 4 * noise) return false;
    fold_tip = tp;
    fold_blank = bk;
    return true;
  }
  // The noise in one place of a fold of single samples over kFoldLines lines
  float fold_noise() const {
    return std::sqrt(fold_var / static_cast<float>(fold_n < kFoldLines ? std::max(fold_n, 1) : 2 * kFoldLines));
  }

  // fpvDROID: ... and when the signal has moved. The fold looks 4.5 us either side of the grid. A block of samples
  // lost on the way here (the phone drops one a few times a minute) puts the hsyncs 14 us from it: with a good
  // signal six of them in a row re-phase the grid (on_hsync), with a weak one no six lines find theirs, the fold
  // shows nothing, and the lock would be given up after kHoldS and looked for again. So while the fold holds the
  // grid, the whole line is folded as well, coarsely (every fourth sample will do at the sync path's 1 MHz), and
  // when the fold has shown no hsync for a few milliseconds that one is looked at: an hsync by its shape, the
  // lowest thing in the line, well out of the noise, away from the grid, and in the same place at two looks 32
  // lines or more apart, moves the grid there.
  static constexpr int kWideStep = 4;
  std::vector<float> wide, wide_ps;
  int wide_n = 0;
  double wide_at = 0;         // where the last look had an hsync, samples from the grid ...
  int64_t wide_line = 0;      // ... and at which line (0: none)
  void wide_fold(double ls) {
    const size_t K = wide.size();
    const int64_t i0 = static_cast<int64_t>(std::llround(ls));
    if (i0 < 0 || i0 + static_cast<int64_t>(K) * kWideStep > static_cast<int64_t>(n) ||
        static_cast<int64_t>(n) - i0 > static_cast<int64_t>(kSyncMask))
      return;
    const float a = 1.f / static_cast<float>(std::min(wide_n + 1, kFoldLines)), s = levels.sign;
    for (size_t k = 0; k < K; k++)
      wide[k] += a * (s * sring[static_cast<size_t>(i0 + static_cast<int64_t>(k) * kWideStep) & kSyncMask] - wide[k]);
    wide_n = std::min(wide_n + 1, kFoldLines);
  }
  // How far from the grid the whole line's fold has an hsync, samples (the nearer way round); false: none.
  bool wide_look(double &off) {
    if (wide_n < kFoldLines / 2) return false;
    const int K = static_cast<int>(wide.size());
    const double us = 1e-6 * fs;
    for (int k = 0; k < 3 * K; k++) wide_ps[static_cast<size_t>(k + 1)] = wide_ps[static_cast<size_t>(k)] + wide[static_cast<size_t>(k % K)];
    const auto mean = [&](int a, int b) {
      return (wide_ps[static_cast<size_t>(K + b)] - wide_ps[static_cast<size_t>(K + a)]) / static_cast<float>(b - a);
    };
    const auto at = [&](double t) { return static_cast<int>(std::lround(t * us / kWideStep)); };
    float best = 0, lowest = 1e30f, tp = 0, bk = 0;
    int e = -1;
    for (int k = 0; k < K; k++) {
      float t, b;
      const float r = pulse_at(mean, at, k, t, b);
      lowest = std::min(lowest, t);
      if (r > best) best = r, e = k, tp = t, bk = b;
    }
    double edge;
    const float noise = std::sqrt(fold_var / static_cast<float>(2 * kFoldLines));
    if (e < 0 || best < 4 * noise || best < 0.02f || tp > lowest + 0.25f * best || !edge_of(mean, at, e, tp, bk, edge))
      return false;
    off = std::remainder(edge * kWideStep - fold_ref, T);
    return true;
  }

  // In a fold (mean(a, b): its mean over bins [a, b); at(us): so many us in bins), how much like an hsync that
  // begins at bin k it looks there: how far a stretch as long as most of a sync tip lies under what is before it
  // and behind it (tip, blank: its level, and the back porch's). Under the lower of the two: the front porch is
  // short, the sync path's filter runs the end of the picture into it, and a bright picture there must not make a
  // place a microsecond early look the better one.
  template <class Mean, class At>
  static float pulse_at(const Mean &mean, const At &at, int k, float &tip, float &blank) {
    const float front = mean(k + at(-0.9), k + at(-0.3));
    tip = mean(k + at(0.8), k + at(4.0));
    blank = mean(k + at(5.6), k + at(8.6));
    return std::min(front, blank) - tip;
  }

  // ... and where that hsync's leading edge is (bins, to a fraction; e: where pulse_at had it, which is anywhere
  // within a microsecond). First to the bin where the fold goes under half way between the two levels, then by
  // area: how much of a stretch around the edge is up at blanking's level (the back porch's: the longer and the
  // cleaner of the two), which takes every sample of the edge into the count. That edge alone, against the tip's
  // level over the pulse's first 2 us: the equalising and the broad pulses of the vertical interval are in the
  // fold with the hsyncs, and they begin as an hsync does and end otherwise.
  template <class Mean, class At>
  static bool edge_of(const Mean &mean, const At &at, int e, float tip, float blank, double &edge) {
    const float mid = 0.5f * (tip + blank);
    const int w = std::max(1, at(0.15));
    int c = e + at(-1.5);
    const int end = e + at(1.0);
    while (c < end && !(mean(c, c + w) < mid && mean(c + w, c + 2 * w) < mid)) c++;
    if (c >= end) return false;
    // (twice: in noise the fold first goes under half way a little early, and the stretch is then put again around
    // where the area has the edge)
    for (int pass = 0; pass < 2; pass++) {
      const int l0 = c + at(-0.7), l1 = c + at(1.0);
      const float low = mean(l1, c + at(2.1));
      if (blank - low < 0.5f * (blank - tip)) return false;
      edge = l0 + static_cast<double>(mean(l0, l1) - low) / (blank - low) * (l1 - l0);
      const int again = static_cast<int>(std::lround(edge));
      if (again == c || std::abs(again - c) > at(1.0)) break;
      c = again;
    }
    return true;
  }

  // The line idx begins at start, and the grid is right there (most lines find their hsync, or the fold shows it):
  // a place to come back to, and one end of a long stretch of lines by which the line period is known far better
  // than the lines' own tracking has it: (start - where the stretch began) / lines.
  void grid_sure(double start, uint64_t idx) {
    sure_start = start;
    sure_idx = idx;
    if (base_start < 0 || idx <= base_idx) {
      base_start = start;
      base_idx = idx;
      mid_start = -1;
      return;
    }
    const uint64_t span = idx - base_idx;
    if (span >= kLongMin) {
      T_long = (start - base_start) / static_cast<double>(span);
      long_ok = true;
    }
    if (mid_start < 0 && span >= kLongSpan / 2) mid_start = start, mid_idx = idx;
    if (span >= kLongSpan) {  // the stretch moves on, so that the period follows a crystal that drifts
      base_start = mid_start;
      base_idx = mid_idx;
      mid_start = start;
      mid_idx = idx;
    }
  }

  // The grid has been moved by c samples: so has where the fold has everything, the other way.
  void fold_moved(double c) {
    fold_carry += c;
    const int k = static_cast<int>(fold_carry);
    if (k == 0) return;
    fold_carry -= k;
    const int len = static_cast<int>(fold.size());
    if (std::abs(k) >= len) {
      fold_n = 0;
    } else if (k > 0) {
      std::memmove(fold.data(), fold.data() + k, static_cast<size_t>(len - k) * sizeof(float));
      std::fill(fold.end() - k, fold.end(), fold[static_cast<size_t>(len - k - 1)]);
    } else {
      std::memmove(fold.data() - k, fold.data(), static_cast<size_t>(len + k) * sizeof(float));
      std::fill(fold.begin(), fold.begin() - k, fold[static_cast<size_t>(-k)]);
    }
  }

  // A line has ended (it began at ls; next: where the one after it begins): it goes into the fold, and with a weak
  // signal the fold steers. Returns where the next line begins.
  double hold_line(double ls, double next, bool broad) {
    if (!broad) fold_line(ls);  // (a line of broad pulses has no porch behind its pulse)
    noise_line(ls);
    weak_ago = weak ? 0 : std::min(weak_ago + 1, 1 << 30);
    if (!weak) {
      if (lines >= 64 && found64() < 32) return go_weak(next);
      if (++fold_due >= 16) {
        fold_due = 0;
        const bool seen = fold_look();
        const double us = 1e-6 * fs;
        if (seen && found64() >= 58 && fold_pos > kFoldRefMinUs * us && fold_pos < kFoldRefMaxUs * us)
          fold_ref += std::max(0.05, 1.0 / ++fold_ref_n) * (fold_pos - fold_ref);
        fold_good = seen && std::fabs(fold_pos - fold_ref) < 0.5 * us;
      }
      return next;
    }
    if (found64() >= 60) {
      KP_LOGD("video: the lines find their hsync again, after %d lines held by the fold (tip %.2f blank %.2f, noise %.2f)",
              weak_lines, levels.tip, levels.blank, std::sqrt(fold_var));
      weak = false;
      fold_due = 0;
      fold_good = false;
      return next;
    }
    weak_lines++;
    unseen++;
    wide_fold(ls);
    if (++fold_due < 4) return next;
    fold_due = 0;
    const bool seen = fold_look();
    // Has the signal moved (a block of samples lost on the way here)? Then the whole line's fold has the hsync
    // away from the grid, at two looks 32 lines apart or more. Looked for when the fold at the grid has shown
    // nothing for 128 lines, and every 64 lines whatever it shows: after such a jump a dark edge in the picture
    // can pass for a pulse there, with levels of its own that the kept ones then follow, and the grid stayed on
    // it for good (the hsync, lower than anything in the picture, is the whole line's best pulse all the same).
    if (seen ? weak_lines % 64 == 0 : unseen >= 128 && unseen % 16 == 0) {
      double off;
      if (wide_look(off) && std::fabs(off) >= 4.5e-6 * fs) {
        if (wide_line != 0 && line_idx - wide_line >= 32 && std::fabs(off - wide_at) < 0.5e-6 * fs) {
          KP_LOGI("video: re-phased the line grid by %+.2f us at line %lld of the field, by the fold of the lines (%s)",
                  off / fs * 1e6, line_in_field(), seen ? "what it had at the grid was no hsync" : "no hsync at the grid");
          rephases++;
          comb_epoch++;
          rhythm.doubt();
          fold_n = wide_n = fold_due = unseen = 0;
          wide_line = 0;
          sure_start = base_start = -1;
          vq_n = 0;
          return next + off;
        }
        if (wide_line == 0 || std::fabs(off - wide_at) >= 0.5e-6 * fs) wide_at = off, wide_line = line_idx;
      } else if (seen) {
        wide_line = 0;
      }
    }
    if (!seen) return next;
    unseen = 0;
    const double err = fold_pos - fold_ref;
    const double c = kSteer * err;
    if (std::fabs(err) < 0.5e-6 * fs) grid_sure(next + c, static_cast<uint64_t>(line_idx) + 1);
    if (long_ok) T = T_long;
    else T += err * (1.0 / 8192);
    fold_moved(c);
    // (the levels are measured ones from here on: no more of the histogram's)
    levels.set(levels.tip + 0.5f * (fold_tip - levels.tip), levels.blank + 0.5f * (fold_blank - levels.blank));
    return next + c;
  }

  // Fewer than half of the lines find their hsync: the fold steers from here on. The lines of the way down here
  // may have been pulled about by pulses that were noise, and the line period with them: both go back to what
  // they were when most lines still found theirs, if that is not long ago.
  double go_weak(double next) {
    weak = true;
    holds++;
    weak_lines = unseen = fold_due = 0;
    wide_n = 0;
    wide_line = 0;
    fold_carry = 0;
    if (long_ok) T = T_long;
    // (not if the fold shows the pulse where the grid is now)
    const uint64_t at = static_cast<uint64_t>(line_idx) + 1;  // the line that begins at next
    const bool back = long_ok && sure_start >= 0 && at - sure_idx <= 256 &&
                      !(fold_look() && std::fabs(fold_pos - fold_ref) < 3e-6 * fs);
    if (back) {
      next = sure_start + static_cast<double>(at - sure_idx) * T;
      fold_n = 0;  // what is in the fold was folded on the grid as it was pulled about
    }
    KP_LOGD("video: fewer than half of the lines find their hsync: held by the fold%s (line %lld, tip %.2f blank %.2f)",
            back ? ", from the grid as it was" : "", static_cast<long long>(line_idx), levels.tip, levels.blank);
    return next;
  }


  // A field begins, at line s.line (the first of its vertical sync): the one before it is done.
  void start_field(FieldStart s) {
    // smooth lines: A is the lower field, B the upper one (B's line k is drawn above A's line k)
    if (cur_field == 0) haveA = true;
    if (cur_field == 1) haveB = true;
    if (o.fields && cur_field >= 0) field_done();
    if (!o.fields && haveA && haveB) {
      emit(fieldA, fieldB);
      haveA = haveB = false;
    }
    cur_field = s.kind;
    field_line0 = s.line;
    fields_seen++;
    if (s.kind == 0) haveB = false;  // a frame is A then B
    begin_field();
  }

  // Is a pulse that begins at edge the third or later of a run of strays a line apart?
  bool in_a_run(double edge) const {
    return stray_run >= 2 && last_stray >= 0 && std::fabs(edge - last_stray - T) < 0.03 * T;
  }

  // An hsync-width pulse has ended: it began at edge and was len samples long.
  void on_hsync(double edge, double len) {
    if (line_start < 0) {
      line_start = edge;
      return;
    }
    const double pred = line_start + T;
    // Where the hsync began. A dip in the signal just ahead of the pulse (a click, the black edge of an OSD
    // character at the end of the picture) runs into it, and the pulse seems to begin early; one inside it
    // splits it, and the part that is left begins late. The pulse's end is then still right, and where the hsync
    // began follows from that and from how long hsyncs are (hs_len); with a dip behind the pulse it is the other
    // way round. So when the pulse is not as long as hsyncs are (by more than 0.1 us: they vary by a sample), of
    // the two, its own beginning and its end less hs_len, the one nearer to where the hsync was due is taken.
    // (Taken by its beginning alone, a pulse that began 0.85 us early moved the grid 0.5 us at once, 7 pixels,
    // for the lines after it to pull back.)
    const double by_end = edge + len - hs_len;
    const bool odd = hs_len > 0 && std::fabs(len - hs_len) > 0.1e-6 * fs;
    const double at = odd && std::fabs(by_end - pred) < std::fabs(edge - pred) ? by_end : edge;
    if (weak) {
      // Held by the fold (see kFoldLines): a pulse where the grid has the hsync counts as that line's, and steers
      // nothing. One further off is a stray like any other.
      if (std::fabs(at - pred) <= kTightUs * 1e-6 * fs) {
        miss_run = 0;
        stray_run = 0;
        end_line(pred, true);
        return;
      }
    } else if (std::fabs(at - pred) <= (fold_good && found64() < 58 && !in_a_run(edge) ? kNearUs * 1e-6 * fs : 0.08 * T)) {
      // (In a signal with noise in it, one where more than a few lines in 64 miss their hsync, a pulse far from
      // the grid is no hsync while the fold shows the hsync where the grid is: at six tenths of the way towards
      // each such pulse the grid was thrown about, and the line period with it. A signal that has really moved
      // brings them in a row, a line apart: the third of such a run is followed as ever (the grid a few us off
      // after a vertical interval's pulses pulled it: left to the sixth, a re-phase, the fields' rhythm went with
      // it), and six further off re-phase. A clean signal that steps is followed at once, as ever.)
      // hs_len follows the pulses a sixteenth of a sample at a time, whatever their length: one that something
      // ran into does not move it
      hs_len = hs_len > 0 ? hs_len + std::clamp(len - hs_len, -1.0, 1.0) * (1.0 / 16) : len;
      const double ns = 0.6 * at + 0.4 * pred;
      T = 0.995 * T + 0.005 * (ns - line_start);
      miss_run = 0;
      stray_run = 0;
      end_line(ns, true);
      return;
    }
    // an hsync-width pulse off the grid: noise, an equalising pulse, or the picture has jumped
    if (last_stray >= 0 && std::fabs(edge - last_stray - T) < 0.03 * T) stray_run++;
    else stray_run = 1;
    last_stray = edge;
    if (stray_run >= 6 && miss_run >= 4) {
      rephases++;
      comb_epoch++;
      if (faint()) rhythm.doubt();
      else rhythm.forget();
      // In the log every time, with how far the grid moved (the shorter way round), because that says why. A
      // block of samples lost on the way here (the decode queue drops 4096 at a time, the stream a bank, which
      // is as many) moves it by that many samples less whole lines: -14.1 us for each at 20 MS/s NTSC, -12.8 us
      // PAL. A stream restart moves it by anything. A small move the late way (+) with nothing lost is hsyncs
      // that came later than their line.
      KP_LOGI("video: re-phased the line grid by %+.2f us at line %lld of the field, after %d lines without an hsync",
              std::remainder(edge - line_start, T) / fs * 1e6, line_in_field(), miss_run);
      line_start = edge;
      line_low = 0;
      line_has_longrun = false;
      stray_run = 0;
      miss_run = 0;
      // six hsyncs running: the signal is no weak one. What the fold holds was folded on the old grid.
      weak = false;
      found_bits = ~0ull;
      fold_n = fold_due = 0;
      fold_good = false;
      sure_start = base_start = -1;
      vq_n = 0;
    }
  }
};

// ---- front end ----------------------------------------------------------------------------------------
// fpvDROID: block-wise and branch-free so the compiler vectorises it (the spike's per-sample loop used a whole
// phone core at 40 MS/s). Per chunk of <= kTab samples: DC removal (block mean, smoothed), mix by a
// precomputed rotation table, halfband /2 over a work array with 22 samples of history, discriminator.
FrontEnd::FrontEnd(double fs_in, double shift_hz, FrameSink sink, const DecoderOpts &o, const PictureOpts &p)
    : _fs_in(fs_in), _fs_out(fs_in >= 40e6 ? fs_in / 2 : fs_in), _decimate(fs_in >= 40e6), _picture(p),
      _cf(o.channel_filter && o.hold) {
  _cf_r.assign(kCf - 1, 0.f);
  _cf_i.assign(kCf - 1, 0.f);
  for (int i = 0; i < kHb; i++) {
    const int k = i - kHb / 2;
    const double w = 0.5 - 0.5 * std::cos(2 * M_PI * (i + 0.5) / kHb);
    _h[i] = k == 0 ? 0.5f : static_cast<float>(std::sin(M_PI * k / 2) / (M_PI * k) * w);
  }
  _wr.assign(kHb - 1 + kTab, 0.f);
  _wi.assign(kHb - 1 + kTab, 0.f);
  _dec = new Decoder(_fs_out, std::move(sink), o, _picture);
  _clicks = new ClickDetector(_fs_out);
  set_shift(shift_hz);
}

FrontEnd::~FrontEnd() {
  delete _clicks;
  delete _dec;
}

void FrontEnd::set_shift(double s) {
  _shift = s;
  tune(_shift + _cf_centre * 1e6);
}

// The mixer brings hz to 0.
void FrontEnd::tune(double hz) {
  _tr.resize(kTab);
  _ti.resize(kTab);
  for (int i = 0; i < kTab; i++) {
    const double a = -2 * M_PI * hz / _fs_in * i;
    _tr[i] = static_cast<float>(std::cos(a));
    _ti[i] = static_cast<float>(std::sin(a));
  }
  const double a = -2 * M_PI * hz / _fs_in * kTab;
  _stepN = cf(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
  _rot = cf(1, 0);
}

DecoderStats FrontEnd::stats() const {
  DecoderStats s = _dec->stats();
  s.filter_mhz = _cf_want_width.load(std::memory_order_relaxed);
  s.filter_changes = _cf_changes.load(std::memory_order_relaxed);
  return s;
}

// decode()'s side of the channel filter: in or out, by how noisy the decoder finds the sync, and how wide and
// where, by the levels it has. In whole MHz and quarters of one, so that it is not retuned for nothing.
void FrontEnd::steer_channel_filter(size_t samples) {
  const DecoderStats s = _dec->stats();
  const float width = _cf_want_width.load(std::memory_order_relaxed);
  const double block = static_cast<double>(samples) / _fs_out;  // seconds
  if (s.sync_snr <= 0) {
    // No lock, or no levels yet. The filter stays as it is, which a weak signal needs to be found again; but not
    // for long, in case it is the filter that is in the way.
    if (s.acquiring && width > 0 && (_cf_lost += block) > kCfLostS) {
      _cf_want_width.store(0, std::memory_order_relaxed);
      _cf_at_n = 0;
      _cf_noisy = _cf_unclean = 0;
    }
    return;
  }
  _cf_lost = 0;
  // Where the middle of the signal's swing is: 1.75 sync depths above the tip, with the tip and the depth as the
  // decoder has them now. A weak signal's levels are not the transmitter's: they are pulled towards the middle of
  // whatever band reaches the discriminator, which is the filter's own middle once it is in, and the depth
  // shrinks by as much. Tip and depth as they are measured still lead there: each time the filter is put where
  // they say, it is that much nearer (by the part of the swing that is left), and it never runs past. (With a
  // depth from elsewhere under a shrunk tip it runs away: every step pulls the tip further.) Smoothed over
  // kCfSmoothS, so that levels that jump with the picture do not have the filter chasing them.
  const float depth = s.blank - s.tip, at = s.tip + 1.75f * depth;
  const float sm = _cf_at_n++ == 0 ? 1.f : static_cast<float>(std::min(1.0, block / kCfSmoothS));
  _cf_at += sm * (at - _cf_at);
  _cf_now += sm * (depth - _cf_now);
  // How wide: by the transmitter's sync depth, which is the deepest seen (a weak signal's only ever shrinks; the
  // smoothed one, or the noise in a weak signal's levels would make it deeper and deeper)
  _cf_depth = std::max({_cf_depth * static_cast<float>(1 - block / 60), _cf_now, s.depth_ref});
  // In or out: by how much of the last tenth of a second or so the sync was noisy. In, when it was for more than
  // half of it; out again, when it was clean for four fifths, by the stricter measure. (Not by the moment: the
  // transmitter whose levels jump with a flashing light has its sync noisy for a millisecond at every jump.)
  const float a = static_cast<float>(std::min(1.0, block / kCfJudgeS));
  _cf_noisy += a * ((s.sync_snr < kCfOn ? 1.f : 0.f) - _cf_noisy);
  _cf_unclean += a * ((s.sync_snr < kCfOff ? 1.f : 0.f) - _cf_unclean);
  if (width > 0 ? _cf_unclean < 0.2f : _cf_noisy <= 0.5f) {
    if (width > 0) _cf_want_width.store(0, std::memory_order_relaxed);
    return;
  }
  // (no depth worth the name seen yet: as wide as for the transmitter that swings widest, which costs the others
  // some of the gain and breaks no picture)
  const float w = _cf_depth >= kCfMinDepth ? std::clamp(std::round(kCfDepths * _cf_depth), kCfMinMhz, kCfMaxMhz) : kCfMaxMhz;
  if (width <= 0 || std::fabs(w - width) >= 2) _cf_want_width.store(w, std::memory_order_relaxed);
  if (width <= 0 || std::fabs(_cf_at - _cf_want_centre.load(std::memory_order_relaxed)) > 0.5f)
    _cf_want_centre.store(std::round(_cf_at * 4) / 4, std::memory_order_relaxed);
}

// One half of the stream (I or Q) through the channel filter, in place; with the filter out, delayed by as much.
// hist: the kCf - 1 samples before these, which it keeps for the next call.
void FrontEnd::channel_filter(const float *s_in, int ny, std::vector<float> &out_v) {
  constexpr int Hc = kCf - 1, D = Hc / 2;
  out_v.resize(static_cast<size_t>(ny));
  const float *__restrict s = s_in;
  float *__restrict out = out_v.data();
  const float *h = _cf_h;
  for (int j = 0; j < ny; j++) {  // (the taps are the same from either end)
    float acc = h[D] * s[j + D];
    for (int t = 0; t < D; t++) acc += h[t] * (s[j + t] + s[j + Hc - t]);
    out[j] = acc;
  }
}

// demod()'s side: the filter as decode() last wanted it. Its middle goes to 0 (the mixer), a windowed-sinc
// low-pass does the rest, and the discriminator's output has the middle added back, so that the decoder's levels
// are where they were.
void FrontEnd::retune_channel_filter() {
  const float w = _cf_want_width.load(std::memory_order_relaxed);
  const float c = w > 0 ? _cf_want_centre.load(std::memory_order_relaxed) : 0;
  if (w == _cf_width && c == _cf_centre) return;
  _cf_width = w;
  _cf_centre = c;
  _cf_changes.fetch_add(1, std::memory_order_relaxed);
  if (w > 0) {
    const double fc = 0.5e6 * w / _fs_out;  // the low-pass's edge, of the sample rate
    double sum = 0;
    for (int i = 0; i < kCf; i++) {
      const double k = i - (kCf - 1) / 2.0;
      const double win = 0.42 - 0.5 * std::cos(2 * M_PI * i / (kCf - 1)) + 0.08 * std::cos(4 * M_PI * i / (kCf - 1));
      _cf_h[i] = static_cast<float>((k == 0 ? 2 * fc : std::sin(2 * M_PI * fc * k) / (M_PI * k)) * win);
      sum += _cf_h[i];
    }
    for (float &h : _cf_h) h = static_cast<float>(h / sum);
  }
  tune(_shift + _cf_centre * 1e6);
}


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

void FrontEnd::demod(const cf *xs, size_t n, std::vector<float> &out, std::vector<uint8_t> &bad) {
  out.clear();
  if (_cf) retune_channel_filter();
  const bool rep = _picture.load(std::memory_order_relaxed).repair;
  _clicks->begin(rep);
  bad.clear();
  const float k = static_cast<float>(_fs_out / (2 * M_PI) / 1e6);  // rad/sample -> MHz
  constexpr int H = kHb - 1;
  const int pad = _cf ? kCf - 1 : 0;  // the channel filter's history, in front of each chunk's samples
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
      _yr.resize(static_cast<size_t>(pad + nout));
      _yi.resize(static_cast<size_t>(pad + nout));
      float *yr_w = _yr.data() + pad, *yi_w = _yi.data() + pad;
      for (int j = 0; j < nout; j++) {
        const int s0 = j0 + 2 * j;
        float a = 0.5f * r[s0 + kHb / 2], b = 0.5f * q[s0 + kHb / 2];
        for (int t = 0; t < kHb; t += 2) {
          a += _h[t] * r[s0 + t];
          b += _h[t] * q[s0 + t];
        }
        yr_w[j] = a;
        yi_w[j] = b;
      }
      _phase = j0 + 2 * nout - m;  // where the next window starts, relative to the next chunk
      std::memmove(_wr.data(), _wr.data() + m, H * sizeof(float));
      std::memmove(_wi.data(), _wi.data() + m, H * sizeof(float));
    } else {
      _yr.resize(static_cast<size_t>(pad + m));
      _yi.resize(static_cast<size_t>(pad + m));
      std::memcpy(_yr.data() + pad, wr, static_cast<size_t>(m) * sizeof(float));
      std::memcpy(_yi.data() + pad, wi, static_cast<size_t>(m) * sizeof(float));
    }
    const int ny = static_cast<int>(_yr.size()) - pad;
    // What the discriminator reads. With the channel filter: the chunk lies behind the stream's last kCf - 1
    // samples, and is read through the filter, or with the filter out half its length early: delayed by as much
    // as the filter delays, without a sample being moved.
    const float *yr = _yr.data(), *yi = _yi.data();
    if (_cf && ny > 0) {
      constexpr int Hc = kCf - 1;
      std::memcpy(_yr.data(), _cf_r.data(), Hc * sizeof(float));
      std::memcpy(_yi.data(), _cf_i.data(), Hc * sizeof(float));
      std::memcpy(_cf_r.data(), _yr.data() + ny, Hc * sizeof(float));
      std::memcpy(_cf_i.data(), _yi.data() + ny, Hc * sizeof(float));
      if (_cf_width > 0) {
        channel_filter(_yr.data(), ny, _cf_fr);
        channel_filter(_yi.data(), ny, _cf_fi);
        yr = _cf_fr.data();
        yi = _cf_fi.data();
      } else {
        yr += Hc / 2;
        yi += Hc / 2;
      }
    }
    out.resize(o0 + ny);
    float *d = out.data() + o0;
    float lr = _prev.real(), li = _prev.imag();
    if (ny > 0) {
      d[0] = atan2_bf(yi[0] * lr - yr[0] * li, yr[0] * lr + yi[0] * li) * k;
      for (int j = 1; j < ny; j++) {
        const float re = yr[j] * yr[j - 1] + yi[j] * yi[j - 1];
        const float im = yi[j] * yr[j - 1] - yr[j] * yi[j - 1];
        d[j] = atan2_bf(im, re) * k;
      }
      _prev = cf(yr[ny - 1], yi[ny - 1]);
      if (_cf_centre != 0)
        for (int j = 0; j < ny; j++) d[j] += _cf_centre;
    }
    if (rep) _clicks->envelope(yr, yi, ny);
    xs += m;
    n -= m;
  }
  if (rep) _clicks->mark(out, bad);
}

void FrontEnd::decode(const std::vector<float> &d, const std::vector<uint8_t> &bad, double t_end) {
  _dec->push_block(d.data(), bad.size() == d.size() ? bad.data() : nullptr, d.size(), t_end);
  if (_cf) steer_channel_filter(d.size());
}

void FrontEnd::feed(const cf *xs, size_t n) {
  demod(xs, n, _dbuf, _badbuf);
  decode(_dbuf, _badbuf);
}

}  // namespace kp
