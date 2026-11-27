// fpvDROID video decoder. See video.h. The algorithm follows vtx-spike/viewer/vtxview.cpp (measured on a
// real R4 NTSC VTX: 93-96 % of lines locked); changes are marked "fpvDROID:".
#include "video.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <optional>
#include <utility>

#include "acquire.h"
#include "biquad.h"
#include "burst_lock.h"
#include "channel_filter.h"
#include "deinterlace.h"
#include "field_rhythm.h"
#include "line_draw.h"
#include "line_hold.h"
#include "log.h"
#include "picture_filters.h"
#include "rings.h"
#include "sparkle.h"
#include "sync_levels.h"
#include "sync_noise.h"
#include "vert_sync.h"

namespace kp {

// ---- the decoder: discriminator output (MHz) in, frames out ---------------------------------------------
// It finds the sync pulses (a line PLL that freewheels and re-phases, the field starts by the broad pulses' run),
// which is its own; for the rest it runs the modules: acquisition (acquire.h); the levels the pulses are found by
// (sync_levels.h); for a weak signal the fold of the lines that holds the grid (line_hold.h), its vertical sync
// (vert_sync.h) and how noisy its sync is (sync_noise.h); where a field begins when its vertical sync is not found
// (field_rhythm.h); the luma and colour paths behind the pulse tracking into the sample rings (picture_filters.h,
// rings.h), and as each line ends, its burst (burst_lock.h), sparkle repair (sparkle.h), the line into a row
// (line_draw.h) and, as each field ends, the frame (deinterlace.h); with strips (PictureOpts::strips), the frame a
// strip at a time as the rows come in.
class Decoder {
public:
  // picture: the switches, which whoever owns the decoder may change at any time (FrontEnd::set_picture)
  Decoder(double fs, FrameSink sink, const DecoderOpts &o, const std::atomic<PictureOpts> &picture)
      : fs(fs), sink(std::move(sink)), o(o), picture(picture), levels(fs), drawer(o.width, fs), burst(fs, o.hold), filt(fs, o.colour),
        spark(fs), hold(fs, o.coast ? o.coast_s : 0) {
    lp_sync.design(1.0e6, fs);
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
    s.colour = burst.on();
    s.burst_err = burst.err();
    s.clicks_repaired = spark.clicks_repaired;
    s.lines_mended = spark.lines_mended;
    s.holding = hold.holding();
    s.holds = hold.holds();
    s.sync_snr = sync_snr();
    s.depth_ref = depth_ref;
    if (!acquiring && line_start >= 0) {
      s.grid = line_start;
      s.field = cur_field;
      if (cur_field >= 0) s.field0 = line_start - static_cast<double>(line_idx - field_line0) * T;
    }
    return s;
  }

  // The sync as the front end's channel filter steers by it (channel_filter.h).
  ChannelFilter::Sync sync() const { return {acquiring, sync_snr(), levels.tip, levels.blank, depth_ref}; }

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
      if (o.hold) sync_ring.put(n, mb, pn);
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
  BurstLock burst;  // fpvDROID: colour. The reference the lines' bursts give (PAL: and each line's V switch)
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
  // fpvDROID: a weak signal (DecoderOpts::hold). The sync path's samples are kept (sync_ring) for three things
  // that look at each line's sync once the line has ended: the fold of the lines, which holds the grid when the
  // lines' own hsyncs are no longer found (hold); the vertical sync by what its half lines are (vert); and how
  // noisy the sync is (noise).
  SyncRing sync_ring;
  LineHold hold;
  VertSync vert;
  SyncNoise noise;
  float snr_now = 0;  // the sync's snr as noise last judged it (every SyncNoise::kLines lines; 0: not known)
  // The sync depth of better times, while nearly every line finds its hsync: a weak signal does not show it (the
  // discriminator is then captured by the noise part of the time, and tip and blanking both shrink towards the
  // middle of the band). For the front end's channel filter (sync).
  float depth_ref = 0;
  SyncSamples sync_now() const { return {sync_ring, n}; }  // the sync path's samples as the pulse tracking has them
  // Is the signal a faint one: held by the fold now, or within the last two fields? Such a signal shows the
  // vertical sync of few of its fields (one in nine, in one recording): see FieldRhythm::doubt and Signal.
  bool faint() const { return o.hold && hold.lately(2 * std_->field_lines); }
  FieldRhythm::Signal signal() const {
    return hold.holding() ? FieldRhythm::Signal::held : faint() ? FieldRhythm::Signal::faint : FieldRhythm::Signal::steady;
  }

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
    LineLevels sync;
    if (o.follow_levels && !hold.holding()) {  // (held by the fold, the levels are the fold's)
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
    T = s.T_us * 1e-6 * fs;
    hold.set_std(T);
    fieldA.assign(static_cast<size_t>(o.width) * s.window.lines * 4, 0);
    fieldB = fieldA;
    frame.assign(static_cast<size_t>(o.width) * s.window.frame_lines() * 4, 0);
    row_t.assign(s.window.lines, 0);
    part_y = 0;
    filt.set_std(s);
    refresh_switches();
    comb_epoch++;
    drawer.set_std(s, filt.comb_w());
    burst.set_std(&s == &kPal);
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
    hold.count(locked);
  }

  // fpvDROID: lost when fewer than 40 % of the last ~2 fields' lines found their hsync (a healthy picture
  // locks ~90 %: only the vertical interval freewheels). The spike used the ratio since acquisition. With the
  // hold, which takes such a signal over long before: when the fold has not shown the hsync for half a second,
  // or coasting for as long as the lock had lasted (LineHold::lost).
  void check_lost() {
    if (hold.holding()) {  // the fold has the say
      if (hold.lost(T, lines)) {
        losses++;
        KP_LOGI("video: lost lock (no hsync in the fold for %.1f s), re-acquiring", hold.silent_s(T));
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
    hold.restart();
    vert.reset();
    noise.restart();
    snr_now = 0;
  }

  // acquisition: pick polarity + levels + line period from a 60 ms buffer
  void acquire() {
    float best = 0, bt = 0, bb = 0, bs = 0;
    for (float sgn : {+1.f, -1.f}) {
      if (o.force_invert && sgn > 0) continue;
      float t, b;
      if (!levels.survey(acq, acq_n, sgn, t, b)) continue;
      const std::optional<PulseScore> ps = score_pulses(acq, acq_n, fs, sgn, 0.5f * (t + b));
      if (!ps) continue;
      KP_LOGD("video acq: sign %+.0f tip %.2f blank %.2f, %zu pulses, score %.2f", sgn, t, b, ps->pulses, ps->score);
      if (ps->score > best) {  // (so some pulses came a line apart: there is a period)
        best = ps->score;
        bs = sgn;
        bt = t;
        bb = b;
        const double Tus = ps->T_us;
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
      const std::optional<double> known = o.hold ? hold.period() : std::nullopt;
      if (known && std::fabs(T - *known) < 0.25) T = *known;
      else hold.forget_period();
      KP_LOGI("video: acquired, sync %s end, tip %.2f blank %.2f MHz, line %.3f us -> %s, score %.2f",
              bs > 0 ? "low" : "high", bt, bb, T / fs * 1e6, std_->name, best);
    }
    // fpvDROID: a signal too weak for the above is acquired by the fold of its lines (acquire.h)
    if (acquiring && o.hold)
      if (const std::optional<FoldLock> got = acquire_by_fold(acq, acq_n, fs, levels.sign, o.std_force, std_, hold.period()))
        acquired_by_fold(*got);
    acq_n = 0;
  }

  // ... and the fold holds its grid from the first line on.
  void acquired_by_fold(const FoldLock &got) {
    if (got.std != std_) set_std(*got.std);
    T = got.T;
    levels.set(got.tip, got.blank);  // (measured levels: none of the histogram's after this)
    acquiring = false;
    acquisitions++;
    // sample j of the buffer is sample n - 1 - acq_n + j of the signal; an hsync is found a little before where
    // the fold has its edge (LineHold::ref)
    line_start = static_cast<double>(n - 1 - acq_n) + got.edge - hold.ref();
    hold.acquired();
    KP_LOGI("video: acquired by the fold of the lines (%d of %d parts), tip %.2f blank %.2f MHz, line %.4f us -> %s",
            got.parts, kAcqParts, levels.tip, levels.blank, T / fs * 1e6, std_->name);
  }

  bool in_picture(int64_t k) const { return k >= 0 && k < std_->window.lines; }

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
    if (locked && !own && !hold.holding()) levels.measure(R, ls);
    if (filt.luma_comb()) top_line(ls, k);
    l.hit = pic.repair && patch_line(ls);
    // (in colour: a line whose hsync was found, or one the hold has the start of, for a noisy signal's colour)
    l.col = o.colour && (locked || (hold.holding() && burst.averaging())) &&
            burst.line(R, ls, line_idx, levels.blank - levels.tip, l.pal_v);
    l.ok = l.col && burst.last_err() < kCombMaxErr;
    l.phase = burst.phase();
    l.mag = burst.size();
    l.calm = l.col ? burst.calm() : 0;
    l.fade = filt.fade_noted();
    return l;
  }

  // A line has ended: it is drawn, or with the 3-line comb the one before it is.
  void render_line(double ls, bool locked, const std::optional<LevelPair> &own) {
    if (cur_field < 0) return;
    const LineInfo cur = measure(ls, line_idx - field_line0 - std_->window.first_line, locked, own);
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
    const double a0 = std_->window.start_us * 1e-6 * fs, aw = std_->window.width_us * 1e-6 * fs;
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
    filt.redo_without_comb(R, ls + a0, aw, fix_end(ls), whole);
  }

  // Sparkle repair (sparkle.h): the picture of the line that starts at ls patched in the rings, before it is
  // drawn; true when it is still badly hit, which mend_rows is told once the row (k) is drawn.
  bool patch_line(double ls) {
    const double a0 = std_->window.start_us * 1e-6 * fs, aw = std_->window.width_us * 1e-6 * fs;
    return spark.patch_line(R, ls + a0, aw, fix_end(ls), T, filt.luma_comb(), filt.colour_fades());
  }

  // How far the picture's fixes (top_line's redo, sparkle repair) may change the luma of the line that starts at
  // ls: short of the next line's sync, by as much as SyncLevels stops reading that line's front porch ahead of
  // its hsync. The window ends close to that hsync, and a patch that ramped onto its edge would tilt the reading.
  double fix_end(double ls) const { return ls + T - SyncLevels::kFrontUs[1] * 1e-6 * fs; }
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
    const int W = o.width, A = std_->window.lines;
    y1 = std::min(y1, std_->window.frame_lines());
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
    const int r0 = std::max(y0 >> 1, 0), r1 = std::min((y1 - 1) >> 1, std_->window.lines - 1);
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
  void field_done() { put_part(std_->window.frame_lines()); }
  // A frame woven from two fields (not DecoderOpts::fields)
  void emit(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
    weave(a.data(), b.data(), o.width, std_->window.lines, frame.data());
    emit_frame(0, std_->window.frame_lines(), {});
  }
  // The frame to the sink, its lines [y0, y1) new
  void emit_frame(int y0, int y1, const LineTimes &t) {
    const int H = std_->window.frame_lines();
    if (y1 >= H) frames_out++;
    if (sink) sink(Frame{frame.data(), o.width, H, y0, y1, t});
  }

  // The line in hand is closed, and the next begins at next_start. moved: its sync was at other levels than the
  // kept ones (end_line).
  void close_line(double next_start, bool moved) {
    // A line whose levels have moved was low against a threshold that was no longer right: it starts no field.
    // Nor does the line before the first of them, the one the signal dropped in, low from there to its end (the
    // picture's black under the old threshold). So a field start waits for the line after it, in a vertical sync
    // another of its lines, and while lines say the levels have moved: when they have (end_line), it is dropped.
    const bool vs = line_low > 0.5 * T;
    if (o.hold) {  // the vertical sync by what its half lines are, which shows in noise
      // (not in a dropout: its half lines are noise, and the fields go by their rhythm, where there is one to go
      // by: the flywheel's, with a field in hand. Reset, it also calls none of the dropout's lines one of broad
      // pulses (broad, below), so all of them go into the fold: left running on the noise it kept some out, and
      // after a gap the lines and the fields were right a little less of the time.)
      if (hold.coasting() && rhythm.knows(cur_field)) {
        vert.reset();
      } else if (const std::optional<FieldStart> s = vert.line(sync_now(), line_start, T, fs, levels.sign,
                                                               {levels.tip, levels.blank}, std_ == &kPal, line_idx)) {
        found_start(*s, StartBy::halves);
      }
    }
    if (hold.holding()) {
      pending_start.reset();  // (a run under the threshold says nothing in such noise)
    } else if (!moved) {
      if (pending_start) {
        found_start(*std::exchange(pending_start, std::nullopt), StartBy::run);
      } else if (vs && !prev_line_vsync && line_has_longrun) {
        // field start: phase of the broad-pulse start inside this line
        double ph = (static_cast<double>(line_longrun_start) - line_start) / T;
        ph -= std::floor(ph + 0.25);  // -> [-0.25, 0.75)
        pending_start = FieldStart{std::fabs(ph) < 0.25 ? 0 : 1, line_idx};
      }
    }
    // the flywheel: no vertical sync where the field in hand should end, and none waited on (with a weak signal a
    // vertical sync is known VertSync::kWait lines later)
    if (!pending_start) {
      if (const std::optional<FieldStart> due =
              rhythm.overdue({cur_field, field_line0}, line_idx - (hold.holding() ? VertSync::kWait : 0))) {
        fields_coasted++;
        start_field(*due);
      }
    }
    if (o.hold) next_start = held_line(next_start, vs || vert.broad());
    prev_line_vsync = vs;
    line_idx++;
    line_low = 0;
    line_has_longrun = false;
    line_start = next_start;
    refresh_switches();
  }

  // A field start has been found, by the broad pulses' run under the threshold (close_line) or by what the half
  // lines are (vert_sync.h): the field begins there, or with the flywheel where the fields' rhythm says it is due
  // (the start is put right), or the start is passed over (field_rhythm.h: found, seen). Without the flywheel a
  // field begins wherever either way finds one, once.
  enum class StartBy { run, halves };
  void found_start(FieldStart s, StartBy by) {
    const FieldStart in_hand{cur_field, field_line0};
    std::optional<FieldStart> at = s;
    if (o.flywheel) {
      at = by == StartBy::run ? rhythm.found(in_hand, s, std_->field_lines) : rhythm.seen(in_hand, s, std_->field_lines, signal());
    } else if (by == StartBy::halves && in_hand == s) {
      return;  // (the run has begun it already)
    }
    if (!at) return;
    if (*at != s) starts_put_right++;
    start_field(*at);
  }

  // How clean the sync is (DecoderStats::sync_snr): its depth over the noise on the back porch (sync_noise.h); 0
  // while that is not known. (settled: the levels are measured ones, on locked lines or in the fold, not the
  // histogram's)
  float sync_snr() const { return !acquiring && levels.settled() ? noise.snr(levels.blank - levels.tip) : 0; }

  // The line in hand has ended, and the next begins at `next` (DecoderOpts::hold): how noisy its sync is, and the
  // line into the fold, which steers when the signal is weak (line_hold.h). Returns where the next line begins.
  double held_line(double next, bool broad) {
    const SyncSamples s = sync_now();
    if (noise.line(s, line_start, fs)) {
      const float depth = levels.blank - levels.tip;
      if (noise.round()) {
        snr_now = noise.snr(depth);
        if (snr_now > 0) burst.sync_snr(snr_now);  // (colour goes by it)
      }
      // (lines >= 128: the first 64 lines after an acquisition count as found before they are)
      if (hold.clean() && lines >= 128 && depth > 0.2f) depth_ref += (depth_ref > 0 ? 1.f / 16 : 1.f) * (depth - depth_ref);
    }
    const LineHold::Verdict v = hold.line(s, {line_start, next, line_idx, line_in_field(), lines, broad, T, levels.sign,
                                              {levels.tip, levels.blank}, snr_now});
    T = v.T;
    if (v.levels) levels.set(v.levels->tip, v.levels->blank);
    if (v.rephased) {  // the signal had moved, and the grid is on it again
      rephases++;
      comb_epoch++;
      rhythm.doubt();
      vert.reset();
    }
    return v.next;
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
    // How far from where it was due a pulse may begin and still be the line's hsync: the line PLL's own window, or
    // less where the hold says so (LineHold::pulses).
    //  - Held by the fold, a pulse where the grid has the hsync counts as that line's, and steers nothing. One
    //    further off is a stray like any other.
    //  - Guarded: in a signal with noise in it, one where more than a few lines in 64 miss their hsync, a pulse far
    //    from the grid is no hsync while the fold shows the hsync where the grid is: at six tenths of the way
    //    towards each such pulse the grid was thrown about, and the line period with it. A signal that has really
    //    moved brings them in a row, a line apart: the third of such a run is followed as ever (the grid a few us
    //    off after a vertical interval's pulses pulled it: left to the sixth, a re-phase, the fields' rhythm went
    //    with it), and six further off re-phase. A clean signal that steps is followed at once, as ever.
    const LineHold::Pulses say = hold.pulses();
    const bool held = say == LineHold::Pulses::counted;
    const double window = held ? LineHold::kTightUs * 1e-6 * fs
                               : say == LineHold::Pulses::guarded && !in_a_run(edge) ? LineHold::kNearUs * 1e-6 * fs : 0.08 * T;
    if (std::fabs(at - pred) <= window) {
      double ns = pred;
      if (!held) {
        // hs_len follows the pulses a sixteenth of a sample at a time, whatever their length: one that something
        // ran into does not move it
        hs_len = hs_len > 0 ? hs_len + std::clamp(len - hs_len, -1.0, 1.0) * (1.0 / 16) : len;
        ns = 0.6 * at + 0.4 * pred;
        T = 0.995 * T + 0.005 * (ns - line_start);
      }
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
      hold.jumped();
      vert.reset();
    }
  }
};

// ---- front end: its two halves made, and decode()'s (demod()'s is in demod.cpp) --------------------------
FrontEnd::FrontEnd(double fs_in, double shift_hz, FrameSink sink, const DecoderOpts &o, const PictureOpts &p)
    : _fs_in(fs_in), _fs_out(fs_in >= 40e6 ? fs_in / 2 : fs_in), _decimate(fs_in >= 40e6), _picture(p) {
  for (int i = 0; i < kHb; i++) {
    const int k = i - kHb / 2;
    const double w = 0.5 - 0.5 * std::cos(2 * M_PI * (i + 0.5) / kHb);
    _h[i] = k == 0 ? 0.5f : static_cast<float>(std::sin(M_PI * k / 2) / (M_PI * k) * w);
  }
  _wr.assign(kHb - 1 + kTab, 0.f);
  _wi.assign(kHb - 1 + kTab, 0.f);
  _dec = new Decoder(_fs_out, std::move(sink), o, _picture);
  _clicks = new ClickDetector(_fs_out);
  // (the filter goes by the hold's measure of the sync's noise: without the hold there is none)
  if (o.channel_filter && o.hold) _chan = new ChannelFilter(_fs_out);
  set_shift(shift_hz);
}

FrontEnd::~FrontEnd() {
  delete _chan;
  delete _clicks;
  delete _dec;
}

DecoderStats FrontEnd::stats() const {
  DecoderStats s = _dec->stats();
  if (_chan) {
    s.filter_mhz = _chan->width();
    s.filter_changes = _chan->changes();
  }
  return s;
}

void FrontEnd::decode(const std::vector<float> &d, const std::vector<uint8_t> &bad, double t_end) {
  _dec->push_block(d.data(), bad.size() == d.size() ? bad.data() : nullptr, d.size(), t_end);
  if (_chan) _chan->steer(_dec->sync(), static_cast<double>(d.size()) / _fs_out);
}

void FrontEnd::feed(const cf *xs, size_t n) {
  demod(xs, n, _dbuf, _badbuf);
  decode(_dbuf, _badbuf);
}

}  // namespace kp
