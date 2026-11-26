// fpvDROID video decoder. See video.h. The algorithm follows vtx-spike/viewer/vtxview.cpp (measured on a
// real R4 NTSC VTX: 93-96 % of lines locked); changes are marked "fpvDROID:".
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
#include "picture_filters.h"
#include "rings.h"
#include "sparkle.h"

namespace kp {

const VideoStd kNtsc{63.5556, 9.4, 52.6, 240, 18, "NTSC"};
const VideoStd kPal{64.0, 10.5, 52.0, 288, 22, "PAL"};

// ---- the decoder: discriminator output (MHz) in, frames out ---------------------------------------------
// It finds the sync pulses (levels, acquisition, a line PLL that freewheels and re-phases), which is all its own;
// for the rest it runs the modules: the luma and colour paths behind the pulse tracking into the sample rings
// (picture_filters.h, rings.h), and as each line ends, its burst, sparkle repair (sparkle.h), the line into a row
// (line_draw.h) and, as each field ends, the frame (deinterlace.h); with strips (PictureOpts::strips), the frame a
// strip at a time as the rows come in.
class Decoder {
public:
  // picture: the switches, which whoever owns the decoder may change at any time (FrontEnd::set_picture)
  Decoder(double fs, FrameSink sink, const DecoderOpts &o, const std::atomic<PictureOpts> &picture)
      : fs(fs), sink(std::move(sink)), o(o), picture(picture), drawer(o.width, fs), filt(fs, o.colour), spark(fs) {
    lp_sync.design(1.0e6, fs);
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
    s.level_jumps = level_jumps;
    s.fields_coasted = fields_coasted;
    s.starts_put_right = starts_put_right;
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
  float sign = +1, tip = -1, blank = 0, thr = -0.5f;
  uint64_t n = 0;  // samples taken (the pulse tracking's count)
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
  uint64_t lines = 0, lines_locked = 0, line_idx = 0, acquisitions = 0, losses = 0;
  uint64_t line_low = 0;
  bool line_has_longrun = false;
  uint64_t line_longrun_start = 0;
  bool prev_line_vsync = false;
  bool start_due = false, start_odd = false;  // a field start that waits for the lines after it (close_line) ...
  int64_t start_line = 0;                     // ... and its first line
  // fpvDROID: the vertical flywheel. Lines freewheel when their hsync is not found; fields did not: without its
  // vertical sync a field was not begun, and its lines, found or not, went undrawn (a weak signal that kept 86 %
  // of its hsyncs showed a third of its pictures). The fields come at a fixed rhythm, though: so many lines from
  // an odd field's start to the next field's, so many from an even one's, and which kind follows. The standard's
  // interlace is taken as read from the first start found; a source with another rhythm shows it, and that is
  // then learnt (seen twice running). With a rhythm, a field whose vertical sync has not shown kNear lines after
  // it was due is begun where it was due, and a start found up to kNear lines late or as the other kind (a
  // vertical sync damaged at its start) is put there. A start found early or far off is put right or passed over
  // too, but kOff of those in a row, with none where the rhythm said, show that the source has another rhythm. A
  // jump in the stream (re-phase) loses the count of lines: the next start found is taken as it is.
  static constexpr int kNear = 2, kOff = 3;
  int64_t fw_len[2] = {0, 0};  // lines from a field's start to the next field's, by its kind (0 odd, 1 even) ...
  int fw_next[2] = {0, 0};     // ... and the kind of that one
  int fw_seen[2] = {0, 0};     // times running that this has been seen (2: believed)
  int fw_off = 0;              // starts found in a row that were not where the rhythm said
  bool fw_coasted = false;     // the field in hand was begun by the flywheel
  bool fw_plain = false;       // the source has shown a rhythm that is not the standard's
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
  std::vector<uint32_t> hist;
  double hlo = -8, hhi = 8;
  int hdec = 0;
  uint64_t hist_count = 0;
  // fpvDROID: sync-gated levels: sync tip from the middle of each hsync pulse, blanking from the back porch,
  // measured on locked active lines and smoothed over ~64 lines. Picture content cannot move them (the
  // spike's histogram peaks could, and the threshold wandered with the scene).
  uint64_t gated_lines = 0;
  // fpvDROID: ... and after a jump. A transmitter whose video input is AC-coupled moves the whole signal, sync
  // and picture alike, when the picture's brightness changes: a camera pointed at a light flashing at 7 Hz moved
  // it by 1.3 times the sync depth with every flash. The threshold is then outside the pulses, no line finds its
  // hsync, and the gated levels, measured on lines that did, never hear of it: the lock went with every flash.
  // So every line is looked at where its hsync is due, found or not (the grid freewheels on: it is the levels
  // that moved, not the timing), and when kJumpLines of them in a row show a pulse of the same depth as before at
  // the same new levels, the levels go there at once (levels_moved). That transmitter's signal took some
  // milliseconds over each move: so for kFastLines lines after a jump the gated levels follow a quarter of the way
  // a line, not a 64th, and the rest of the slide is followed line by line (else in steps, each a jump of its
  // own, and the picture's brightness with them).
  static constexpr int kJumpLines = 3, kFastLines = 128;
  int fast_lines = 0;                  // lines the gated levels still follow fast for
  int jump_run = 0, jump_gap = 0;      // lines that have said so, and lines since that could not say
  float jump_tip = 0, jump_blank = 0;  // the levels they said (their mean)
  bool line_moved = false;             // the line in hand is one of them ...
  float line_tip = 0, line_blank = 0;  // ... with its sync at these levels: it is drawn against them
  uint64_t level_jumps = 0;
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
      const double ls = line_start;
      miss_run++;
      count_line(false);
      line_moved = o.follow_levels && levels_moved(ls);
      render_line(ls, false);
      close_line(ls + T);
      check_lost();
    }
  }

  // How many of the next samples (sync-path values m[0..cnt)) track() would do nothing with: locked on (not
  // acquiring, the levels settled), between pulses, still above the threshold, and the line not yet overdue.
  // They are passed over here, n moved on past them; the first that would do something is left to track().
  // ~90 % of the samples, looked at eight at a time (a vector compare), where track() takes each on its own.
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
    filt.run(R, piece_d + k0, bd, cnt);
  }

  void set_std(const VideoStd &s) {
    std_ = &s;
    forget_fields();
    fw_plain = false;
    T = s.T_us * 1e-6 * fs;
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
    inv_depth = 1.f / std::max(blank - tip, 0.2f);
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
  }

  // fpvDROID: lost when fewer than 40 % of the last ~2 fields' lines found their hsync (a healthy picture
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
    field_strips = false;
    haveA = haveB = false;
    recent_n = recent_locked = recent_pos = 0;
    line_start = -1;
    gated_lines = 0;
    jump_run = jump_gap = fast_lines = 0;
    line_moved = false;
    start_due = false;
    forget_fields();
    fw_plain = false;
    miss_run = stray_run = 0;
    last_stray = -1;
    hs_len = 0;
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

  bool in_picture(int64_t k) const { return k >= 0 && k < std_->active; }

  // The line that has just ended (ls: its start; k: its row; locked: its hsync was found), ready to be drawn:
  // levels measured on it, then its picture put right in the rings (the top line with the luma comb, sparkle
  // repair), then its burst. In that order: each reads what the one before left.
  LineInfo measure(double ls, int64_t k, bool locked) {
    LineInfo l;
    l.ls = ls;
    l.k = k;
    l.field = cur_field;
    l.epoch = comb_epoch;
    l.moved = line_moved;
    l.tip = line_tip;
    l.blank = line_blank;
    if (!in_picture(k)) return l;
    if (locked && !line_moved) measure_levels(ls);
    if (filt.luma_comb()) top_line(ls, k);
    l.hit = pic.repair && patch_line(ls);
    l.col = o.colour && locked && burst(ls, l.pal_v);
    l.ok = l.col && burst_e < kCombMaxErr;
    l.phase = c_phase;
    l.mag = c_mag;
    l.fade = filt.fade_noted();
    return l;
  }

  // A line has ended: it is drawn, or with the 3-line comb the one before it is.
  void render_line(double ls, bool locked) {
    if (cur_field < 0) return;
    const LineInfo cur = measure(ls, static_cast<int64_t>(line_idx) - field_line0 - std_->skip, locked);
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
    const float t = l.moved ? l.tip : tip, b = l.moved ? l.blank : blank;
    drawer.draw(row, R, l, up, dn, DrawParams{sign, t, b, T, filt.cdel()});
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
      return ring_mean(l + a0 + aw * x0, l + a0 + aw * x1, m) && (m - blank) * inv_depth < limit;
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

  // Does the line that began at ls (its hsync found there, or only due there) have its sync at other levels than
  // the ones kept? It does when there is an hsync at ls by its shape (a pulse flat at its bottom, between two
  // porches at one level, as deep as the sync was) and both its levels are off the kept ones the same way by more
  // than a quarter of that depth. (A click moves one of the two, noise neither that far; where the grid is off
  // the signal there is no such pulse, and a broad pulse has no porch behind it: one PAL transmitter's dip 2 us
  // into its broad pulses, as deep again as the sync, would else pass for an hsync one depth down.) The
  // kJumpLines-th such line in a row at the same new levels moves the kept levels onto them. A line with an hsync
  // where the levels were ends the run at once, lines that cannot say (the vertical interval's) after a few.
  bool levels_moved(double ls) {
    const double us = 1e-6 * fs;
    const float depth = blank - tip;
    float t0, t1, b0, b1, front;
    if (depth < 0.2f || !ring_mean(ls - 1.1 * us, ls - 0.4 * us, front) ||
        !ring_mean(ls + 1.2 * us, ls + 2.4 * us, t0) || !ring_mean(ls + 2.4 * us, ls + 3.6 * us, t1) ||
        !ring_mean(ls + 5.8 * us, ls + 7.3 * us, b0) || !ring_mean(ls + 7.3 * us, ls + 8.8 * us, b1)) {
      jump_run = 0;
      return false;
    }
    const float t = 0.5f * (t0 + t1), b = 0.5f * (b0 + b1);
    const bool pulse = std::fabs(t1 - t0) < 0.2f * depth && std::fabs(b1 - b0) < 0.2f * depth &&
                       std::fabs(front - b) < 0.3f * depth && std::fabs(b - t - depth) < 0.3f * depth;
    if (!pulse || std::fabs(t - tip) <= 0.25f * depth || std::fabs(b - blank) <= 0.25f * depth) {
      if (pulse || ++jump_gap > 4) jump_run = 0;
      return false;
    }
    jump_gap = 0;
    line_tip = t;
    line_blank = b;
    if (jump_run > 0 && std::fabs(t - jump_tip) < 0.25f * depth && std::fabs(b - jump_blank) < 0.25f * depth) {
      jump_run++;
      jump_tip += (t - jump_tip) / static_cast<float>(jump_run);
      jump_blank += (b - jump_blank) / static_cast<float>(jump_run);
    } else {
      jump_run = 1;
      jump_tip = t;
      jump_blank = b;
    }
    if (jump_run >= kJumpLines) {
      KP_LOGD("video: the levels moved by %+.2f MHz, to tip %.2f blank %.2f, at line %lld of the field",
              jump_tip - tip, jump_tip, jump_blank,
              cur_field < 0 ? -1LL : static_cast<long long>(line_idx) - static_cast<long long>(field_line0));
      tip = jump_tip;
      blank = jump_blank;
      thr = 0.5f * (tip + blank);
      jump_run = 0;
      fast_lines = kFastLines;
      level_jumps++;
      start_due = false;  // the field start that waits (close_line) was the signal dropping
    }
    return true;
  }

  void measure_levels(double ls) {
    const double us = 1e-6 * fs;
    float t, b;
    if (!ring_mean(ls + 1.2 * us, ls + 3.6 * us, t)) return;  // hsync is 4.7 us
    if (!ring_mean(ls + 5.8 * us, ls + 8.8 * us, b)) return;  // back porch (burst averages out)
    if (b - t < 0.05f) return;
    const float a = gated_lines < 64 ? 0.1f : fast_lines > 0 ? 0.25f : 1.f / 64;
    if (fast_lines > 0) fast_lines--;
    tip += a * (t - tip);
    blank += a * (b - blank);
    thr = 0.5f * (tip + blank);
    gated_lines++;
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

  void close_line(double next_start) {
    // A line whose levels have moved was low against a threshold that was no longer right: it starts no field.
    // Nor does the line before the first of them, the one the signal dropped in, low from there to its end (the
    // picture's black under the old threshold). So a field start waits for the line after it, in a vertical sync
    // another of its lines, and while lines say the levels have moved: when they have (levels_moved), it is
    // dropped.
    const bool moved = line_moved, vs = line_low > 0.5 * T;
    line_moved = false;
    if (start_due && !moved) {
      start_due = false;
      found_start(start_odd, start_line);
    } else if (vs && !moved && !start_due && !prev_line_vsync && line_has_longrun) {
      // field start: phase of the broad-pulse start inside this line
      double ph = (static_cast<double>(line_longrun_start) - line_start) / T;
      ph -= std::floor(ph + 0.25);  // -> [-0.25, 0.75)
      start_due = true;
      start_odd = std::fabs(ph) < 0.25;
      start_line = static_cast<int64_t>(line_idx);
    }
    // the flywheel: no vertical sync where the field in hand should end, and none waited on
    if (!start_due && cur_field >= 0 && believed(cur_field) &&
        static_cast<int64_t>(line_idx) >= field_line0 + fw_len[cur_field] + kNear) {
      const int kind = cur_field;
      fields_coasted++;
      start_field(fw_next[kind] == 0, field_line0 + fw_len[kind], true);
    }
    prev_line_vsync = vs;
    line_idx++;
    line_low = 0;
    line_has_longrun = false;
    line_start = next_start;
    refresh_switches();
  }

  bool believed(int kind) const { return o.flywheel && fw_seen[kind] >= 2; }
  void forget_fields() {
    fw_seen[0] = fw_seen[1] = 0;
    fw_off = 0;
  }

  // A field start has been found, its first line s: the field begins there, or where the flywheel says, or it is
  // passed over (see fw_len).
  void found_start(bool odd, int64_t s) {
    const int kind = cur_field, next = odd ? 0 : 1;
    if (!o.flywheel) return start_field(odd, s);
    // the standard's rhythm: an odd field of so many lines, then an even one of one more
    const int64_t half = std_ == &kPal ? 312 : 262;
    if (kind < 0 || !believed(kind)) {
      // (no field is begun by the flywheel without a rhythm: the field in hand is the last one found)
      const int64_t len = kind < 0 ? 0 : s - field_line0;
      const int64_t n = std::llround(static_cast<double>(len) / (static_cast<double>(half) + 0.5));  // fields
      bool standard = !fw_plain;
      if (!standard && kind >= 0) {  // does the standard's rhythm lead from the last start found to this one?
        int64_t at = 0;
        int k = kind;
        for (int i = 0; i < 16 && at < len; i++) {
          at += half + k;
          k = 1 - k;
        }
        standard = at == len && k == next;
      }
      if (standard) {
        // the standard's rhythm: taken as read from the first start found, until the source shows another
        fw_plain = false;
        for (int k = 0; k < 2; k++) {
          fw_len[k] = half + k;
          fw_next[k] = 1 - k;
          fw_seen[k] = 2;
        }
      } else if (kind >= 0 && n >= 1 && n <= 16 && (n == 1 || next == kind) && len % n == 0 &&
                 std::llabs(len / n - half) <= 3) {
        // another, seen twice running: the next field's start found (about a field on), or a later one of the same
        // kind a whole number of fields on, which says how long each is where the fields are all alike
        if (fw_seen[kind] > 0 && fw_len[kind] == len / n && fw_next[kind] == next) {
          fw_seen[kind] = 2;
        } else {
          fw_len[kind] = len / n;
          fw_next[kind] = next;
          fw_seen[kind] = 1;
        }
      }
      return start_field(odd, s);
    }
    const int64_t due = field_line0 + fw_len[kind];
    const bool due_odd = fw_next[kind] == 0;
    if (s == due && odd == due_odd) {
      fw_off = 0;
      start_field(odd, s);
    } else if (s >= due && s <= due + kNear) {
      // late by a line or two, or as the other kind: what a vertical sync damaged at its start looks like
      starts_put_right++;
      start_field(due_odd, due);
    } else if (++fw_off >= kOff) {
      // early, or far off, kOff times running: the source's rhythm is another
      KP_LOGD("video: the fields' rhythm is forgotten (a start %lld lines from where it was due)",
              static_cast<long long>(s - due));
      forget_fields();
      fw_plain = true;
      start_field(odd, s);
    } else if (s >= due - kNear && s < due) {
      starts_put_right++;
      start_field(due_odd, due);
    }
  }

  // A field begins, at line line0 (the first of its vertical sync): the one before it is done.
  void start_field(bool odd, int64_t line0, bool coasted = false) {
    fw_coasted = coasted;
    // smooth lines: A is the lower field, B the upper one (B's line k is drawn above A's line k)
    if (cur_field == 0) haveA = true;
    if (cur_field == 1) haveB = true;
    if (o.fields && cur_field >= 0) field_done();
    if (!o.fields && haveA && haveB) {
      emit(fieldA, fieldB);
      haveA = haveB = false;
    }
    cur_field = odd ? 0 : 1;
    field_line0 = line0;
    fields_seen++;
    if (odd) haveB = false;  // a frame is A then B
    begin_field();
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
    if (std::fabs(at - pred) <= 0.08 * T) {
      // hs_len follows the pulses a sixteenth of a sample at a time, whatever their length: one that something
      // ran into does not move it
      hs_len = hs_len > 0 ? hs_len + std::clamp(len - hs_len, -1.0, 1.0) * (1.0 / 16) : len;
      const double ns = 0.6 * at + 0.4 * pred;
      T = 0.995 * T + 0.005 * (ns - line_start);
      miss_run = 0;
      stray_run = 0;
      count_line(true);
      line_moved = o.follow_levels && levels_moved(line_start);
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
      forget_fields();
      // In the log every time, with how far the grid moved (the shorter way round), because that says why. A
      // block of samples lost on the way here (the decode queue drops 4096 at a time, the stream a bank, which
      // is as many) moves it by that many samples less whole lines: -14.1 us for each at 20 MS/s NTSC, -12.8 us
      // PAL. A stream restart moves it by anything. A small move the late way (+) with nothing lost is hsyncs
      // that came later than their line.
      KP_LOGI("video: re-phased the line grid by %+.2f us at line %lld of the field, after %d lines without an hsync",
              std::remainder(edge - line_start, T) / fs * 1e6,
              cur_field < 0 ? -1LL : static_cast<long long>(line_idx) - static_cast<long long>(field_line0), miss_run);
      line_start = edge;
      line_low = 0;
      line_has_longrun = false;
      stray_run = 0;
      miss_run = 0;
    }
  }
};

// ---- front end ----------------------------------------------------------------------------------------
// fpvDROID: block-wise and branch-free so the compiler vectorises it (the spike's per-sample loop used a whole
// phone core at 40 MS/s). Per chunk of <= kTab samples: DC removal (block mean, smoothed), mix by a
// precomputed rotation table, halfband /2 over a work array with 22 samples of history, discriminator.
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

void FrontEnd::demod(const cf *xs, size_t n, std::vector<float> &out, std::vector<uint8_t> &bad) {
  out.clear();
  const bool rep = _picture.load(std::memory_order_relaxed).repair;
  _clicks->begin(rep);
  bad.clear();
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
  if (rep) _clicks->mark(out, bad);
}

void FrontEnd::decode(const std::vector<float> &d, const std::vector<uint8_t> &bad, double t_end) {
  _dec->push_block(d.data(), bad.size() == d.size() ? bad.data() : nullptr, d.size(), t_end);
}

void FrontEnd::feed(const cf *xs, size_t n) {
  demod(xs, n, _dbuf, _badbuf);
  decode(_dbuf, _badbuf);
}

}  // namespace kp
