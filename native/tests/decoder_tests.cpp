// kp_decoder_tests: the decoder's sync tracking on a signal made here (ctest: "decoder"): the lock, strips, the
// picture window, level jumps and the vertical flywheel. A weak signal has a program of its own
// (weak_signal_tests.cpp); what the two share is in decoder_tests.h.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <optional>
#include <vector>

#include "decoder_tests.h"
#include "field_rhythm.h"
#include "log.h"
#include "rings.h"
#include "sparkle.h"
#include "sync_levels.h"
#include "video.h"

// ---------------------------------------------------------------------------------------- the checks

static void locks_on_a_clean_signal() {
  const DecoderStats &st = clean();
  CHECK(!st.acquiring && st.acquisitions == 1 && st.losses == 0);
  CHECK(st.rephases == 0);
  CHECK(st.lock_recent > 0.95f);
  CHECK(std::fabs(st.line_us - 63.5556) < 0.01);
  CHECK(std::fabs(st.tip - kTip) < 0.1f && std::fabs(st.blank - kBlank) < 0.1f);
}

// A dip that runs into the hsync from the front (the black edge of an OSD character at the end of a line, a
// click): the pulse begins 0.85 us early and is that much longer. The grid stays where it is: no line is lost,
// and nothing re-phases.
static void a_dip_ahead_of_the_hsync() {
  const DecoderStats st = decode(video(0.4, on_the_grid, [](Signal &s, int64_t line, double start) {
    if (line % 97 == 40) s.put(start - 0.85 * kUs, start, kTip - 1.f);
  }));
  CHECK(st.rephases == 0);
  CHECK(st.acquisitions == 1 && st.losses == 0);
  CHECK(st.lines_locked == clean().lines_locked);
}

// One that splits the hsync: 1.3 us of it, a gap, and the rest, which alone is long enough to be an hsync and
// begins 1.6 us late.
static void a_gap_in_the_hsync() {
  const DecoderStats st = decode(video(0.4, on_the_grid, [](Signal &s, int64_t line, double start) {
    if (line % 97 == 40) s.put(start + 1.3 * kUs, start + 1.6 * kUs, kBlank + 1.f);
  }));
  CHECK(st.rephases == 0);
  CHECK(st.lines_locked == clean().lines_locked);
}

// The transmitter's line timing steps: every line from one on comes later. Inside the window (5.1 us either
// side of where the hsync is due) the line PLL follows without losing a line; past it the grid re-phases, once.
static void a_step_in_the_line_timing() {
  for (const double us : {0.5, 1.5, 4.0, -0.5, -4.0}) {
    const DecoderStats st = decode(video(0.4, [us](int64_t line) { return line >= 3000 ? us * kUs : 0.0; }, nullptr));
    CHECK(st.rephases == 0);
    CHECK(st.lines_locked == clean().lines_locked);
    if (st.rephases || st.lines_locked != clean().lines_locked)
      std::fprintf(stderr, "  step %+.1f us: %llu re-phases, %llu lines locked (clean: %llu)\n", us,
                   (unsigned long long)st.rephases, (unsigned long long)st.lines_locked,
                   (unsigned long long)clean().lines_locked);
  }
  const DecoderStats st = decode(video(0.4, [](int64_t line) { return line >= 3000 ? 12.0 * kUs : 0.0; }, nullptr));
  CHECK(st.rephases == 1);
  CHECK(st.acquisitions == 1 && st.losses == 0);
}

// ---------------------------------------------------------------------------------------- strips

// With strips on, the pictures are the same as without them, lines and times, and every field goes out in strips
// that add up to it: whatever smooth lines, sparkle repair and the 3-line comb (which draws each line a line late)
// do.
static void strips_add_up_to_the_frames() {
  const Signal s = moving_stripes(0.3);
  for (int combo = 0; combo < 8; combo++) {
    PictureOpts p;
    p.smooth_lines = combo & 1;
    p.repair = combo & 2;
    p.chroma_comb = combo & 4;
    const std::vector<Picture> plain = frames_of(s, p);
    p.strips = true;
    int per_frame = 0;
    const std::vector<Picture> in_strips = frames_of(s, p, &per_frame);
    CHECK(plain.size() >= 12 && plain.size() == in_strips.size());
    bool same = plain.size() == in_strips.size(), same_times = same;
    for (size_t i = 0; same && i < plain.size(); i++) {
      same = plain[i].px == in_strips[i].px;
      const LineTimes &a = plain[i].times, &b = in_strips[i].times;
      same_times &= a.t_first == b.t_first && a.t_last == b.t_last;
    }
    CHECK(same);
    CHECK(same_times);
    // NTSC: 242 field lines, 32 to a strip (a strip may wait 3 lines for sparkle repair, one for smooth lines)
    CHECK(per_frame >= 7 && per_frame <= 9);
    if (!same || per_frame < 7 || per_frame > 9)
      std::fprintf(stderr, "  smooth %d repair %d comb %d: %zu / %zu frames, %d strips a frame\n", p.smooth_lines,
                   p.repair, p.chroma_comb, plain.size(), in_strips.size(), per_frame);
  }
}

// A frame's times: its first line was taken about a field (262.5 lines) before its last, and after the frame
// before's last. A decoder that is not told when its samples were taken says nothing.
static void frames_say_when_their_lines_were_taken() {
  const Signal s = moving_stripes(0.3);
  const std::vector<Picture> f = frames_of(s, PictureOpts{});
  CHECK(f.size() >= 8);
  for (size_t i = 2; i < f.size(); i++) {
    const double lines = (f[i].times.t_last - f[i].times.t_first) * kFs / kT;
    CHECK(lines > 235 && lines < 245);  // first row to last row: 241 lines apart
    if (i > 2) CHECK(f[i].times.t_first > f[i - 1].times.t_last);
  }
  int frames = 0, timed = 0;
  FrontEnd fe(kFs, 0, [&](const Frame &fr) {
    frames++;
    timed += fr.times.t_first != 0 || fr.times.t_last != 0;
  });
  fe.decode(s.d, {});
  CHECK(frames >= 8 && timed == 0);
}

// ---------------------------------------------------------------------------------------- the picture window

// A camera sends more than the standard's active picture. On a lit NTSC recording its picture ran 8.85-62.3 us after
// the hsync, on lines 19-258 or 20-259 (by field) after the line of the field's first broad pulse (video_std.h).
// All of that is in the frame, with blanking either side: each field's 240 lines, and each line its whole width.
static void the_window_keeps_the_whole_picture() {
  constexpr float kWhite = 4.f;  // 0.8 of black to white
  constexpr int kLit = 140, kDark = 40;
  const Signal s = video(0.3, on_the_grid, [](Signal &s, int64_t line, double start) {
    s.put(start + 9.4 * kUs, start + 62.0 * kUs, kBlank);
    // the line's number after the line of its field's first broad pulse (the field's half line 6)
    const int64_t field = 2 * line / kHalfLinesPerField, broad = (field * kHalfLinesPerField + 6) / 2;
    const int64_t top = field % 2 ? 20 : 19, n = line - broad;
    if (n >= top && n < top + 240) s.put(start + 8.85 * kUs, start + 62.3 * kUs, kWhite);
  });
  const std::vector<Picture> f = frames_of(s, PictureOpts{});
  CHECK(f.size() >= 12);
  // a field's row (each is two lines of the frame) at x: its red
  const auto px = [](const Picture &p, int row, int x) { return p.px[(static_cast<size_t>(2 * row) * 720 + x) * 4]; };
  int tops[2] = {}, left = 720, right = -1;
  bool ok = f.size() >= 12;
  for (size_t i = 4; i < f.size(); i++) {
    int first = -1, last = -1, lit = 0;
    for (int r = 0; r < kNtsc.window.lines; r++) {
      if (px(f[i], r, 360) <= kLit) continue;
      lit++;
      if (first < 0) first = r;
      last = r;
      // across: one lit run, with dark columns at both ends
      int x0 = 0, x1 = 719;
      while (x0 < 720 && px(f[i], r, x0) <= kLit) x0++;
      while (x1 > x0 && px(f[i], r, x1) <= kLit) x1--;
      for (int x = x0; x <= x1; x++) ok &= px(f[i], r, x) > kLit;
      ok &= px(f[i], r, 0) < kDark && px(f[i], r, 719) < kDark;
      left = std::min(left, x0);
      right = std::max(right, x1);
    }
    ok &= lit == 240 && last - first == 239 && px(f[i], 0, 360) < kDark;
    if (first == 1 || first == 2) tops[first - 1]++;
    else ok = false;
  }
  ok &= tops[0] > 0 && tops[1] > 0;  // both fields: the one with its picture a line lower reaches the last row
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  %zu frames: picture from row 1 in %d, from row 2 in %d; lit columns %d..%d\n", f.size(), tops[0],
                 tops[1], left, right);
}

// Sparkle repair keeps off the next line's sync. The window ends close to the next hsync, and a patch for a click
// at its very end would ramp onto that hsync's edge, where the next line's front porch is read. Every click that
// begins in the picture is still patched, but no luma from `end` on is touched.
static void a_click_at_the_end_of_the_picture() {
  const double ls = 1000, pic0 = ls + kNtsc.window.start_us * kUs, aw = kNtsc.window.width_us * kUs;
  const double end = ls + kT - SyncLevels::kFrontUs[1] * kUs;
  const int64_t hsync = static_cast<int64_t>(std::ceil(ls + kT)), e = static_cast<int64_t>(end), n = 4000;
  SparkleRepair spark(kFs);
  int patched = 0, left = 0;
  bool clear = true, all_in_picture = true;
  for (int64_t at = e - 40; at < e; at++) {
    SampleRings R;
    // a front porch with a ripple on it (a patch's straight line would show), then the hsync
    for (int64_t i = 0; i < n; i++)
      R.luma[i & R.mask] = i >= hsync && i < hsync + 94 ? kTip : kBlank + 0.05f * std::sin(0.7f * static_cast<float>(i));
    for (int64_t i = at; i < at + 8; i++) {  // a click: a run of 8 marked samples
      R.luma[i & R.mask] = kGrey + 3;
      R.bad[i & R.mask] = 1;
    }
    R.n = n;
    const std::vector<float> was = R.luma;
    spark.patch_line(R, pic0, aw, end, kT, false, false);
    for (int64_t i = e; i < n; i++) clear &= R.luma[i & R.mask] == was[i & R.mask];
    const bool done = R.luma[at & R.mask] != was[at & R.mask];
    (done ? patched : left)++;
    if (at < pic0 + aw) all_in_picture &= done;
  }
  const bool ok = clear && all_in_picture && patched > 0;
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  clicks in the last 2 us before end: %d patched, %d left (those in the picture: %s); luma from "
                 "end on %s\n", patched, left, all_in_picture ? "all patched" : "NOT ALL", clear ? "untouched" : "CHANGED");
}

// ---------------------------------------------------------------------------------------- level jumps

// The whole signal, sync and picture alike, moved by by(seconds) MHz. A transmitter whose video input is AC-coupled
// does that when the picture's brightness changes: seen from a camera pointed at a light flashing at 7 Hz, which
// moved the sync by 1.3 times its depth with every flash (a Pixel 8 Pro's log, 2026-11-25).
static Signal moved(Signal s, const std::function<double(double)> &by) {
  for (size_t i = 0; i < s.d.size(); i++) s.d[i] += static_cast<float>(by(static_cast<double>(i) / kFs));
  return s;
}

// by() for a light flashing at 7 Hz: the signal down by mhz while it is lit (the second half of each flash), and
// each move taking tau seconds (an RC coupling's exponential; 0: at once).
static std::function<double(double)> flashing(double mhz, double tau) {
  return [=](double t) {
    const double ph = t * 7 - std::floor(t * 7);  // 0..1 through a flash
    if (t * 7 < 0.5) return 0.0;                  // not lit before the first flash
    const bool lit = ph >= 0.5;
    const double since = (lit ? ph - 0.5 : ph) / 7, to = lit ? -mhz : 0, from = lit ? 0 : -mhz;
    return tau > 0 ? to + (from - to) * std::exp(-since / tau) : to;
  };
}

static constexpr int kJumpsASecond = 13;  // at 7 Hz, the first after half a flash

// The lock holds through them, whichever way the signal moves, by less or more than the sync depth, at once or
// over a few milliseconds: the line grid stays, no field is started that is none, and all but a few lines at each
// jump find their hsync. (A jump that lands on the lines of a vertical sync hides it: they have no hsync to read
// the new levels from, and are over before the lines after them have said. The flywheel begins that field.)
static void the_levels_jump_with_a_flashing_light() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const DecoderStats steady = decode(plain);
  CHECK(steady.acquisitions == 1 && steady.losses == 0);
  for (const double depths : {0.6, 1.3, 2.0, -0.6, -1.3, -2.0})
    for (const double tau : {0.0, 0.5e-3, 2e-3, 5e-3}) {
      const DecoderStats st = decode(moved(plain, flashing(depths * (kBlank - kTip), tau)));
      const long long missed = static_cast<long long>(steady.lines_locked) - static_cast<long long>(st.lines_locked);
      const bool ok = st.acquisitions == 1 && st.losses == 0 && st.rephases == 0 && st.fields == steady.fields &&
                      missed <= kJumpsASecond * 8 &&
                      (tau > 0 || st.level_jumps == kJumpsASecond);  // (a slow move is partly followed without)
      CHECK(ok);
      if (shown(ok))
        std::fprintf(stderr, "  %+.1f depths over %.1f ms: %llu acquisitions, %llu losses, %llu re-phases, %llu fields "
                     "(steady: %llu), %lld lines without their hsync, %llu level jumps\n", depths, tau * 1e3,
                     (unsigned long long)st.acquisitions, (unsigned long long)st.losses,
                     (unsigned long long)st.rephases, (unsigned long long)st.fields,
                     (unsigned long long)steady.fields, missed, (unsigned long long)st.level_jumps);
    }
}

// And the picture keeps its brightness: its grey is drawn against the levels as they are after the jump, the lines
// that showed the jump against their own.
static void the_picture_keeps_its_grey_through_a_jump() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const auto mid = [](const Picture &p, int row) { return p.px.data() + (static_cast<size_t>(row) * 720 + 360) * 4; };
  const std::vector<Picture> steady = frames_of(plain, PictureOpts{});
  CHECK(steady.size() >= 50);
  if (steady.size() < 50) return;
  const uint8_t grey[3] = {mid(steady[20], 240)[0], mid(steady[20], 240)[1], mid(steady[20], 240)[2]};
  const auto off_rows = [&](const std::vector<Picture> &f) {
    int off = 0;
    for (size_t i = 4; i < f.size(); i++)
      for (int row = 16; row < 464; row++)
        for (int c = 0; c < 3; c++)
          if (std::abs(mid(f[i], row)[c] - grey[c]) > 6) { off++; break; }
    return off;
  };
  CHECK(off_rows(steady) == 0);
  for (const double depths : {1.3, -1.3}) {
    const std::vector<Picture> f = frames_of(moved(plain, flashing(depths * (kBlank - kTip), 0)), PictureOpts{});
    const int off = off_rows(f);
    // the line the jump is in is off from there on; a field's line is two of the frame's, and smooth lines spread
    // it over one more either side
    const bool ok = f.size() == steady.size() && off <= kJumpsASecond * 4;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %+.1f depths: %zu frames (steady: %zu), %d rows off their grey\n", depths, f.size(),
                          steady.size(), off);
  }
}

// ---------------------------------------------------------------------------------------- the vertical flywheel

// The rhythm by itself (field_rhythm.h), on line numbers alone: where a field begins for the starts found, or for
// none found. NTSC: an odd field (kind 0) of 262 lines, then an even one (kind 1) of 263.
static void the_rhythm_on_line_numbers() {
  using Start = std::optional<FieldStart>;
  const int64_t lines = 262;
  const FieldStart none{-1, 0}, odd{0, 100}, even{1, 362}, next_odd{0, 625};
  {
    FieldRhythm r;
    CHECK(r.overdue(none, 5000) == std::nullopt);  // no rhythm before the first start
    // the standard's rhythm is taken as read from the first start found
    CHECK(r.found(none, odd, lines) == Start(odd));
    CHECK(r.overdue(odd, even.line + 1) == std::nullopt);  // a start may yet be found a line late
    CHECK(r.overdue(odd, even.line + 2) == Start(even));   // none two lines on: the field begins where it was due
    CHECK(r.found(odd, even, lines) == Start(even));
    // a start a line or two off, or on its line as the other kind, is put where it was due
    CHECK(r.found(even, FieldStart{0, 626}, lines) == Start(next_odd));
    CHECK(r.found(even, FieldStart{0, 627}, lines) == Start(next_odd));
    CHECK(r.found(even, FieldStart{1, 625}, lines) == Start(next_odd));
    CHECK(r.found(even, FieldStart{0, 624}, lines) == Start(next_odd));
    CHECK(r.found(even, next_odd, lines) == Start(next_odd));
    // far off: passed over. The third in a row is where the field begins, and the standard's rhythm is forgotten
    CHECK(r.found(even, FieldStart{0, 500}, lines) == std::nullopt);
    CHECK(r.found(even, FieldStart{0, 501}, lines) == std::nullopt);
    CHECK(r.found(even, FieldStart{0, 502}, lines) == Start(FieldStart{0, 502}));
    CHECK(r.overdue(FieldStart{0, 502}, 5000) == std::nullopt);
    // the source's own is learnt once it is seen twice running: here fields that are all alike, 262 lines each,
    // the first of them found three fields on
    CHECK(r.found(FieldStart{0, 502}, FieldStart{0, 1288}, lines) == Start(FieldStart{0, 1288}));
    CHECK(r.overdue(FieldStart{0, 1288}, 5000) == std::nullopt);
    CHECK(r.found(FieldStart{0, 1288}, FieldStart{0, 1550}, lines) == Start(FieldStart{0, 1550}));
    CHECK(r.overdue(FieldStart{0, 1550}, 1812 + 2) == Start(FieldStart{0, 1812}));
    // ... and the standard's again when it leads from one start found to the next
    r.forget();
    CHECK(r.found(FieldStart{0, 1550}, FieldStart{1, 1812}, lines) == Start(FieldStart{1, 1812}));
    CHECK(r.overdue(FieldStart{1, 1812}, 2075 + 2) == Start(FieldStart{0, 2075}));
  }
  {
    // a jump in the stream loses the count of lines: nothing is due, and the next start found is taken as it is
    FieldRhythm r;
    CHECK(r.found(none, odd, lines) == Start(odd));
    r.forget();
    CHECK(r.overdue(odd, 5000) == std::nullopt);
    CHECK(r.found(odd, FieldStart{1, 437}, lines) == Start(FieldStart{1, 437}));
    CHECK(r.overdue(FieldStart{1, 437}, 700 + 2) == Start(FieldStart{0, 700}));
  }
  {
    // ... and in a faint signal, whose next vertical sync may be many fields off, the fields go on as they were
    // until one shows: that one is taken as it is, and the rhythm runs on from it
    FieldRhythm r;
    CHECK(r.found(none, odd, lines) == Start(odd));
    r.doubt();
    CHECK(r.overdue(odd, even.line + 2) == Start(even));
    CHECK(r.seen(even, FieldStart{0, 600}, lines, FieldRhythm::Signal::held) == Start(FieldStart{0, 600}));
    CHECK(r.overdue(FieldStart{0, 600}, 862 + 2) == Start(FieldStart{1, 862}));
    CHECK(r.seen(FieldStart{0, 600}, FieldStart{1, 880}, lines, FieldRhythm::Signal::held) == std::nullopt);  // (no longer)
  }
  {
    // Vertical syncs seen off the rhythm (FieldRhythm::seen), 15 lines early here and ten fields apart: in a faint
    // signal, which shows few of them, the second is where the fields begin; in any other it takes three, and
    // each within eight fields of the one before.
    for (const FieldRhythm::Signal signal : {FieldRhythm::Signal::held, FieldRhythm::Signal::faint, FieldRhythm::Signal::steady}) {
      const bool faint = signal != FieldRhythm::Signal::steady;
      FieldRhythm r;
      CHECK(r.found(none, odd, lines) == Start(odd));
      CHECK(r.seen(even, FieldStart{0, 610}, lines, signal) == std::nullopt);
      const Start second = r.seen(FieldStart{1, 2987}, FieldStart{0, 3235}, lines, signal);
      CHECK(second == (faint ? Start(FieldStart{0, 3235}) : std::nullopt));
    }
  }
}

// The signal with the vertical sync of some fields damaged: of its 6 half lines of broad pulses, the first
// gone(field) are blanked. 6: the field has no vertical sync; 2 or 3: it shows a line late; 1: it shows as the
// other kind of field's.
static Signal without_vertical_sync(Signal s, const std::function<int(int64_t)> &gone,
                                    int per_field = kHalfLinesPerField) {
  const int64_t fields = static_cast<int64_t>(static_cast<double>(s.d.size()) / (kT / 2)) / per_field + 1;
  for (int64_t f = 0; f < fields; f++) {
    const double t = static_cast<double>(f * per_field + 6) * (kT / 2);
    s.put(t, t + gone(f) * (kT / 2), kBlank);
  }
  return s;
}

// Two fields of three without their vertical sync: every picture comes out, and the same as with it. Interlaced
// fields, the rhythm taken as read; and fields that are all alike, a rhythm the source has to show first (its first
// few fields are begun where the standard's would be, then its own is learnt: after that its pictures are the ones
// a decoder without the flywheel draws). Without the flywheel the fields with no vertical sync are not drawn.
static void fields_coast_through_a_missing_vertical_sync() {
  for (const int per_field : {525, 524}) {
    const Signal s = moving_stripes(1.0, per_field);
    const int from = per_field == 525 ? 8 : 20;  // once the rhythm is had
    const auto two_of_three = [](int64_t f0) { return [f0](int64_t f) { return f >= f0 && f % 3 != 0 ? 6 : 0; }; };
    const Signal hit = without_vertical_sync(s, two_of_three(from), per_field);
    const std::vector<Picture> whole = frames_of(s, PictureOpts{});
    const Watched coasted = watched(hit, PictureOpts{}, nullptr, DecoderOpts{}, kLateCoasted);
    // ... and gone from the very start: the rhythm is had from the fields that are found, a few fields apart
    const DecoderStats sparse = decode(without_vertical_sync(s, two_of_three(0), per_field));
    const bool sparse_ok = sparse.frames + 30 >= whole.size() && sparse.losses == 0;
    CHECK(sparse_ok);
    if (shown(sparse_ok))
      std::fprintf(stderr, "  %d half lines a field, two vertical syncs in three gone from the start: %llu pictures of "
                   "%zu (%llu coasted)\n", per_field, (unsigned long long)sparse.frames, whole.size(),
                   (unsigned long long)sparse.fields_coasted);
    const DecoderStats &st = coasted.st;
    const DecoderStats bare = decode(hit, kNoFlywheel);
    const bool learnt = same_pictures(whole, frames_of(s, PictureOpts{}, nullptr, kNoFlywheel), 40);
    const bool ok = whole.size() >= 50 && same_pictures(whole, coasted.pics) && learnt && st.fields_coasted >= 20 &&
                    st.acquisitions == 1 && st.losses == 0 && bare.frames + 20 <= whole.size() &&
                    bare.fields_coasted == 0;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %d half lines a field: %zu pictures (the last 40 as without the flywheel: %d), %zu with "
                   "two vertical syncs in three gone (%llu coasted), %llu without the flywheel\n", per_field,
                   whole.size(), learnt, coasted.pics.size(), (unsigned long long)st.fields_coasted,
                   (unsigned long long)bare.frames);
  }
}

// The vertical sync damaged at its start, so that it is found a line late, or as the other kind of field's: the
// field begins where it is due all the same, as the kind it is, and the pictures are the same. Without the
// flywheel they are not: the field is drawn a line off, or into the other field's lines.
static void a_field_start_found_off_is_put_right() {
  const Signal s = moving_stripes(1.0);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  for (const int gone : {1, 2, 3}) {
    const Signal hit = without_vertical_sync(s, [gone](int64_t f) { return f >= 8 && f % 3 != 0 ? gone : 0; });
    const Watched w = watched(hit, PictureOpts{}, nullptr, DecoderOpts{}, kLateCoasted);
    const DecoderStats &st = w.st;
    const bool same = same_pictures(whole, w.pics);
    const bool bare = same_pictures(whole, frames_of(hit, PictureOpts{}, nullptr, kNoFlywheel, kLateCoasted));
    const bool ok = same && !bare && st.starts_put_right + st.fields_coasted >= 15 && st.losses == 0;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %d half lines of the vertical sync gone: pictures the same %d (without the flywheel %d), "
                   "%llu starts put right, %llu fields coasted\n", gone, same, bare,
                   (unsigned long long)st.starts_put_right, (unsigned long long)st.fields_coasted);
  }
}

// The source's fields move: 125 lines of it are cut out (a whole number of samples, so the line grid is unbroken).
// The field starts found are then all elsewhere than the rhythm says. Three of them in a row and they are
// believed: the pictures after that are the source's.
static void a_new_rhythm_is_believed() {
  const Signal s = moving_stripes(1.0);
  const size_t at = static_cast<size_t>(std::llround(40 * 125 * kT)), cut = static_cast<size_t>(std::llround(125 * kT));
  Signal moved_on;
  moved_on.d.assign(s.d.begin(), s.d.begin() + at);
  moved_on.d.insert(moved_on.d.end(), s.d.begin() + at + cut, s.d.end());
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const Watched w = watched(moved_on, PictureOpts{}, nullptr, DecoderOpts{}, kLateCoasted);
  const std::vector<Picture> &after = w.pics;
  const DecoderStats &st = w.st;
  const bool same = same_pictures(whole, after, 30, 3), ok = same && after.size() + 6 >= whole.size() && st.losses == 0;
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  125 lines cut out: %zu pictures (whole: %zu), the last 30 the same %d, %llu coasted\n",
                 after.size(), whole.size(), same, (unsigned long long)st.fields_coasted);
}

// The two together and each by itself, on the light flashing at 7 Hz: following the levels keeps the lock, and the
// flywheel begins the field whose vertical sync a jump landed on. The flywheel alone does not help there: with
// the lock gone at every flash there is no rhythm left to go by.
static void levels_and_flywheel_together_and_apart() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const DecoderStats steady = decode(plain);
  for (const double depths : {1.3, -1.3, -2.0}) {
    const Signal s = moved(plain, flashing(depths * (kBlank - kTip), 0.5e-3));
    const DecoderStats both = decode(s), levels = decode(s, kNoFlywheel),
                       fly = decode(s, kNoLevels), neither = decode(s, kNeither);
    const bool ok = both.losses == 0 && both.fields == steady.fields && levels.losses == 0 &&
                    levels.fields + 1 >= steady.fields && fly.losses >= 5 && neither.losses >= 5 &&
                    fly.frames + 20 < both.frames;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %+.1f depths: fields / losses with both %llu / %llu, levels alone %llu / %llu, flywheel "
                   "alone %llu / %llu, neither %llu / %llu (steady: %llu)\n", depths,
                   (unsigned long long)both.fields, (unsigned long long)both.losses,
                   (unsigned long long)levels.fields, (unsigned long long)levels.losses,
                   (unsigned long long)fly.fields, (unsigned long long)fly.losses,
                   (unsigned long long)neither.fields, (unsigned long long)neither.losses,
                   (unsigned long long)steady.fields);
  }
}

int main() {
  if (std::getenv("KP_TEST_DEBUG")) set_log_debug(true);
  locks_on_a_clean_signal();
  a_dip_ahead_of_the_hsync();
  a_gap_in_the_hsync();
  a_step_in_the_line_timing();
  strips_add_up_to_the_frames();
  frames_say_when_their_lines_were_taken();
  the_window_keeps_the_whole_picture();
  a_click_at_the_end_of_the_picture();
  the_levels_jump_with_a_flashing_light();
  the_picture_keeps_its_grey_through_a_jump();
  the_rhythm_on_line_numbers();
  fields_coast_through_a_missing_vertical_sync();
  a_field_start_found_off_is_put_right();
  a_new_rhythm_is_believed();
  levels_and_flywheel_together_and_apart();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
