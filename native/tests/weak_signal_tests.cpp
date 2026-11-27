// kp_weak_signal_tests: the decoder on a weak signal made here (ctest: "weak_signal"): the hold and its fold, a fade,
// a weak signal from the start, nothing for a moment, a dropout, a block missing, colour in noise and the channel
// filter. What it shares with kp_decoder_tests (decoder_tests.cpp) is in decoder_tests.h.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

#include "decoder_tests.h"
#include "fold.h"
#include "log.h"
#include "sync_noise.h"
#include "video.h"

// ---------------------------------------------------------------------------------------- a weak signal

// The fold by itself (fold.h), on a fold made here: blanking at 0, an hsync 4.7 us long down at -2 that begins at
// `edge` (bins, to a fraction: a bin an edge falls in has its share of both levels), picture from 9.4 us after it.
// The hsync is found by its shape, its edge to a hundredth of a bin, and its levels. In a whole line's fold too,
// where it lies across the fold's ends, and where something dark in the picture looks like a pulse as well (less
// deep than the sync is): the hsync is the line's best pulse and the lowest thing in it, and the dark bar, looked at
// by itself, is not. A fold of nothing has none.
static void the_fold_finds_the_hsync() {
  const auto level = [](Fold &f, double from, double to, float v) {  // bins [from, to) at v, wrapping
    const int K = static_cast<int>(f.v.size());
    for (int i = static_cast<int>(std::floor(from)); i < to; i++) {
      const double share = std::min<double>(i + 1, to) - std::max<double>(i, from);
      float &bin = f.v[static_cast<size_t>(((i % K) + K) % K)];
      bin += static_cast<float>(share) * (v - bin);
    }
  };
  {
    Fold f;  // a stretch around where the hsync is due, a sample to a bin
    f.shape(static_cast<size_t>(21 * kUs), kFs, 1, false);
    const double edge = 7 * kUs + 3.3;
    level(f, edge, edge + 4.7 * kUs, kTip);
    const std::optional<FoldPulse> p = f.pulse(f.bins(2.5), f.bins(11.5));
    CHECK(p && std::fabs(p->edge - edge) < 0.01 && std::fabs(p->tip - kTip) < 0.01f && std::fabs(p->blank - kBlank) < 0.01f &&
          std::fabs(p->depth - (kBlank - kTip)) < 0.01f);
    Fold none;
    none.shape(f.v.size(), kFs, 1, false);
    CHECK(!none.pulse(none.bins(2.5), none.bins(11.5)));
  }
  {
    Fold f;  // a whole line, four samples to a bin, the hsync two bins before its end
    const int K = static_cast<int>(kT / 4);
    f.shape(static_cast<size_t>(K), kFs, 4, true);
    const double edge = K - 2.4, bar = f.bins(30);
    level(f, edge, edge + f.bins(4.7), kTip);
    level(f, edge + f.bins(9.4), edge + f.bins(62), kGrey);
    level(f, bar, bar + f.bins(5), 0.2f);  // something dark in the picture, as long as a pulse (not as deep)
    const std::optional<FoldPulse> p = f.pulse(0, K - 1), dark = f.pulse(static_cast<int>(bar) - 4, static_cast<int>(bar) + 4);
    CHECK(p && std::fabs(p->edge - edge) < 0.01 && std::fabs(p->tip - kTip) < 0.01f && p->lowest_there());
    // (the bar passes for a pulse by its shape, but lies well above the lowest thing in the line)
    CHECK(dark && std::fabs(dark->edge - bar) < 0.5 && dark->tip > p->lowest + 0.25f * dark->depth);
  }
}

// Noisy or not goes by how much of the time (sync_noise.h): one noisy moment in ten is not noise; noisy most of
// the time is, after some thirty measurements at a fiftieth each; between the two limits it stays what it was;
// and it is clean again once it has been over the upper one for four fifths of the time.
static void noisy_by_how_much_of_the_time() {
  NoisyLatch l{12, 16};
  for (int i = 0; i < 300; i++) l.update(i % 10 == 0 ? 5.f : 30.f, 0.02f);
  CHECK(!l.noisy);
  int n = 0;
  while (!l.noisy && n < 1000) l.update(5.f, 0.02f), n++;
  CHECK(l.noisy && n > 20 && n < 40);
  for (int i = 0; i < 300; i++) l.update(14.f, 0.02f);
  CHECK(l.noisy);
  n = 0;
  while (l.noisy && n < 1000) l.update(30.f, 0.02f), n++;
  CHECK(!l.noisy && n > 60 && n < 100);
}

// A faded signal through a decoder with the switches o, once (whole: the clean signal's pictures). What came out, how
// many of its pictures begin where the clean signal's do, whether the last 8 are the clean signal's to a rounding, and
// whether the signal was kept all through: the lock never lost, one acquisition, no re-phase, every picture out and
// each in its place, the last 8 the clean signal's.
struct Kept {
  Watched w;
  size_t placed = 0;
  bool same = false, ok = false;
};
static Kept kept_through(const std::vector<Picture> &whole, const Signal &f, const DecoderOpts &o = DecoderOpts{}) {
  Kept k{watched(f, PictureOpts{}, nullptr, o, 18)};
  const DecoderStats &st = k.w.st;
  k.placed = in_place(whole, k.w.pics);
  k.same = same_pictures(whole, k.w.pics, 8, 3);
  k.ok = st.losses == 0 && st.acquisitions == 1 && st.rephases == 0 && k.w.pics.size() == whole.size() &&
         k.placed == k.w.pics.size() && k.same;
  return k;
}

// 0.4 s of a signal in which a line's own hsync is a matter of chance (noise through the sync path as large as
// half the sync is deep, or as large as all of it, and the sync half as deep as it was): the lock is not lost,
// every picture comes out and begins where the signal's does, and afterwards the pictures are the clean signal's
// again, to the pixel: no line and no field was slipped. Without the hold the lock goes, and pictures with it.
static void the_grid_is_held_through_a_fade() {
  const Signal s = moving_stripes(1.0);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  for (const float sd : {3.f, 6.f}) {
    const Signal f = faded(s, 0.3, 0.7, 0.5f, sd);
    const Kept k = kept_through(whole, f);
    const DecoderStats &st = k.w.st;
    const DecoderStats bare = decode(f, kNoHold);
    const bool ok = k.ok && st.holds >= 1 && !st.holding && bare.losses >= 1 && bare.frames + 5 <= whole.size();
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  a fade, noise %.0f MHz: %zu pictures of %zu, %zu in place, the last 8 the same %d; %llu holds, "
                   "%llu losses, %llu re-phases, %llu coasted; without the hold %llu pictures, %llu losses\n", sd,
                   k.w.pics.size(), whole.size(), k.placed, k.same, (unsigned long long)st.holds,
                   (unsigned long long)st.losses, (unsigned long long)st.rephases,
                   (unsigned long long)st.fields_coasted, (unsigned long long)bare.frames,
                   (unsigned long long)bare.losses);
  }
}

// Such a signal from the first sample: it is acquired, the fold holds its lines, and its fields are found by what
// their half lines are (the broad pulses do not show as one run under the threshold in that noise): nearly every
// picture comes out, in its place. Without the hold next to none does.
static void a_weak_signal_from_the_start() {
  const Signal s = moving_stripes(1.0);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const Signal f = faded(s, 0, 1.0, 0.5f, 6.f);
  const Kept k = kept_through(whole, f);
  const DecoderStats &st = k.w.st;
  const DecoderStats bare = decode(f, kNoHold);
  const size_t got = k.w.pics.size();
  const bool ok = st.losses == 0 && st.holding && got + 8 >= whole.size() && k.placed + 2 >= got && bare.frames * 2 < got;
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  weak from the start: %zu pictures of %zu, %zu in place; %llu acquisitions, %llu losses, %llu "
                 "holds; without the hold %llu pictures\n", got, whole.size(), k.placed,
                 (unsigned long long)st.acquisitions, (unsigned long long)st.losses, (unsigned long long)st.holds,
                 (unsigned long long)bare.frames);
}

// No signal at all, only noise, for 0.2 s: the grid runs on at the line period, pictures of noise come out in
// their places, and when the signal is back so are its pictures, to the pixel. For 0.8 s, after a lock of 0.6 s:
// the lock is given up after as long as it had lasted, and found again when the signal is back.
static void nothing_for_a_moment_and_for_long() {
  const Signal s = moving_stripes(1.8);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const Kept k = kept_through(whole, faded(s, 0.6, 0.8, 0.f, 6.f));
  const DecoderStats &st = k.w.st;
  const DecoderStats gone = decode(faded(s, 0.6, 1.4, 0.f, 6.f));
  CHECK(k.ok);
  CHECK(gone.losses == 1 && gone.acquisitions == 2 && !gone.acquiring && gone.lock_recent > 0.9f);
  if (shown(k.ok && gone.losses == 1))
    std::fprintf(stderr, "  nothing for 0.2 s: %zu pictures of %zu, %zu in place, the last 8 the same %d, %llu losses, %llu "
                 "re-phases; for 0.8 s: %llu losses, %llu acquisitions, lock %.2f\n", k.w.pics.size(), whole.size(),
                 k.placed, k.same, (unsigned long long)st.losses, (unsigned long long)st.rephases,
                 (unsigned long long)gone.losses, (unsigned long long)gone.acquisitions, gone.lock_recent);
}

// The signal gone altogether for a second, after 1.4 s of it, as when the transmitter is out of reach for a
// moment: the lines and the fields coast through the noise (nothing found in it moves them), the lock is kept,
// and when the signal is back so are its pictures, in their places, to the pixel. Without coasting the lock is
// given up after half a second and found again only after the signal is back: pictures are missing.
static void a_dropout_is_coasted_through() {
  const Signal s = moving_stripes(2.8);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const Signal f = faded(s, 1.4, 2.4, 0.f, 6.f);
  const Kept k = kept_through(whole, f);
  const DecoderStats &st = k.w.st;
  const DecoderStats bare = decode(f, kNoCoast);
  const bool ok = k.ok && bare.losses == 1 && bare.acquisitions == 2 && bare.frames + 5 <= whole.size();
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  gone for 1 s: %zu pictures of %zu, %zu in place, the last 8 the same %d; %llu losses, %llu "
                 "re-phases, %llu coasted; without coasting %llu pictures, %llu losses, %llu acquisitions\n",
                 k.w.pics.size(), whole.size(), k.placed, k.same, (unsigned long long)st.losses,
                 (unsigned long long)st.rephases, (unsigned long long)st.fields_coasted,
                 (unsigned long long)bare.frames, (unsigned long long)bare.losses,
                 (unsigned long long)bare.acquisitions);
}

// A block of samples missing from such a signal, as when the phone drops one: 30 lines and a part of a line. From
// there on the signal's lines are not where the grid has them, and its fields begin 30 lines early. The fold of
// the whole line shows where the hsync has gone, and the grid is put there; the pictures go on coming meanwhile,
// at the rhythm they had, and 0.2 s on they begin where the signal's do. Two ways for it to go: nothing like a
// pulse is left where the grid is (stripes, the signal 20 us early); or there is the dark edge of something in the
// picture there, which passes for a pulse (a bar 15 us into each line, the signal 15 us early), and the hsync,
// lower than anything in the picture, is the whole line's best pulse all the same.
static void a_block_missing_from_a_weak_signal() {
  struct Case {
    const char *name;
    Signal s;
    double us;
  };
  const Signal bar = video(1.6, on_the_grid, [](Signal &s, int64_t, double start) {
    s.put(start + 15 * kUs, start + 20 * kUs, 0.2f);
  });
  for (Case c : {Case{"stripes, 20 us early", moving_stripes(1.6), 20}, Case{"a dark bar where the grid is", bar, 15}}) {
    const size_t at = static_cast<size_t>(0.6 * kFs);
    c.s.d.erase(c.s.d.begin() + at, c.s.d.begin() + at + static_cast<size_t>(30 * kT + c.us * kUs));
    // (as a strong signal, its lines found again at once, no field is begun between the jump and the next
    // vertical sync found: the picture in hand at the jump comes out a field late)
    const std::vector<Picture> whole = frames_of(c.s, PictureOpts{}, nullptr, DecoderOpts{}, 300);
    const Signal f = faded(c.s, 0, 1.6, 0.5f, 6.f);
    const std::vector<Picture> got = frames_of(f, PictureOpts{}, nullptr, DecoderOpts{}, 18);
    const DecoderStats st = decode(f);
    // (when they began, without their pixels)
    const auto from = [](const std::vector<Picture> &v, double t0, double t1) {
      std::vector<Picture> out;
      for (const Picture &p : v)
        if (p.times.t_first >= kT0 + t0 && p.times.t_first < kT0 + t1) out.push_back(Picture{{}, p.times});
      return out;
    };
    const std::vector<Picture> after = from(got, 0.8, 1.6);
    const size_t meanwhile = from(got, 0.6, 0.8).size(), due = from(whole, 0.8, 1.6).size(), placed = in_place(whole, after);
    const bool ok = st.losses == 0 && st.acquisitions == 1 && st.rephases >= 1 && st.rephases <= 2 && meanwhile >= 10 &&
                    after.size() + 1 >= due && placed + 1 >= after.size();
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  a block missing, %s: %llu re-phases, %llu losses; %zu pictures in the 0.2 s after it, then %zu "
                   "of %zu, %zu in place\n", c.name, (unsigned long long)st.rephases, (unsigned long long)st.losses,
                   meanwhile, after.size(), due, placed);
  }
}

// ---------------------------------------------------------------------------------------- colour in noise

// The grey picture in one colour: a burst on every picture line's back porch (40 IRE peak to peak, as the sync is
// deep) and a subcarrier across the picture, `hue` radians from the burst. NTSC: 227.5 cycles a line, here on the
// samples' clock.
static Signal coloured(double seconds, double hue) {
  return video(seconds, on_the_grid, [hue](Signal &s, int64_t, double start) {
    const double w = 2 * M_PI * 227.5 / kT;
    const auto wave = [&](double from_us, double to_us, double amp, double ph) {
      const int64_t a = std::max<int64_t>(0, std::llround(start + from_us * kUs));
      const int64_t b = std::min<int64_t>(static_cast<int64_t>(s.d.size()), std::llround(start + to_us * kUs));
      for (int64_t i = a; i < b; i++) s.d[i] += static_cast<float>(amp * std::cos(w * static_cast<double>(i) + ph));
    };
    wave(5.3, 7.8, 1.0, M_PI);
    wave(9.4, 62.0, 0.6, M_PI + hue);
  });
}

// What colour the middle of the last `back` pictures has on average: red and blue against green, 0..255.
static std::pair<double, double> tint(const std::vector<Picture> &v, size_t back) {
  double rg = 0, bg = 0, n = 0;
  for (size_t k = 1; k <= back && k <= v.size(); k++) {
    const std::vector<uint8_t> &px = v[v.size() - k].px;
    const size_t w = 720, h = px.size() / 4 / w;
    for (size_t y = h / 4; y < 3 * h / 4; y += 2)
      for (size_t x = w / 4; x < 3 * w / 4; x += 2) {
        const uint8_t *p = &px[4 * (y * w + x)];
        rg += p[0] - p[1];
        bg += p[2] - p[1];
        n++;
      }
  }
  return {rg / std::max(n, 1.0), bg / std::max(n, 1.0)};
}

// A noisy signal keeps its colour. One line's burst is mostly noise then, and the decoder used to show no colour
// (with the hold), or bands of wrong colour (without): now the bursts are averaged over many lines, and the
// picture's colour is the clean signal's: the same hue, and the paler the larger the noise (a tenth of the clean
// signal's saturation where the noise through the colour path is some three times the burst). A noisy signal
// without a burst gets no colour from its noise.
static void a_noisy_signal_keeps_its_colour() {
  for (const double hue : {0.6, 2.4, 4.4}) {
    const Signal s = coloured(1.0, hue);
    const std::vector<Picture> whole = frames_of(s, PictureOpts{});
    const auto [r0, b0] = tint(whole, 8);
    const double size0 = std::hypot(r0, b0);
    double left[2] = {};
    bool ok = clean().colour == false && size0 > 20;
    int i = 0;
    for (const float sd : {1.5f, 3.f}) {
      const Signal f = faded(s, 0, 1.0, 0.5f, sd);
      const std::vector<Picture> got = frames_of(f, PictureOpts{}, nullptr, DecoderOpts{}, 18);
      const DecoderStats st = decode(f);
      const auto [r1, b1] = tint(got, 16);
      const double turn = std::fabs(std::remainder(std::atan2(b1, r1) - std::atan2(b0, r0), 2 * M_PI));
      left[i] = std::hypot(r1, b1) / std::max(size0, 1e-9);
      const bool one = st.colour && got.size() + 2 >= whole.size() && turn < 0.3;
      ok = ok && one;
      if (shown(one))
        std::fprintf(stderr, "  colour in noise %.1f MHz, hue %.1f: clean red-green %+.0f blue-green %+.0f; noisy %+.0f %+.0f "
                     "(%.0f %% of the size, %.2f rad off), colour %s, %zu pictures of %zu\n", sd, hue, r0, b0, r1, b1,
                     100 * left[i], turn, st.colour ? "on" : "off", got.size(), whole.size());
      i++;
    }
    CHECK(ok);
    CHECK(left[0] < 1.1 && left[0] > left[1] && left[1] > 0.05);
  }
  const DecoderStats grey = decode(faded(video(1.0, on_the_grid, nullptr), 0, 1.0, 0.5f, 3.f));
  CHECK(!grey.colour && grey.holding);
}

// ---------------------------------------------------------------------------------------- the channel filter

// The signal on its carrier, as the adapter gives it: FM with the discriminator's output as its frequency, the
// carrier `above` MHz over the stream's middle, and white noise `cnr_db` under the carrier (in the whole stream).
// (the signal's levels here are the test signal's: sync tip at -2 MHz, white near +4)
static std::vector<cf> on_a_carrier(const Signal &s, double cnr_db, double above) {
  std::vector<cf> x(s.d.size());
  Noise noise{0x2545f4914f6cdd1dull};
  const double sd = cnr_db > 90 ? 0 : std::sqrt(0.5 / std::pow(10, cnr_db / 10));
  double ph = 0;
  for (size_t i = 0; i < x.size(); i++) {
    ph = std::remainder(ph + 2 * M_PI * (s.d[i] + above) * 1e6 / kFs, 2 * M_PI);
    const auto [nr, ni] = noise.normal2(sd);
    x[i] = cf(static_cast<float>(std::cos(ph) + nr), static_cast<float>(std::sin(ph) + ni));
  }
  return x;
}

// The stream through a whole front end (the discriminator too), the carrier `above` MHz over its middle
static Watched watch(const std::vector<cf> &x, double above, const DecoderOpts &o = DecoderOpts{}) {
  Watched w;
  FrontEnd fe(kFs, above * 1e6, [&](const Frame &f) {
    if (!f.finished()) return;
    const size_t bytes = static_cast<size_t>(f.width) * f.height * 4;
    w.pics.push_back(Picture{std::vector<uint8_t>(f.data, f.data + bytes), f.times});
  }, o);
  std::vector<float> d;
  std::vector<uint8_t> bad;
  for (size_t at = 0; at < x.size(); at += 4096) {
    const size_t n = std::min<size_t>(4096, x.size() - at);
    fe.demod(x.data() + at, n, d, bad);
    fe.decode(d, bad, kT0 + static_cast<double>(at + n - 1) / kFs);
    w.widest = std::max(w.widest, fe.stats().filter_mhz);
  }
  w.st = fe.stats();
  return w;
}

// A clean signal never has the filter in, and its pictures are the ones a front end without the filter draws (the
// stream only comes 11 samples later). A noisy one has it in within a tenth of a second, as wide as six of the
// transmitter's sync depths, not retuned over and over, with the decoder's levels where they were, and more of
// the lines find their hsync than without it. From the first sample too, when the signal has never been better:
// the filter is put by a weak signal's own shrunken levels, and does not run off with them.
static void a_noisy_signal_gets_a_channel_filter() {
  DecoderOpts none;
  none.channel_filter = false;
  const Signal s = moving_stripes(0.8);
  const double above = 1.5;
  {
    const std::vector<cf> x = on_a_carrier(s, 99, above);
    const Watched with = watch(x, above), without = watch(x, above, none);
    const bool ok = with.widest == 0 && with.st.filter_changes == 0 && with.st.losses == 0 &&
                    with.pics.size() == without.pics.size() && same_pictures(with.pics, without.pics, 20, 3);
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  clean: filter %.0f MHz at its widest, %llu changes, %zu pictures (without: %zu), sync snr "
                   "%.0f\n", with.widest, (unsigned long long)with.st.filter_changes, with.pics.size(),
                   without.pics.size(), with.st.sync_snr);
  }
  for (const double cnr : {6.0, 3.0}) {
    // clean for 0.25 s, then the noise; and the noise from the start
    for (const bool from_start : {false, true}) {
      std::vector<cf> x = on_a_carrier(s, cnr, above);
      if (!from_start) {
        const std::vector<cf> clean = on_a_carrier(s, 99, above);
        std::copy(clean.begin(), clean.begin() + static_cast<long>(0.25 * kFs), x.begin());
      }
      const Watched with = watch(x, above), without = watch(x, above, none);
      const float depth = kBlank - kTip;
      const bool ok = with.st.filter_mhz >= 7 && with.st.filter_mhz <= 13 && with.st.filter_changes <= 4 &&
                      with.st.losses == 0 && with.st.lines_locked > without.st.lines_locked &&
                      std::fabs(with.st.tip - without.st.tip) < 0.25f * depth &&
                      std::fabs(with.st.blank - without.st.blank) < 0.25f * depth &&
                      with.pics.size() + 3 >= without.pics.size();
      CHECK(ok);
      if (shown(ok))
        std::fprintf(stderr, "  carrier %.0f dB over the noise%s: filter %.0f MHz, %llu changes, lines locked %llu "
                     "(without: %llu), tip %.2f blank %.2f (without: %.2f %.2f), sync snr %.1f (without: %.1f), %zu "
                     "pictures (without: %zu), %llu losses\n", cnr, from_start ? " from the start" : "",
                     with.st.filter_mhz, (unsigned long long)with.st.filter_changes,
                     (unsigned long long)with.st.lines_locked, (unsigned long long)without.st.lines_locked,
                     with.st.tip, with.st.blank, without.st.tip, without.st.blank, with.st.sync_snr,
                     without.st.sync_snr, with.pics.size(), without.pics.size(), (unsigned long long)with.st.losses);
    }
  }
}

int main() {
  if (std::getenv("KP_TEST_DEBUG")) set_log_debug(true);
  the_fold_finds_the_hsync();
  noisy_by_how_much_of_the_time();
  the_grid_is_held_through_a_fade();
  a_weak_signal_from_the_start();
  nothing_for_a_moment_and_for_long();
  a_dropout_is_coasted_through();
  a_block_missing_from_a_weak_signal();
  a_noisy_signal_keeps_its_colour();
  a_noisy_signal_gets_a_channel_filter();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
