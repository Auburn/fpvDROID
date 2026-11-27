// kp_replay: run fpvDROID's scanner DSP and video decoder over a recording (.cs16 + .json from wsdr_probe,
// or .sigmf-data + .sigmf-meta) and print lock numbers. The bench for lock robustness.
//
//   kp_replay REC.cs16 [--offset HZ] [--start S] [--seconds S] [--cnr DB] [--fs HZ] [--mix HZ] [--flash MHZ]
//             [--as20] [--frames] [--woven] [--comb] [--luma-comb] [--adaptive] [--smooth] [--repair]
//             [--no-levels] [--no-flywheel] [--no-hold] [--no-filter] [--grid FILE] [-v]
//             [--loop N] [--fade T:DB,T:DB,...] [--fade-every S] [--drop T:N,T:N,...]
//   --comb: the 3-line colour comb. --luma-comb, --adaptive, --smooth, --repair: the picture switches of video.h
//   (comb filter, clean OSD edges, smooth lines, sparkle repair). --as20: as the adapter delivers it at 20 MS/s.
//   --mix HZ: the recording shifted by HZ first (scanner edge tests). --fs HZ: for a recording without a .json.
//   --flash MHZ: the transmitter's frequency down by MHZ for half of every 1/7 s, as when its sync levels jump with
//   a light flashing in the picture (an AC-coupled video input): the lock should hold through it.
//   --no-levels, --no-flywheel, --no-hold, --no-filter: without the decoder's following of such jumps, without its
//   vertical flywheel, without its holding of the grid through a weak signal, without the front end's channel filter
//   for a noisy one (DecoderOpts), to see what each does by itself.
//   --grid FILE: where the decoder has its line grid and its field, a row for every block of 65536 samples: samples
//   done, acquiring, the line's start (samples), line period (us), field, the field's start (samples), tip, blank,
//   lock, frames, held, sync snr, channel filter (MHz), --fade's ratio now (dB; 0 without), the sync depth of better
//   times, colour on. For holding against where the signal's lines really are.
//   --loop N: the first N samples (from --start) over and over, a long signal out of a short recording: with a whole
//   number of frames the lines and the fields go on across the seam. --fade T:DB,T:DB,...: noise added for a
//   carrier-to-noise ratio that follows these points (seconds of the stream, straight lines between them; 99 and
//   over: none), and --fade-every S: the same again every S seconds. --drop T:N,T:N,...: N samples of the recording
//   missing at T seconds of the stream, as when the phone drops a block.
//   kp_replay REC.cs16 --frames | ffplay -f rawvideo -pixel_format rgb0 -video_size 720x480 -i -
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "log.h"
#include "spectrum.h"
#include "video.h"

using namespace kp;

// "a:b,a:b,...": pairs of numbers
static std::vector<std::pair<double, double>> pairs_of(const char *s) {
  std::vector<std::pair<double, double>> out;
  for (;;) {
    char *e;
    const double a = std::strtod(s, &e);
    if (e == s || *e != ':') break;
    const double b = std::strtod(e + 1, &e);
    out.emplace_back(a, b);
    if (*e != ',') break;
    s = e + 1;
  }
  return out;
}

static double json_number(const std::string &txt, const char *key) {
  const auto p = txt.find(std::string("\"") + key + "\"");
  if (p == std::string::npos) return NAN;
  const auto c = txt.find(':', p + std::strlen(key) + 2);  // after the key (sigmf keys contain ':')
  return std::atof(txt.c_str() + c + 1);
}

int main(int argc, char **argv) {
  std::string file;
  double offset = NAN, start = 0, seconds = 0, cnr = 0, fs = 40e6, mix = 0, flash = 0, fade_every = 0;
  size_t loop = 0;
  std::vector<std::pair<double, double>> fade, drops;  // (seconds, dB), (seconds, samples)
  bool frames = false, as20 = false;
  std::string grid_file;
  PictureOpts pic;
  DecoderOpts dec;
  for (int i = 1; i < argc; i++) {
    const std::string k = argv[i];
    auto v = [&] { return i + 1 < argc ? argv[++i] : (std::fprintf(stderr, "missing value\n"), std::exit(2), ""); };
    if (k == "--offset") offset = std::atof(v());
    else if (k == "--start") start = std::atof(v());
    else if (k == "--seconds") seconds = std::atof(v());
    else if (k == "--cnr") cnr = std::atof(v());
    else if (k == "--fs") fs = std::atof(v());
    else if (k == "--mix") mix = std::atof(v());
    else if (k == "--flash") flash = std::atof(v());
    else if (k == "--no-levels") dec.follow_levels = false;
    else if (k == "--no-flywheel") dec.flywheel = false;
    else if (k == "--no-hold") dec.hold = false;
    else if (k == "--no-filter") dec.channel_filter = false;
    else if (k == "--grid") grid_file = v();
    else if (k == "--loop") loop = static_cast<size_t>(std::atof(v()));
    else if (k == "--fade") fade = pairs_of(v());
    else if (k == "--fade-every") fade_every = std::atof(v());
    else if (k == "--drop") drops = pairs_of(v());
    else if (k == "--as20") as20 = true;  // emulate the adapter at 20 MS/s: ~18.4 MHz filter, decimate by 2
    else if (k == "--frames") frames = true;
    else if (k == "--woven") dec.fields = false;
    else if (k == "--comb") pic.chroma_comb = true;
    else if (k == "--luma-comb") pic.luma_comb = true;
    else if (k == "--adaptive") pic.adaptive_comb = true;
    else if (k == "--repair") pic.repair = true;
    else if (k == "--smooth") pic.smooth_lines = true;
    else if (k == "-v") set_log_debug(true);
    else if (k[0] != '-') file = k;
    else {
      std::fprintf(stderr, "usage: kp_replay REC.cs16 [--offset HZ] [--start S] [--seconds S] [--cnr DB] [--fs HZ] [--mix HZ] [--flash MHZ]\n"
                           "       [--as20] [--frames] [--woven] [--comb] [--luma-comb] [--adaptive] [--smooth] [--repair]\n"
                           "       [--no-levels] [--no-flywheel] [--no-hold] [--no-filter] [--grid FILE] [-v]\n"
                           "       [--loop N] [--fade T:DB,T:DB,...] [--fade-every S] [--drop T:N,T:N,...]\n");
      return 2;
    }
  }
  if (file.empty()) return 2;
  double centre = NAN;
  {
    const auto dot = file.rfind('.');
    const std::string base = file.substr(0, dot), ext = file.substr(dot);
    std::ifstream m(ext == ".sigmf-data" ? base + ".sigmf-meta" : base + ".json");
    if (m) {
      const std::string txt((std::istreambuf_iterator<char>(m)), {});
      double r = json_number(txt, ext == ".sigmf-data" ? "core:sample_rate" : "sample_rate");
      if (!std::isnan(r)) fs = r;
      if (ext == ".sigmf-data") centre = json_number(txt, "core:frequency");
      else if (!std::isnan(json_number(txt, "centre_hz"))) {
        centre = json_number(txt, "centre_hz");  // fpvDROID's own recordings
      } else {
        const double ch = json_number(txt, "channel") + (fs >= 40e6 ? 2 : 0);  // primary -> centre at BW40
        centre = (5000 + 5 * ch) * 1e6;
      }
    }
  }
  FILE *f = std::fopen(file.c_str(), "rb");
  if (!f) {
    std::perror(file.c_str());
    return 1;
  }
  std::fseek(f, static_cast<long>(start * fs) * 4, SEEK_SET);
  const size_t total = seconds > 0 ? static_cast<size_t>(seconds * fs) : SIZE_MAX;

  std::vector<int16_t> raw(2 * 65536);
  std::vector<cf> x(65536);
  double mix_ph = 0;
  auto read_block = [&](size_t want, size_t at = 0) -> size_t {  // into x from x[at]
    const size_t n = std::fread(raw.data(), 4, std::min<size_t>(want, 65536 - at), f);
    for (size_t i = 0; i < n; i++) x[at + i] = cf(raw[2 * i] / 16.f, raw[2 * i + 1] / 16.f);
    if (mix != 0)
      for (size_t i = at; i < at + n; i++) {
        x[i] *= cf(static_cast<float>(std::cos(mix_ph)), static_cast<float>(std::sin(mix_ph)));
        mix_ph = std::fmod(mix_ph + 2 * M_PI * mix / fs, 2 * M_PI);
      }
    return n;
  };

  // ---- scanner view of the first 25 ms: humps + sync check ----
  {
    const long pos = std::ftell(f);
    const size_t want = static_cast<size_t>(0.1 * fs);
    std::vector<cf> buf;
    while (buf.size() < want) {
      const size_t n = read_block(want - buf.size());
      if (!n) break;
      buf.insert(buf.end(), x.begin(), x.begin() + n);
    }
    Psd psd(1024);
    psd.add(buf.data(), std::min(buf.size(), static_cast<size_t>(0.025 * fs)));
    const auto humps = find_humps(psd.db(), fs, 0.42 * fs);
    std::fprintf(stderr, "scan: %zu hump(s) in +-%.0f MHz around %.1f MHz\n", humps.size(), 0.42 * fs / 1e6, centre / 1e6);
    for (const auto &h : humps) {
      const auto vc = check_video(buf.data(), buf.size(), fs, h.offset_hz);
      const double mhz = (centre + h.offset_hz - mix) / 1e6;  // --mix moves the signal, not the centre
      const auto *ch = nearest_channel(mhz);
      std::fprintf(stderr, "  %+.2f MHz (%.1f MHz %s) width %.1f MHz snr %.1f dB -> %s %s lock %.2f dev %.1f MHz\n",
                   h.offset_hz / 1e6, mhz, ch ? ch->name : "--", h.width_hz / 1e6, h.snr_db,
                   vc.video ? "VIDEO" : "no video", vc.std_name, vc.lock, vc.dev_mhz);
      if (std::isnan(offset) && vc.video) offset = h.offset_hz;
    }
    std::fseek(f, pos, SEEK_SET);
  }
  if (std::isnan(offset)) {
    std::fprintf(stderr, "no video found; pass --offset\n");
    offset = -1e6;
  }

  // ---- decode ----
  // --as20: a 63-tap low-pass at 9.2 MHz (the chip's BW20 filter is ~18.4 MHz at -3 dB), then every other
  // sample; in place
  std::vector<float> h20(63);
  std::vector<cf> hist20(62, cf{});
  for (int i = 0; i < 63; i++) {
    const double k = i - 31, w = 0.42 - 0.5 * std::cos(2 * M_PI * i / 62) + 0.08 * std::cos(4 * M_PI * i / 62);
    h20[i] = static_cast<float>((k == 0 ? 2 * 9.2e6 / 40e6 : std::sin(2 * M_PI * 9.2e6 / 40e6 * k) / (M_PI * k)) * w);
  }
  int ph20 = 0;
  auto dec20 = [&](cf *xs, size_t n) -> size_t {
    std::vector<cf> w(hist20);
    w.insert(w.end(), xs, xs + n);
    size_t o = 0;
    for (size_t i = 62; i < w.size(); i++) {
      if ((ph20++ & 1) == 0) {
        cf acc{};
        for (int t = 0; t < 63; t++) acc += w[i - t] * h20[t];
        xs[o++] = acc;
      }
    }
    hist20.assign(w.end() - 62, w.end());
    return o;
  };
  const double fs_dec = as20 ? fs / 2 : fs;
  int fw = 0, fh = 0;
  FrontEnd fe(fs_dec, offset, [&](const Frame &fr) {
    fw = fr.width;
    fh = fr.height;
    if (frames) std::fwrite(fr.data, 1, static_cast<size_t>(fr.width) * fr.height * 4, stdout);
  }, dec, pic);
  std::mt19937 rng(1);
  std::normal_distribution<float> nd(0.f, 1.f);
  float noise_sd = 0;
  double noise_0db = 0;  // (--fade) the noise's sd at a ratio of 0 dB
  if (cnr > 0 || !fade.empty()) {  // add white noise for this carrier-to-noise ratio in an 18 MHz video channel
    double p = 0;
    const size_t n = read_block(1 << 16);
    for (size_t i = 0; i < n; i++) p += std::norm(x[i]);
    p /= std::max<size_t>(1, n);
    // p is ~all signal (the recordings are ~30 dB above the floor); noise spreads over the whole fs
    const double noise_total = p / std::pow(10, cnr / 10) * (fs / 18e6);
    if (cnr > 0) noise_sd = static_cast<float>(std::sqrt(noise_total / 2));
    noise_0db = std::sqrt(p * (fs / 18e6) / 2);
    std::fseek(f, static_cast<long>(start * fs) * 4, SEEK_SET);
    std::fprintf(stderr, "adding noise: sd %.1f per component (input power %.1f)\n", noise_sd, p);
  }
  auto fade_db = [&](double t) -> double {  // --fade's ratio at t seconds
    if (fade_every > 0) t = std::fmod(t, fade_every);
    for (size_t i = 0; i < fade.size(); i++)
      if (t < fade[i].first) {
        if (i == 0) break;
        const double u = (t - fade[i - 1].first) / (fade[i].first - fade[i - 1].first);
        return fade[i - 1].second + u * (fade[i].second - fade[i - 1].second);
      }
    return t < fade.front().first ? fade.front().second : fade.back().second;
  };
  FILE *grid = grid_file.empty() ? nullptr : std::fopen(grid_file.c_str(), "w");
  std::vector<float> dbuf;
  std::vector<uint8_t> bad;
  double t_demod = 0, t_decode = 0, flash_ph = 0;
  size_t done = 0, next_report = static_cast<size_t>(fs);
  uint64_t last_frames = 0;
  const auto t0 = std::chrono::steady_clock::now();
  // The stream's next block: the recording, round its loop (--loop), without what goes missing (--drop).
  size_t src = 0, drop_i = 0;  // the next sample from the replay's start; the next of the drops
  auto seek_src = [&] { std::fseek(f, (static_cast<long>(start * fs) + static_cast<long>(src)) * 4, SEEK_SET); };
  auto next_block = [&](size_t want) -> size_t {
    size_t n = 0;
    while (n < want) {
      size_t part = want - n;
      if (drop_i < drops.size()) {
        const size_t at = static_cast<size_t>(drops[drop_i].first * fs);
        if (done + n >= at) {
          src += static_cast<size_t>(drops[drop_i++].second);
          if (loop) src %= loop;
          seek_src();
          continue;
        }
        part = std::min(part, at - (done + n));
      }
      if (loop) {
        if (src >= loop) {
          src = 0;
          seek_src();
        }
        part = std::min(part, loop - src);
      }
      const size_t got = read_block(part, n);
      if (!got) break;
      n += got;
      src += got;
    }
    return n;
  };
  while (done < total) {
    const size_t n = next_block(std::min<size_t>(65536, total - done));
    if (!n) break;
    if (flash != 0)
      for (size_t i = 0; i < n; i++) {
        const double t7 = static_cast<double>(done + i) / fs * 7;
        if (t7 - std::floor(t7) >= 0.5) flash_ph = std::remainder(flash_ph - 2 * M_PI * flash * 1e6 / fs, 2 * M_PI);
        x[i] *= cf(static_cast<float>(std::cos(flash_ph)), static_cast<float>(std::sin(flash_ph)));
      }
    if (noise_sd > 0)
      for (size_t i = 0; i < n; i++) x[i] += cf(noise_sd * nd(rng), noise_sd * nd(rng));
    for (size_t i = 0; i < n && !fade.empty(); i += 256) {
      const double db = fade_db(static_cast<double>(done + i) / fs);
      if (db >= 99) continue;
      const float sd = static_cast<float>(noise_0db / std::pow(10, db / 20));
      for (size_t j = i; j < std::min(n, i + 256); j++) x[j] += cf(sd * nd(rng), sd * nd(rng));
    }
    const auto ta = std::chrono::steady_clock::now();
    size_t nn = n;
    if (as20) nn = dec20(x.data(), n);
    fe.demod(x.data(), nn, dbuf, bad);
    const auto tb = std::chrono::steady_clock::now();
    fe.decode(dbuf, bad);
    t_demod += std::chrono::duration<double>(tb - ta).count();
    t_decode += std::chrono::duration<double>(std::chrono::steady_clock::now() - tb).count();
    done += n;
    if (grid) {
      const auto s = fe.stats();
      std::fprintf(grid, "%zu %d %.2f %.5f %d %.2f %.3f %.3f %.3f %llu %d %.2f %.0f %.3f %.3f %d\n", done, s.acquiring ? 1 : 0,
                   s.grid, s.line_us, s.field, s.field0, s.tip, s.blank, s.lock_recent, (unsigned long long)s.frames,
                   s.holding ? 1 : 0, s.sync_snr, s.filter_mhz, fade.empty() ? 0.0 : fade_db(static_cast<double>(done) / fs),
                   s.depth_ref, s.colour ? 1 : 0);
    }
    if (done >= next_report) {
      next_report += static_cast<size_t>(fs);
      const auto s = fe.stats();
      std::fprintf(stderr, "t=%5.1fs %s %s lock(recent) %.2f lines %llu/%llu fields %llu frames +%llu acq %llu lost %llu tip %.2f blank %.2f sync snr %.1f filter %.0f MHz colour %s (burst err %.2f)\n",
                   done / fs, s.acquiring ? "ACQ " : "LOCK", s.std_name, s.lock_recent, (unsigned long long)s.lines_locked,
                   (unsigned long long)s.lines, (unsigned long long)s.fields, (unsigned long long)(s.frames - last_frames),
                   (unsigned long long)s.acquisitions, (unsigned long long)s.losses, s.tip, s.blank, s.sync_snr,
                   s.filter_mhz, s.colour ? "on" : "off", s.burst_err);
      last_frames = s.frames;
    }
  }
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const auto s = fe.stats();
  std::fprintf(stderr, "done: %.2f s of IQ in %.2f s (%.1fx realtime), offset %+.3f MHz, frames %llu (%dx%d), "
               "acquisitions %llu, losses %llu, re-phases %llu, level jumps %llu, fields coasted %llu put right %llu, "
               "holds %llu, filter changes %llu, lines locked %.1f%%\n",
               done / fs, dt, done / fs / dt, offset / 1e6, (unsigned long long)s.frames, fw, fh,
               (unsigned long long)s.acquisitions, (unsigned long long)s.losses, (unsigned long long)s.rephases,
               (unsigned long long)s.level_jumps, (unsigned long long)s.fields_coasted,
               (unsigned long long)s.starts_put_right, (unsigned long long)s.holds,
               (unsigned long long)s.filter_changes, s.lines ? 100.0 * s.lines_locked / s.lines : 0.0);
  std::fprintf(stderr, "cpu per second of IQ: demod %.0f ms, decode %.0f ms\n", t_demod / (done / fs) * 1e3,
               t_decode / (done / fs) * 1e3);
  if (pic.repair)
    std::fprintf(stderr, "sparkle repair: %llu clicks patched, %llu lines mended\n", (unsigned long long)s.clicks_repaired,
                 (unsigned long long)s.lines_mended);
  return 0;
}
