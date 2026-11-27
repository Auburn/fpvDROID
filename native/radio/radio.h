// fpvDROID radio: one RTL8812BU (wifi-sdr Receiver) + a DSP thread. Modes:
//   scan   - sweep the analog FPV bands at 40 MS/s, list pilots (humps that pass a sync check)
//   watch  - tune to one pilot and decode its video (frames to the FrameSink)
//   test   - stream at a given rate and only count banks (USB/controller soak, no DSP)
//   sweep  - the spectrum page: step across a span over and over for its spectrum alone (no sync check, no picture)
// The Receiver's sink only copies banks into a queue (wifi-sdr rule); everything else runs on the DSP thread.
// Two-antenna combining, off-centre tuning and the sparkle repair plumbing come from Play RX
// (github.com/scerstt/PlayRX, GPL-2.0), a Windows receiver built on kepyur.
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "combine.h"
#include "decode_queue.h"
#include "fallback_policy.h"
#include "frame_gate.h"
#include "pilots.h"
#include "scan.h"
#include "spectrum.h"
#include "steps.h"
#include "sweep.h"
#include "video.h"

namespace wsdr {
class Receiver;
struct RxStatus;
struct BankMeta;
}

namespace kp {

class Radio {
public:
  // frames: the watched channel's pictures, on the decode thread, past the FrameGate; in strips while
  // PictureOpts::strips is on (low latency: Frame::y0, y1). Their LineTimes say when the adapter took them.
  explicit Radio(FrameSink frames);
  ~Radio();
  Radio(const Radio &) = delete;
  Radio &operator=(const Radio &) = delete;

  // fd >= 0: the adapter behind this usbfs fd (Android); fd < 0: the first 8812BU on the bus (desktop).
  // Blocking (~5 s chip bring-up). Throws std::exception with a user-presentable message.
  void open(int fd);
  void close();
  bool is_open() const;
  bool superspeed() const;
  // The DSP thread has stopped because the receiver did (the adapter was lost, or its stream kept failing): the
  // Radio still reads as open, but nothing more will come until open() again (which clears this).
  bool dead() const;

  // Then watch prefer_hz if this scan finds it (or, with nothing found, anyway), else the strongest this scan
  // finds; pilots from earlier scans do not count. prefer_hz 0: the pilot being watched.
  void scan(double prefer_hz = 0);
  void watch(double hz);
  void stream_test(int rate_sel, double seconds);
  // Sweep lo..hi at 40 MS/s until the next mode call (sweep.h: a retune every 30 MHz), each step's spectrum into the
  // band picture (spectrum_json). One pass over 300 MHz is 11 or 12 retunes. Scans add their steps to it too. Each
  // sweep step's capture also says what it recognises on the air (sweep.h: StepAnalysers), such as which signals are
  // DJI, HDZero or another digital system (digital.h) and where Wi-Fi networks are (wifi.h); scans do not judge that.
  void sweep(double lo_hz, double hi_hz);
  void idle();
  // The three setters below only take effect at the next watch() (the next tune). Each returns true when it
  // changed something, so that a watch already running has to be started again (watch()) to have it.
  //
  // The rate watch uses (1 = 40 MS/s, the default; 2 = 20 MS/s). A change also clears a 20 MS/s fallback, even
  // one that holds over reopens (FallbackPolicy::watch_rate_set); the same rate again changes nothing.
  bool set_watch_rate(int rate_sel);
  // Gain: < 0 = the chip's AGC, else fixed gain in dB (CCA off, IGI pinned).
  bool set_gain(double db);
  // Off-centre tuning (watch only): the hardware centre on the 5 MHz grid ~6 MHz to one side of the pilot at
  // 40 MS/s, so the video never sits on the chip's DC point: its DC filter eats whatever brightness lands there
  // and rings when the picture moves off it (stripes at the left edge, ghost outlines). At 20 MS/s it changes
  // nothing: the stream is only +-10 MHz wide, the video has to stay in its middle, and the centre stays the
  // grid point nearest the pilot (at most 2.5 MHz from it). Off by default.
  bool set_offset_tuning(bool on);
  // The hardware centre off-centre tuning picks for a pilot at hz, streaming at fs.
  static double offset_centre(double hz, double fs);
  // Two-antenna video: true = paths A and B combined before demodulation (combine.h), false = A only. Takes
  // effect at once (no retune).
  void set_combine(bool on);
  // The decoder's picture switches (video.h), for the watch that is running and those after it. Takes effect
  // within a line of video (no retune). Any thread.
  void set_picture(const PictureOpts &p) { _picture.store(p, std::memory_order_relaxed); }
  struct ComboStatus {
    bool on = false;
    Combiner::Stats s;  // the last second; the time offset between the paths as of its end
  };
  ComboStatus combine_status();
  // Debug switches (the app's "env" extra: KP_CONCEAL, KP_FAST_CPUS, KP_LEVELS, KP_FLYWHEEL, KP_HOLD, KP_FILTER;
  // kp_live reads the same variables once, before open()). All on by default; any may be called from any thread,
  // open or not.
  // Torn frames hidden (FrameGate); off: every frame is shown. Takes effect at once.
  void set_conceal(bool on);
  // The DSP and decode threads kept off the slowest cores (thread_placement.h); off: left to the OS. Takes
  // effect when those threads next start: at the next open().
  void set_fast_cores(bool on);
  // The decoder's helps, to see each by itself (KP_LEVELS, KP_FLYWHEEL, KP_HOLD, KP_FILTER;
  // DecoderOpts::follow_levels, flywheel, hold, channel_filter). Each takes effect with the next decoder: at the
  // next carrier or rate watched.
  void set_follow_levels(bool on) { _follow_levels.store(on, std::memory_order_relaxed); }
  void set_flywheel(bool on) { _flywheel.store(on, std::memory_order_relaxed); }
  void set_hold(bool on) { _hold.store(on, std::memory_order_relaxed); }
  void set_channel_filter(bool on) { _channel_filter.store(on, std::memory_order_relaxed); }
  // Debug: save the next `seconds` of watched IQ as PREFIX.cs16 + PREFIX.json (kp_replay reads them), and path B
  // as PREFIX_b.cs16.
  void record(double seconds, const std::string &prefix);

  std::string status_json();   // for the UI (polled); its "pilots" are the scan under way's, or the last one's
  // The band picture over lo..hi, for the spectrum page (polled while it is shown):
  //   {"first_mhz", "bin_mhz", "db": [dB over the noise floor, null where nothing has been seen],
  //    "carriers": [{"mhz", "width", "snr", "kind"}], "at_mhz": the step being taken now (0: not sweeping or scanning),
  //    "at_width_mhz", "sweeps": passes finished since sweep() (0 while the first is under way),
  //    "signals": [{"lo_mhz", "hi_mhz", "kind", "label", "active"}]}
  // "signals": what sweeps have recognised on the air reaching into lo..hi, each a span of the band, by its "kind":
  //   "dji", "hdzero", "hdzero_narrow" or "digital": digital video (digital.h), the edges of its block as measured.
  //   "label" is the system ("DJI", "HDZero", "HDZero narrow", "Digital"), "active" true (a video link sends all the
  //   time it is on). Kept until the sweeps over it have not found it for 8 s, and after the sweep stops.
  //   "wifi": a Wi-Fi network (wifi.h), the 20, 40, 80 or 160 MHz of its channels; "label" "Wi-Fi 149" or
  //   "Wi-Fi 100-112" (an en dash), "active" once seen in use (false: only its beacons seen). Kept 5 minutes from the
  //   last time a sweep saw it (a quiet one is caught about one pass in eight), and after the sweep stops.
  // Other kinds may join the list (a reader skips one it does not know): "active" says in use, false on the air but
  // idle. A carrier on a signal has that signal's "kind"; one with none is not recognised (analog, or not judged).
  std::string spectrum_json(double lo_hz, double hi_hz);

private:
  enum class Mode { Idle, Scan, Watch, Test, Sweep };
  struct Bank {
    std::vector<uint8_t> data;
    uint64_t seq;
    uint32_t gen;
    double fs, centre_hz, shift_hz;
    double t_sampled;  // when its last sample was taken (steady clock, s): it came in, less the adapter's shipping
  };
  enum Path { kPathA = 0, kPathB = 1 };
  void dsp_loop();
  void decode_loop();
  void on_bank(const uint8_t *b, size_t n, const wsdr::BankMeta &m);
  // watching: off-centre tuning applies (the hardware centre recomputed for this rate)
  void tune(double hz, int rate_sel, bool watching = false);
  void begin_watch();
  void begin_scan();
  void begin_sweep();
  void begin_test();
  // a watched bank, step by step (handle_watch)
  void handle_watch(Bank &b);
  void new_front_end(const Bank &b);
  bool stream_restarted(const Bank &b, int64_t t_ms);
  bool blocks_lost(const Bank &b, int64_t t_ms);
  void fall_back(double hz);
  const std::vector<cf> &path_b(const Bank &b);
  void measure_snr(const std::vector<cf> &iq, double fs, double shift_hz);
  void record_bank(const Bank &b);
  bool begin_recording(const Bank &b);
  void end_recording();
  void combine_antennas(const Bank &b, bool gap, int64_t t_ms);
  void demod_for_decoder(bool gap, double t_sampled);

  // the steps of a scan or a sweep (_steps), each a retune
  void start_steps(std::vector<double> plan);
  bool next_step();
  bool capture_step(const Bank &b);
  void handle_scan(Bank &b);
  void add_extra_steps(const std::vector<ScanPlan::Extra> &extra);
  void handle_sweep(Bank &b);
  void handle_test(Bank &b);
  void scan_step_done();
  void add_spectrum(const std::vector<float> &db, double centre_hz);
  static void unpack(const Bank &b, Path path, std::vector<cf> &out);
  void note_faults(const wsdr::RxStatus &st);  // log each stream restart the receiver has made since the last call

  std::unique_ptr<wsdr::Receiver> _rx;
  std::mutex _rxm;  // _rx itself (open and close replace it; status_json reads it from another thread)
  mutable std::mutex _m;  // mode, pilots, status fields below
  Mode _mode = Mode::Idle;
  uint64_t _mode_epoch = 0;  // bumped on every mode change; the DSP thread restarts its state
  double _watch_hz = 0, _req_hz = 0;
  int _req_rate = 1;
  int _watch_rate = 1;
  // 40 MS/s giving way to 20 MS/s in a watch. One for the process, like this Radio: open() clears a fallback, but
  // not its "no more falling back", nor a fallback because 40 MS/s never streamed. Locks for itself (taken inside
  // _m where both are needed, never the reverse).
  FallbackPolicy _fb;
  double _gain_db = -1;
  std::atomic<PictureOpts> _picture{PictureOpts{}};  // set_picture(); the DSP thread hands it to the decoder
  bool _offset = false;      // off-centre tuning
  bool _dead = false;        // the DSP thread exited because the receiver stopped (cleared by open())
  bool _wedged = false;      // ... because the chip's C2H path wedged: only open() again recovers (RxGaveUp::Wedged)
  // recording (DSP thread writes; the request is under _m)
  double _rec_req_s = 0;
  std::string _rec_req_prefix;
  FILE *_rec = nullptr, *_rec_b = nullptr;  // path A (PREFIX.cs16) and B (PREFIX_b.cs16)
  std::string _rec_prefix;
  uint64_t _rec_left = 0, _rec_samples = 0;
  double _rec_fs = 0, _rec_centre = 0, _rec_shift = 0;
  Pilots _pilots;  // what the scan under way, or the last one, found (begin_scan empties it)
  BandPicture _band;  // what scans and sweeps have seen, and what sweeps have recognised in it (sweep.h)
  Sweep _sweep;       // the span and the passes of the sweep asked for last
  bool _superspeed = false, _open = false;
  std::string _chip;

  // bank queue
  std::mutex _qm;
  std::condition_variable _qcv;
  std::deque<Bank> _q;
  std::vector<std::vector<uint8_t>> _pool;
  size_t _qcap = 512;  // 100 ms at 40 MS/s
  std::atomic<uint64_t> _banks{0}, _dropped{0};

  std::thread _dsp;
  std::atomic<bool> _stop{false};
  std::atomic<bool> _started{false};  // open() has the receiver started: only then can it be found stopped

  std::atomic<bool> _fast_cores{true};  // set_fast_cores()
  // set_follow_levels(), set_flywheel(), set_hold(), set_channel_filter()
  std::atomic<bool> _follow_levels{true}, _flywheel{true}, _hold{true}, _channel_filter{true};

  // decode thread: watch mode runs demod on the DSP thread and the video decoder here (two cores)
  FrameGate _gate;  // between the decoders and the frame sink: the frames torn by a hole in the stream stop here
  std::thread _decoder;
  DecodeQueue _decq;

  double _dload = 0;

  // DSP-thread state
  uint64_t _epoch_seen = ~0ull;
  uint64_t _faults_seen = 0;  // the receiver's restart count as last logged
  std::shared_ptr<FrontEnd> _fe;
  uint32_t _fe_gen = ~0u;
  uint64_t _watch_seq = 0;  // the last bank this watch took (with _fe_gen: does the next one follow on from it?)
  // live SNR of the watched carrier: a spectrum of kPsdSpan of IQ every kSnrEveryMs
  std::unique_ptr<Psd> _snr_psd;
  size_t _snr_have = 0, _snr_need = 0;
  int64_t _snr_t_ms = 0;
  float _snr_db = -1;  // < 0: none yet (under _m)
  double _fe_fs = 0, _fe_shift = 0;
  std::vector<cf> _iq, _iqb;  // the bank in hand: path A, and path B once path_b() was asked for it
  bool _iqb_fresh = false;    // _iqb is this bank's
  // two-antenna video
  std::atomic<bool> _combine{false};
  bool _combine_was = false;  // what the last bank did (the combiner restarts on a change)
  Combiner _combiner;
  int64_t _comb_t0_ms = 0;
  ComboStatus _comb_status;  // under _m
  // scan, and sweep
  Steps _steps;  // the retunes of the scan or of the sweep's pass, and the capture of the one under way
  ScanPlan _scan;  // the scan's plan, and its extra steps for signals its steps see cut by an edge
  // The scan's progress as status_json() shows it (steps done, of how many: the plan's and the extra steps given so
  // far, so the count can grow while the scan runs). Atomic: read there while the DSP thread moves it on, and set
  // back to the start by scan() for the status to show until that thread takes the new scan up.
  std::atomic<size_t> _step{0}, _plan_n{0};
  std::vector<Hump> _humps;  // this step's candidates (from the first kPsdSpan of the capture)
  StepAnalysers _analysers;  // what each sweep step's capture is measured for (sweep.h)
  double _resume_hz = 0;
  // test
  uint64_t _test_banks = 0, _test_gaps = 0, _test_last_seq = 0, _test_runs = 0;
  uint32_t _test_gen = 0;
  int64_t _test_t0_ms = 0;
  std::atomic<int64_t> _test_end_ms{0};  // set by stream_test() on the caller's thread
  double _test_fs = 0;
  std::string _test_result;
  // load
  double _busy_s = 0, _load = 0;
  int64_t _load_t0_ms = 0;
  uint64_t _status_ticks = 0;
  // video stats for status
  DecoderStats _vstats;
  uint64_t _frames_at_last = 0;
  int64_t _fps_t0_ms = 0;
  float _fps = 0;
};

int64_t now_ms();

}  // namespace kp
