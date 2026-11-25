// fpvDROID screen: how old the picture is when it is shown, from the adapter taking its lines (LineTimes, video.h)
// to the screen: to the present that has them being handed to the compositor (eglSwapBuffers), and, where the
// display says when each present went up (EGL_ANDROID_get_frame_timestamps), to that.
//
// Measured at the two ends of the picture: its top line (row 0, the oldest line of a whole field) and its bottom
// line. With whole fields the top line waits a field for the rest of the picture; with strips (PictureOpts::strips)
// it goes up with the first strip.
#pragma once
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace kp {

// A delay at the two ends of the picture: its top line and its bottom line (ms).
struct Ends {
  float top = 0, bottom = 0;
};

// Over one window (LatencyMeter::take): medians, ms; 0 where there were none.
struct LatencyStats {
  float seconds = 0;             // the window
  int presents = 0, shown = 0;   // presents, and of them those the display said it showed
  int dropped = 0, unknown = 0;  // ... replaced before they were shown; the display could not say
  Ends handed;                   // a picture's top / bottom line taken -> the present with it handed over
  Ends displayed;                // ... -> on the display (the present, or the first after it shown)
  float displayed_top_p95 = 0;   // the slowest twentieth of the top lines
  // ... -> on the display as the compositor expects it (its next composition after the present, plus its own delay
  // from composition to the display): there for every present, where the display's own times are not (too many
  // presents a second for its history of them, with strips)
  Ends expected;
  // where the time goes: the newest line in a frame taken -> it handed to the screen (radio and decoder); the first
  // handed to the screen since the last present -> the next present handed over (the render thread)
  float to_screen = 0, render = 0;
  float swap = 0;                // ... of which in the call that hands a present over (eglSwapBuffers)
  // the display as the window ended (LatencyMeter::take is told)
  float refresh_hz = 0;          // its refresh rate (0: not known)
  bool front = false;            // front-buffer drawing: "handed over" is drawn into the buffer the display shows,
                                 // and "expected" is that plus half a refresh (the display gives no times)
};

// The window as JSON for the app (RadioStatus.kt reads it: PictureDelay), and as a line for the log.
std::string latency_json(const LatencyStats &s);
std::string latency_log(const LatencyStats &s);

class LatencyMeter {
public:
  // What a query of a present's display time (poll) gives besides a time.
  static constexpr int64_t kPending = -2;  // not on the display yet
  static constexpr int64_t kDropped = -1;  // never will be: a later present took its place first
  static constexpr int64_t kUnknown = -3;  // the display no longer says (or never did)
  static constexpr double kGiveUpS = 0.5;  // a present still pending this long after it was handed over is given up

  // A present handed to the compositor. Times on the steady clock, s.
  struct Present {
    double t = 0;                    // when it was handed over
    double top_t = 0, bottom_t = 0;  // when the top / bottom line of a picture in it for the first time was taken
                                     // (0: none in it)
    uint64_t id = 0;                 // its number for poll() (0: none)
    double expected = 0;             // when the compositor expects to show it (0: not known)
    double posted = 0;               // when the first of what it shows was handed to the screen (0: not known)
    float swap_ms = 0;               // how long the call that handed it over took (0: not known)
  };
  void presented(const Present &p);
  // The ages (ms) of what was handed to the screen since the last call: the newest line in each taken -> handed over.
  void handed(const std::vector<float> &ages_ms);
  // When the presents still waiting went up: query(id) gives the time (ns, steady clock) or one of the codes above.
  // In order, as far as the first one still pending. A dropped present's lines went up with the first one after it
  // that was shown.
  void poll(double now, const std::function<int64_t(uint64_t)> &query);
  // The window since the last take() (or the first present), and a new one from now. refresh_hz, front: the display
  // as it is now (LatencyStats).
  LatencyStats take(double now, float refresh_hz = 0, bool front = false);
  // Forget the presents waiting for a display time (a new window surface).
  void forget();

private:
  struct Waiting {
    uint64_t id;
    double t, top_t, bottom_t;
  };
  // the samples of a window, at the two ends of the picture
  struct Samples {
    std::vector<float> top, bottom;
    void add(double from_top, double from_bottom, double to);  // to - from (s) as ms, where from is known (> 0)
    Ends medians();  // ... and the samples gone
  };

  std::deque<Waiting> _wait;
  double _carry_top = 0, _carry_bottom = 0;  // lines of dropped presents, still to go up
  Samples _handed, _displayed, _expected;
  std::vector<float> _to_screen, _render, _swap;
  int _presents = 0, _shown = 0, _dropped = 0, _unknown = 0;
  double _t0 = 0;
};

}  // namespace kp
