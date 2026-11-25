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
#include <vector>

namespace kp {

// Over one window (LatencyMeter::take): medians, ms; 0 where there were none.
struct LatencyStats {
  float seconds = 0;              // the window
  int presents = 0, shown = 0;    // presents, and of them those the display said it showed
  int dropped = 0, unknown = 0;   // ... replaced before they were shown; the display could not say
  float queued_top = 0, queued_bottom = 0;  // a picture's top / bottom line taken -> the present with it handed over
  float shown_top = 0, shown_bottom = 0;    // ... -> on the display (the present, or the first after it shown)
  float shown_top_p95 = 0;                  // the slowest twentieth of the top lines
  // ... -> on the display as the compositor expects it (its next composition after the present, plus its own delay
  // from composition to the display): there for every present, where the display's own times are not (too many
  // presents a second for its history of them, with strips)
  float expected_top = 0, expected_bottom = 0;
  // where the time goes: the newest line in a strip or frame taken -> it handed to the screen (radio and decoder);
  // the first handed to the screen since the last present -> the next present handed over (the render thread)
  float to_screen = 0, render = 0;
  float swap = 0;                 // ... of which in the call that hands a present over (eglSwapBuffers)
  float refresh_hz = 0;           // the display's, as the compositor last said (0: not known; set by the caller)
};

class LatencyMeter {
public:
  // What a query of a present's display time (poll) gives besides a time.
  static constexpr int64_t kPending = -2;  // not on the display yet
  static constexpr int64_t kDropped = -1;  // never will be: a later present took its place first
  static constexpr int64_t kUnknown = -3;  // the display no longer says (or never did)
  static constexpr double kGiveUpS = 0.5;  // a present still pending this long after it was handed over is given up

  // A present handed to the compositor at t (steady clock, s). top_t, bottom_t: when the top / bottom line of a
  // picture in it for the first time was taken (0: none in it). id: the present's number for poll(), 0: none.
  // expected: when the compositor expects to show it (0: not known). posted: when the first of what it shows was
  // handed to the screen (0: not known).
  // swap_ms: how long the call that handed it over took.
  void presented(double t, double top_t, double bottom_t, uint64_t id, double expected = 0, double posted = 0,
                 float swap_ms = 0);
  // The ages (ms) of what was handed to the screen since the last call: the newest line in each taken -> handed over.
  void handed(const std::vector<float> &ages_ms);
  // When the presents still waiting went up: query(id) gives the time (ns, steady clock) or one of the codes above.
  // In order, as far as the first one still pending. A dropped present's lines went up with the first one after it
  // that was shown.
  void poll(double now, const std::function<int64_t(uint64_t)> &query);
  // The window since the last take() (or the first present), and a new one from now.
  LatencyStats take(double now);
  // Forget the presents waiting for a display time (a new window surface).
  void forget();

private:
  struct Present {
    uint64_t id;
    double t, top_t, bottom_t;
  };
  static float median(std::vector<float> &v, double q = 0.5);

  std::deque<Present> _wait;
  double _carry_top = 0, _carry_bottom = 0;  // lines of dropped presents, still to go up
  std::vector<float> _q_top, _q_bottom, _s_top, _s_bottom, _e_top, _e_bottom, _to_screen, _render, _swap;
  int _presents = 0, _shown = 0, _dropped = 0, _unknown = 0;
  double _t0 = 0;
};

}  // namespace kp
