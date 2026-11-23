// fpvDROID decoder: second-order filter sections (RBJ designs), one at a time (Biquad) and several side by side in
// the lanes of a vector (BiquadN).
#pragma once
#include <cmath>

namespace kp {

struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  void design(double fc, double fs, double q = 0.7071) {  // low-pass
    const double w = 2 * M_PI * fc / fs, c = std::cos(w), s = std::sin(w), al = s / (2 * q);
    const double a0 = 1 + al;
    b0 = static_cast<float>((1 - c) / 2 / a0);
    b1 = static_cast<float>((1 - c) / a0);
    b2 = b0;
    a1 = static_cast<float>(-2 * c / a0);
    a2 = static_cast<float>((1 - al) / a0);
  }
  void bandpass(double fc, double fs, double q) {  // 0 dB at fc
    const double w = 2 * M_PI * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
    const double a0 = 1 + al;
    b0 = static_cast<float>(al / a0);
    b1 = 0;
    b2 = static_cast<float>(-al / a0);
    a1 = static_cast<float>(-2 * c / a0);
    a2 = static_cast<float>((1 - al) / a0);
  }
  void notch(double fc, double fs, double q) {
    const double w = 2 * M_PI * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
    const double a0 = 1 + al;
    b0 = static_cast<float>(1 / a0);
    b1 = static_cast<float>(-2 * c / a0);
    b2 = b0;
    a1 = b1;
    a2 = static_cast<float>((1 - al) / a0);
  }
  inline float run(float x) {
    const float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  // For a filter that is being fed zeros: once its state has died away to nothing (far below anything that could
  // reach a pixel), make it exactly zero, before it gets down to denormal numbers.
  void settle() {
    if (std::fabs(z1) < 1e-25f && std::fabs(z2) < 1e-25f) z1 = z2 = 0;
  }
};

// Biquads side by side, one per lane of a vector of 4 (or 2) floats (NEON on a phone, SSE on a PC): each lane
// goes through exactly Biquad::run's arithmetic, all of them in one step. For filters that run on every sample:
// a step of each on its own is most of their cost.
typedef float v4f __attribute__((vector_size(16)));
typedef float v2f __attribute__((vector_size(8)));
template <typename V> struct BiquadN {
  V b0{}, b1{}, b2{}, a1{}, a2{}, z1{}, z2{};
  void load(int i, const Biquad &q) {
    b0[i] = q.b0;
    b1[i] = q.b1;
    b2[i] = q.b2;
    a1[i] = q.a1;
    a2[i] = q.a2;
    z1[i] = q.z1;
    z2[i] = q.z2;
  }
  void store(int i, Biquad &q) const {
    q.z1 = z1[i];
    q.z2 = z2[i];
  }
  inline V run(V x) {
    const V y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void settle(int i) {  // Biquad::settle, for one lane
    if (std::fabs(z1[i]) < 1e-25f && std::fabs(z2[i]) < 1e-25f) z1[i] = z2[i] = 0;
  }
};
using Biquad4 = BiquadN<v4f>;
using Biquad2 = BiquadN<v2f>;

}  // namespace kp
