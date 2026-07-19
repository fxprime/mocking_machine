#pragma once

#include <cmath>

namespace mm {

class LowPassFilter {
 public:
  void configure(const float cutoff_hz) {
    cutoff_hz_ = cutoff_hz;
    reset();
  }

  float update(const float input, const float dt_s) {
    if (!initialized_ || cutoff_hz_ <= 0.0F || dt_s <= 0.0F) {
      output_ = input;
      initialized_ = true;
      return output_;
    }
    constexpr float kTwoPi = 6.2831853071795864769F;
    const float alpha = 1.0F - std::exp(-kTwoPi * cutoff_hz_ * dt_s);
    output_ += alpha * (input - output_);
    return output_;
  }

  void reset() {
    output_ = 0.0F;
    initialized_ = false;
  }

  float output() const { return output_; }

 private:
  float cutoff_hz_ = 20.0F;
  float output_ = 0.0F;
  bool initialized_ = false;
};

}  // namespace mm
