#pragma once

#include <cstdint>

#include "core/Types.hpp"

namespace mm {

class VelocityProfile {
 public:
  void select(const VelocityProfileConfiguration* configuration, uint64_t start_us);
  void stop();
  float target(uint64_t timestamp_us) const;
  bool active() const { return configuration_ != nullptr; }
  bool finished(uint64_t timestamp_us) const;
  uint16_t id() const { return configuration_ == nullptr ? 0U : configuration_->profile_id; }

 private:
  const VelocityProfileConfiguration* configuration_ = nullptr;
  uint64_t start_us_ = 0;
};

}  // namespace mm
