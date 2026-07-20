#pragma once

#include <array>
#include <cstdint>

#include "core/Types.hpp"

namespace mm {

enum class ProfileUpdateResult : uint8_t {
  Created,
  Replaced,
  AlreadyExists,
  NotFound,
  Full,
};

inline ProfileUpdateResult applyProfileUpdate(
    std::array<VelocityProfileConfiguration, kMaximumProfiles>& profiles,
    uint8_t& profile_count, const VelocityProfileConfiguration& update,
    const bool create_only) {
  for (uint8_t index = 0U; index < profile_count; ++index) {
    if (profiles[index].profile_id != update.profile_id) {
      continue;
    }
    if (create_only) {
      return ProfileUpdateResult::AlreadyExists;
    }
    profiles[index] = update;
    return ProfileUpdateResult::Replaced;
  }
  if (!create_only) {
    return ProfileUpdateResult::NotFound;
  }
  if (profile_count >= kMaximumProfiles) {
    return ProfileUpdateResult::Full;
  }
  profiles[profile_count++] = update;
  return ProfileUpdateResult::Created;
}

}  // namespace mm
