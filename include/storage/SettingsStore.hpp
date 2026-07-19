#pragma once

#include "core/Types.hpp"

namespace mm {

class SettingsStore {
 public:
  bool load(MachineSettings& settings);
  bool save(const MachineSettings& settings);
  bool erase();
  static MachineSettings defaults();
  static bool validate(const MachineSettings& settings);
};

}  // namespace mm

