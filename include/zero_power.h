#pragma once

#include <cstdint>

class PowerSample {
 public:
  void record(int watts, uint32_t now, uint32_t maxGap) {
    if (watts != 0 || this->watts != 0 || !fresh(now, maxGap)) {
      zeroSince = now;
    }
    this->watts = watts;
    timestamp = now;
    valid = true;
  }

  bool fresh(uint32_t now, uint32_t maxGap) const {
    return valid && now - timestamp < maxGap;
  }

  bool zeroFor(uint32_t now, uint32_t duration, uint32_t maxGap) const {
    return watts == 0 && fresh(now, maxGap) && timestamp - zeroSince >= duration;
  }

  int watts = 0;
  uint32_t timestamp = 0;

 private:
  uint32_t zeroSince = 0;
  bool valid = false;
};
