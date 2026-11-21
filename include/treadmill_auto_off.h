#pragma once

#include <cstdint>

// Low HR and missing HR both mean no cooling demand. A valid reading at or
// above the threshold cancels the countdown and allows cooling to resume.
class TreadmillAutoOff {
 public:
  bool update(bool enabled, bool freshHr, int hr, int stopBelow,
              uint32_t now, uint32_t duration) {
    if (!enabled || duration == 0 || (freshHr && hr >= stopBelow)) {
      reset();
      return false;
    }
    if (!pending) {
      since = now;
      pending = true;
    }
    return now - since >= duration;
  }

  bool waiting() const { return pending; }
  void reset() { pending = false; }

 private:
  uint32_t since = 0;
  bool pending = false;
};
