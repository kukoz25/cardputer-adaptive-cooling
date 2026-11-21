#include <cassert>
#include "cooling_math.h"
#include "zero_power.h"
#include "treadmill_auto_off.h"

static void testTreadmillAutoOff() {
  TreadmillAutoOff timer;
  constexpr uint32_t duration = 20000;
  constexpr int threshold = 100;
  assert(!timer.waiting());
  assert(!timer.update(true, true, 140, threshold, 0, duration));
  assert(!timer.waiting());

  // Low HR must last the entire delay, including the exact boundary.
  assert(!timer.update(true, true, 99, threshold, 1000, duration));
  assert(timer.waiting());
  assert(!timer.update(true, true, 90, threshold, 20999, duration));
  assert(timer.update(true, true, 90, threshold, 21000, duration));
  assert(timer.update(true, true, 90, threshold, 22000, duration));

  // At-threshold recovery resumes cooling and cancels a previous pause.
  assert(!timer.update(true, true, 100, threshold, 23000, duration));
  assert(!timer.waiting());

  // No HR counts even if the last remembered value was high.
  assert(!timer.update(true, false, 140, threshold, 24000, duration));
  assert(!timer.update(true, false, 140, threshold, 43999, duration));
  assert(timer.update(true, false, 140, threshold, 44000, duration));
  assert(!timer.update(true, true, 140, threshold, 45000, duration));

  // Recovery before expiry starts a new full countdown on the next drop.
  assert(!timer.update(true, true, 90, threshold, 46000, duration));
  assert(!timer.update(true, true, 120, threshold, 65000, duration));
  assert(!timer.update(true, true, 90, threshold, 66000, duration));
  assert(!timer.update(true, true, 90, threshold, 85999, duration));
  assert(timer.update(true, true, 90, threshold, 86000, duration));

  // Manual mode / cycling disables and resets the timer.
  assert(!timer.update(false, false, -1, threshold, 87000, duration));
  assert(!timer.waiting());
  assert(!timer.update(true, false, -1, threshold, 88000, duration));
  assert(!timer.update(true, false, -1, threshold, 107999, duration));
  assert(timer.update(true, false, -1, threshold, 108000, duration));

  // A zero delay disables auto-off rather than stopping immediately.
  assert(!timer.update(true, true, 90, threshold, 109000, 0));
  assert(!timer.waiting());

  // Low and missing HR share one continuous countdown.
  assert(!timer.update(true, true, 90, threshold, 110000, duration));
  assert(!timer.update(true, false, 90, threshold, 120000, duration));
  assert(timer.update(true, true, 90, threshold, 130000, duration));
  timer.reset();
  assert(!timer.waiting());

  // Unsigned elapsed time remains valid across millis() rollover.
  constexpr uint32_t start = UINT32_MAX - 10000;
  assert(!timer.update(true, false, -1, threshold, start, duration));
  assert(!timer.update(true, false, -1, threshold, start + duration - 1, duration));
  assert(timer.update(true, false, -1, threshold, start + duration, duration));
}
static void testZeroPower() {
  PowerSample power;
  constexpr uint32_t maxGap = 7000;
  constexpr uint32_t duration = 20000;
  assert(!power.zeroFor(20000, duration, maxGap));

  for (uint32_t now = 0; now < duration; now += 1000) {
    power.record(0, now, maxGap);
    assert(!power.zeroFor(now, duration, maxGap));
  }
  // Elapsed wall time cannot substitute for another received zero sample.
  assert(!power.zeroFor(duration, duration, maxGap));
  power.record(0, duration, maxGap);
  assert(power.zeroFor(duration, duration, maxGap));
  assert(!power.zeroFor(duration + maxGap, duration, maxGap));

  power.record(0, duration + maxGap, maxGap);
  assert(!power.zeroFor(duration + maxGap, duration, maxGap));
  power.record(150, 28000, maxGap);
  power.record(0, 29000, maxGap);
  assert(!power.zeroFor(29000, duration, maxGap));
  power.record(-1, 30000, maxGap);
  assert(!power.zeroFor(30000, duration, maxGap));

  power = {};
  constexpr uint32_t start = UINT32_MAX - 10000;
  for (uint32_t elapsed = 0; elapsed <= duration; elapsed += 1000) {
    power.record(0, start + elapsed, maxGap);
  }
  assert(power.zeroFor(start + duration, duration, maxGap));
}

int main() {
  testZeroPower();
  testTreadmillAutoOff();
  assert(CoolingMath::hrAtPercent(49, 199, 54) == 130);
  assert(CoolingMath::hrAtPercent(49, 199, 74) == 160);
  assert(CoolingMath::powerAtPercent(228, 60) == 137);
  assert(CoolingMath::powerAtPercent(228, 90) == 206);
  assert(CoolingMath::stage(129, 130, 160, 8, 1) == 1);
  assert(CoolingMath::stage(145, 130, 160, 8, 1) == 2);
  assert(CoolingMath::stage(164, 130, 160, 8, 1) == 3);
  assert(CoolingMath::stage(154, 130, 160, 8, 3) == 3);
  assert(CoolingMath::stage(150, 130, 160, 8, 3) == 2);
}
