#include <cassert>
#include "cooling_math.h"
#include "zero_power.h"
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
