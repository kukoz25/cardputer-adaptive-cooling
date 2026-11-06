#include <cassert>
#include "cooling_math.h"
int main() {
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
