#pragma once
// Platform-independent cooling calculation helpers. No Arduino dependencies.
namespace CoolingMath {
// Integer ceiling avoids setting the threshold 1 W/bpm too early.
inline int ceilPercent(int span, int pct) {
  return (span * pct + 99) / 100;
}
inline int hrAtPercent(int restingHr, int maxHr, int pctHRR) {
  return restingHr + ceilPercent(maxHr - restingHr, pctHRR);
}
inline int powerAtPercent(int ftp, int pctFTP) {
  return ceilPercent(ftp, pctFTP);
}
inline int stage(float value, int t2, int t3, int hysteresis, int previous) {
  if (value >= t3 || (previous == 3 && value >= t3 - hysteresis)) return 3;
  if (value >= t2 || (previous >= 2 && value >= t2 - hysteresis)) return 2;
  return 1;
}
}  // namespace CoolingMath
