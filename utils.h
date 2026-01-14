#pragma once
#include <algorithm>
#include <cmath>

/**
 * PN Junction Limiting (SPICE-like)
 * Limits the voltage step on a PN junction to prevent exponential overflow.
 *
 * vNew: candidate voltage across junction
 * vOld: voltage from previous iteration
 * vt: thermal voltage (n*Vt)
 * vcrit: critical voltage
 * maxStep: optional hard delta clamp (default 0.25V)
 */
inline double pnjlim(double vNew, double vOld, double vt, double vcrit,
                     double maxStep = 0.2) {
  double vResult = vNew;

  // 1. SPICE-style Logarithmic Growth Limit for Forward Bias
  if (vNew > vcrit && vNew > vOld) {
    double arg = (vNew - vOld) / vt;
    vResult = vOld + vt * std::log(1.0 + arg);
  }

  // 2. Hard Delta Clamp (Safety Net)
  if (vResult > vOld + maxStep)
    vResult = vOld + maxStep;
  if (vResult < vOld - maxStep)
    vResult = vOld - maxStep;

  return vResult;
}
