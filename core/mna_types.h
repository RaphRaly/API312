#pragma once
#include <cstddef>
#include <vector>

using NodeIndex = int;
inline constexpr NodeIndex GND = -1;

struct ConvergenceStats {
  int totalIterations = 0;
  int sourceStepsReached = 0;
  double lastResidual = 0.0;
  bool converged = false;
};

// Returns node voltage from the solution vector.
// Ground is always 0V and not stored in x.
inline double nodeVoltage(const std::vector<double> &x, NodeIndex n) {
  return (n == GND) ? 0.0 : x[(std::size_t)n];
}
