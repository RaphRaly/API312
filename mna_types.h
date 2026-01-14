#pragma once
#include <vector>
#include <cstddef>

using NodeIndex = int;
static constexpr NodeIndex GND = -1;

// Returns node voltage from the solution vector.
// Ground is always 0V and not stored in x.
inline double nodeVoltage(const std::vector<double>& x, NodeIndex n)
{
    return (n == GND) ? 0.0 : x[(std::size_t)n];
}
