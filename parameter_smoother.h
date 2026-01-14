#pragma once

#include <atomic>
#include <cmath>

/**
 * Lock-free One-Pole Parameter Smoother
 * Essential for VST/AU portage to prevent clicks during real-time parameter
 * changes.
 */
class ParameterSmoother {
public:
  ParameterSmoother(double initialValue = 0.0)
      : m_target(initialValue), m_current(initialValue) {}

  void setTarget(double target) {
    m_target.store(target, std::memory_order_release);
  }

  void setTimeConstant(double tc, double sampleRate) {
    m_coeff = 1.0 - std::exp(-1.0 / (tc * sampleRate));
  }

  double process() {
    double target = m_target.load(std::memory_order_acquire);
    m_current += m_coeff * (target - m_current);
    return m_current;
  }

  double getCurrent() const { return m_current; }

private:
  std::atomic<double> m_target;
  double m_current;
  double m_coeff = 0.1; // Default smoothing
};
