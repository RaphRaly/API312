#pragma once

#include "voltage_source.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class SineVoltageSource : public VoltageSource {
public:
  SineVoltageSource(const std::string &name, NodeIndex p, NodeIndex n,
                    double freq, double amp, double phase = 0.0)
      : VoltageSource(name, p, n, 0.0), m_freq(freq), m_amp(amp),
        m_phase(phase) {}

  void setTime(double t) {
    double val = m_amp * std::sin(2.0 * M_PI * m_freq * t + m_phase);
    setVoltage(val);
  }

private:
  double m_freq, m_amp, m_phase;
};
