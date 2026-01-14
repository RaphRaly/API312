#pragma once

#include "circuit.h"
#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class ValidationHarness {
public:
  static void exportToCsv(const std::string &filename,
                          const std::vector<double> &time,
                          const std::vector<double> &signal) {
    std::ofstream fs(filename);
    fs << "time,value\n";
    for (size_t i = 0; i < time.size(); ++i) {
      fs << time[i] << "," << signal[i] << "\n";
    }
  }

  // Simple Discrete Fourier Transform for THD calculation
  // returns (magnitude, phase) for a given frequency
  static double computeMagnitudeAtFreq(const std::vector<double> &signal,
                                       double dt, double targetFreq) {
    std::complex<double> sum(0, 0);
    double T = signal.size() * dt;
    for (size_t n = 0; n < signal.size(); ++n) {
      double angle = 2.0 * M_PI * targetFreq * n * dt;
      sum +=
          signal[n] * std::complex<double>(std::cos(angle), -std::sin(angle));
    }
    return 2.0 * std::abs(sum) / signal.size();
  }

  static double calculateThd(const std::vector<double> &signal, double dt,
                             double fundamentalFreq) {
    double f1_mag = computeMagnitudeAtFreq(signal, dt, fundamentalFreq);
    if (f1_mag < 1e-9)
      return 0.0;

    double harmonics_sq_sum = 0.0;
    for (int h = 2; h <= 10; ++h) {
      double mag = computeMagnitudeAtFreq(signal, dt, fundamentalFreq * h);
      harmonics_sq_sum += mag * mag;
    }

    return std::sqrt(harmonics_sq_sum) / f1_mag;
  }
};
