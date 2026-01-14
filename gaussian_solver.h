#pragma once

#include "linear_system.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

// Simple dense Gaussian elimination with partial pivoting.
// Good for correctness; later we can replace by sparse LU.
class GaussianSolver {
public:
  // Solve using DenseLinearSystem
  // Returns -1 on success, or the 0-indexed row of pivot failure
  static int solve(const DenseLinearSystem &sys, std::vector<double> &xOut) {
    return solve(sys.mat(), sys.size(), sys.rhs(), xOut);
  }

  // Solve using raw vectors (needed by linear_solver.h)
  // Returns -1 on success, or row index on failure.
  static int solve(const std::vector<double> &A_in, int N,
                   const std::vector<double> &b_in, std::vector<double> &xOut) {
    if (N <= 0)
      return -1;

    // Copy A and z because we'll modify them during elimination.
    std::vector<double> A = A_in;
    std::vector<double> b = b_in;

    if ((int)xOut.size() != N) {
      xOut.assign((std::size_t)N, 0.0);
    }

    // Forward elimination
    for (int k = 0; k < N; ++k) {
      // Pivot
      int pivot = k;
      double maxAbs =
          std::abs(A[(std::size_t)k * (std::size_t)N + (std::size_t)k]);

      for (int i = k + 1; i < N; ++i) {
        const double v =
            std::abs(A[(std::size_t)i * (std::size_t)N + (std::size_t)k]);
        if (v > maxAbs) {
          maxAbs = v;
          pivot = i;
        }
      }

      if (maxAbs < 1e-18) { // Relaxed threshold for singularity
        return k;           // Return the row index of failure
      }

      // Swap rows if needed
      if (pivot != k) {
        for (int j = k; j < N; ++j) {
          std::swap(A[(std::size_t)k * (std::size_t)N + (std::size_t)j],
                    A[(std::size_t)pivot * (std::size_t)N + (std::size_t)j]);
        }
        std::swap(b[(std::size_t)k], b[(std::size_t)pivot]);
      }

      const double Akk = A[(std::size_t)k * (std::size_t)N + (std::size_t)k];

      // Eliminate
      for (int i = k + 1; i < N; ++i) {
        const double Aik = A[(std::size_t)i * (std::size_t)N + (std::size_t)k];
        const double factor = Aik / Akk;

        if (factor == 0.0)
          continue;

        A[(std::size_t)i * (std::size_t)N + (std::size_t)k] = 0.0;
        for (int j = k + 1; j < N; ++j) {
          A[(std::size_t)i * (std::size_t)N + (std::size_t)j] -=
              factor * A[(std::size_t)k * (std::size_t)N + (std::size_t)j];
        }

        b[(std::size_t)i] -= factor * b[(std::size_t)k];
      }
    }

    // Back substitution
    for (int i = N - 1; i >= 0; --i) {
      double sum = b[(std::size_t)i];
      for (int j = i + 1; j < N; ++j) {
        sum -= A[(std::size_t)i * (std::size_t)N + (std::size_t)j] *
               xOut[(std::size_t)j];
      }

      const double Aii = A[(std::size_t)i * (std::size_t)N + (std::size_t)i];
      xOut[(std::size_t)i] = sum / Aii;
    }

    return -1; // SUCCESS
  }
};
