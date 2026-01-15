#pragma once

#include "linear_system.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

using namespace std;

// Simple dense Gaussian elimination with partial pivoting.
// Good for correctness; later we can replace by sparse LU.
class GaussianSolver {
public:
  // Solve using DenseLinearSystem
  // Returns -1 on success, or the 0-indexed row of pivot failure
  static int solve(const DenseLinearSystem &sys, vector<double> &xOut) {
    return solve(sys.mat(), sys.size(), sys.rhs(), xOut);
  }

  // Solve using raw vectors (needed by linear_solver.h)
  // Returns -1 on success, or row index on failure.
  static int solve(const vector<double> &A_in, int N,
                   const vector<double> &b_in, vector<double> &xOut) {
    if (N <= 0)
      return -1;

    // Copy A and z because we'll modify them during elimination.
    vector<double> A = A_in;
    vector<double> b = b_in;

    if ((int)xOut.size() != N) {
      xOut.assign((size_t)N, 0.0);
    }

    // Forward elimination
    for (int k = 0; k < N; ++k) {
      // Pivot
      int pivot = k;
      double maxAbs =
          abs(A[(size_t)k * (size_t)N + (size_t)k]);

      for (int i = k + 1; i < N; ++i) {
        const double v =
            abs(A[(size_t)i * (size_t)N + (size_t)k]);
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
          swap(A[(size_t)k * (size_t)N + (std::size_t)j],
                    A[(size_t)pivot * (size_t)N + (std::size_t)j]);
        }
        swap(b[(size_t)k], b[(size_t)pivot]);
      }

      const double Akk = A[(size_t)k * (size_t)N + (size_t)k];

      // Eliminate
      for (int i = k + 1; i < N; ++i) {
        const double Aik = A[(size_t)i * (size_t)N + (size_t)k];
        const double factor = Aik / Akk;

        if (factor == 0.0)
          continue;

        A[(size_t)i * (size_t)N + (size_t)k] = 0.0;
        for (int j = k + 1; j < N; ++j) {
          A[(size_t)i * (size_t)N + (size_t)j] -=
              factor * A[(size_t)k * (size_t)N + (size_t)j];
        }

        b[(size_t)i] -= factor * b[(size_t)k];
      }
    }

    // Back substitution
    for (int i = N - 1; i >= 0; --i) {
      double sum = b[(size_t)i];
      for (int j = i + 1; j < N; ++j) {
        sum -= A[(size_t)i * (size_t)N + (size_t)j] *
               xOut[(size_t)j];
      }

      const double Aii = A[(size_t)i * (size_t)N + (size_t)i];
      xOut[(size_t)i] = sum / Aii;
    }

    return -1; // SUCCESS
  }
};
