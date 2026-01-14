#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

// Dense linear system: A * x = z
// For real-time performance, we will later swap this for a sparse solver,
// but dense is perfect to validate correctness first.
class DenseLinearSystem {
public:
  DenseLinearSystem() = default;

  void resize(int n) {
    N = n;
    A.assign((std::size_t)N * (std::size_t)N, 0.0);
    z.assign((std::size_t)N, 0.0);
  }

  int size() const { return N; }

  void clear() {
    std::fill(A.begin(), A.end(), 0.0);
    std::fill(z.begin(), z.end(), 0.0);
  }

  // Adds value to matrix entry A(r,c)
  void addA(int r, int c, double value) {
    A[(std::size_t)r * (std::size_t)N + (std::size_t)c] += value;
  }

  // Adds value to RHS z(r)
  void addZ(int r, double value) { z[(std::size_t)r] += value; }

  double getA(int r, int c) const {
    return A[(std::size_t)r * (std::size_t)N + (std::size_t)c];
  }

  double getZ(int r) const { return z[(std::size_t)r]; }

  const std::vector<double> &mat() const { return A; }
  const std::vector<double> &rhs() const { return z; }

private:
  int N = 0;
  std::vector<double> A;
  std::vector<double> z;
};
