#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace std;

// Dense linear system: A * x = z
// For real-time performance, we will later swap this for a sparse solver,
// but dense is perfect to validate correctness first.
class DenseLinearSystem {
public:
  DenseLinearSystem() = default;

  void resize(int n) {
    if (n == N)
      return;
    N = n;
    A.resize((size_t)N * (size_t)N);
    z.resize((size_t)N);
    clear();
  }

  int size() const { return N; }

  void clear() {
    if (A.empty())
      return;
    fill(A.begin(), A.end(), 0.0);
    fill(z.begin(), z.end(), 0.0);
  }

  // Adds value to matrix entry A(r,c)
  void addA(int r, int c, double value) {
    A[(size_t)r * (size_t)N + (size_t)c] += value;
  }

  // Adds value to RHS z(r)
  void addZ(int r, double value) { z[(size_t)r] += value; }

  double getA(int r, int c) const {
    return A[(size_t)r * (size_t)N + (std::size_t)c];
  }

  double getZ(int r) const { return z[(size_t)r]; }

  const vector<double> &mat() const { return A; }
  const vector<double> &rhs() const { return z; }

private:
  int N = 0;
  vector<double> A;
  vector<double> z;
};
