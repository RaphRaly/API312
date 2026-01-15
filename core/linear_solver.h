#pragma once
#include "gaussian_solver.h"
#include "linear_system.h"
#include <vector>

// Abstract interface for linear system solvers
// Allows switching between dense and sparse solvers
class ILinearSolver {
public:
  virtual ~ILinearSolver() = default;

  // Solve A * x = b, return -1 if success, or index of zero pivot failure
  virtual int solve(const std::vector<double> &A_flat, int N,
                    const std::vector<double> &b, std::vector<double> &x) = 0;

  // Get solver name for diagnostics
  virtual const char *name() const = 0;
};

// Dense Gaussian elimination (current implementation)
class DenseGaussSolver : public ILinearSolver {
public:
  int solve(const std::vector<double> &A_flat, int N,
            const std::vector<double> &b, std::vector<double> &x) override;

  const char *name() const override { return "DenseGauss"; }
};

// Future: Sparse LU solver (Eigen-based)
// class SparseLUSolver : public ILinearSolver { ... };

// DenseGaussSolver implementation
int DenseGaussSolver::solve(const std::vector<double> &A_flat, int N,
                            const std::vector<double> &b,
                            std::vector<double> &x) {
  return GaussianSolver::solve(A_flat, N, b, x);
}

// Factory function to create solver
inline std::unique_ptr<ILinearSolver> createLinearSolver() {
  return std::make_unique<DenseGaussSolver>();
}
