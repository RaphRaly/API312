#pragma once
#include "linear_system.h"
#include <vector>

// Forward declaration
struct StampContext;

// Base interface for linear stamping elements (R, independent sources, etc.)
class IElement {
public:
  virtual ~IElement() = default;
  virtual void stamp(StampContext &ctx) const = 0;

  // Topological Audit: Return all nodes that are DC-coupled via this element.
  // Connections are pairs of (nodeA, nodeB).
  virtual void
  getDcConnections(std::vector<std::pair<int, int>> &connections) const = 0;
};

// Context for limiting operations
struct LimitContext {
  const std::vector<double> &x;    // Current Newton guess
  const std::vector<double> &xOld; // Previous converged solution (or initial)
};

// Internal limited state for nonlinear elements
struct LimitedState {
  double vbe_lim = 0.0; // Limited Vbe for BJTs/diodes
  double vbc_lim = 0.0; // Limited Vbc for BJTs
  // Add other limited voltages as needed
};

// Interface for Newton-linearized nonlinear elements (diodes, BJTs, saturating
// Lm, etc.)
class INewtonElement {
public:
  virtual ~INewtonElement() = default;

  // Stamp linearized model using internal limited state
  virtual void stampNewton(StampContext &ctx,
                           const std::vector<double> &xGuess) const = 0;

  // Compute and store limited voltages from current guess (doesn't modify x)
  // This is called BEFORE stampNewton to prepare internal limited state
  virtual void computeLimitedVoltages(const LimitContext &ctx) = 0;

  // Update internal state after Newton iteration (e.g. for next limiting
  // reference) Called after each Newton step to update history/limiting state
  virtual void limitUpdate(const std::vector<double> &xNew) = 0;
};

// Interface for time-discrete companion-model elements (capacitors, inductors,
// etc.)
class IDynamicElement {
public:
  virtual ~IDynamicElement() = default;

  // Called once per time step before Newton iterations to compute companion
  // parameters.
  virtual void beginStep(double dt) = 0;

  // Called once per time step after convergence to update history state.
  virtual void commitStep(const std::vector<double> &xSolved) = 0;
};

// Some elements need an extra MNA branch variable (voltage sources, inductors,
// etc.)
class IBranchElement {
public:
  virtual ~IBranchElement() = default;
  virtual int branchCount() const = 0;
  virtual void setBranchIndex(int firstBranchIndex) = 0;
};

// Context passed during stamping
struct StampContext {
  DenseLinearSystem &sys;
  double scale = 1.0;
};
