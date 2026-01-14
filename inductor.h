#pragma once
#include "elements.h"
#include "mna_types.h"
#include "stamping.h"
#include <stdexcept>

// Inductor model with Trapezoidal integration.
// V = L * di/dt
// Discretized: i = iPrev + (dt / (2*L)) * (v + vPrev)
// Rearranging for i = G*v + Ieq:
// i = (dt / (2*L)) * v + (iPrev + (dt / (2*L)) * vPrev)
// G = dt / (2*L)
// Ieq = iPrev + G * vPrev
class Inductor final : public IElement,
                       public IDynamicElement,
                       public IBranchElement {
public:
  Inductor(NodeIndex a, NodeIndex b, double L) : na(a), nb(b), L_(L) {
    if (L_ <= 0.0)
      throw std::invalid_argument("Inductor: L must be > 0");
  }

  Inductor(const std::string &name, NodeIndex a, NodeIndex b, double L)
      : na(a), nb(b), L_(L) {
    if (L_ <= 0.0)
      throw std::invalid_argument("Inductor: L must be > 0");
  }

  void beginStep(double dt) override {
    if (dt <= 0.0)
      throw std::invalid_argument("Inductor: dt must be > 0");

    double stepFactor = 2.0 * L_ / dt; // for Trapezoidal
    R_eff_ = stepFactor;
    rhs_ = -stepFactor * iPrev_ - vPrev_;
  }

  void commitStep(const std::vector<double> &xSolved) override {
    double i_curr = xSolved[branchIdx_];
    double va = nodeVoltage(xSolved, na);
    double vb = nodeVoltage(xSolved, nb);
    double v_curr = va - vb;

    iPrev_ = i_curr;
    vPrev_ = v_curr;
  }

  void stamp(StampContext &ctx) const override {
    if (na != GND)
      ctx.sys.addA(branchIdx_, na, 1.0);
    if (nb != GND)
      ctx.sys.addA(branchIdx_, nb, -1.0);
    ctx.sys.addA(branchIdx_, branchIdx_, -R_eff_);
    ctx.sys.addZ(branchIdx_, rhs_);

    if (na != GND)
      ctx.sys.addA(na, branchIdx_, 1.0);
    if (nb != GND)
      ctx.sys.addA(nb, branchIdx_, -1.0);
  }

  void getDcConnections(
      std::vector<std::pair<int, int>> &connections) const override {
    connections.push_back({na, nb}); // Inductor is a short at DC
  }

  int branchCount() const override { return 1; }
  void setBranchIndex(int firstBranchIndex) override {
    branchIdx_ = firstBranchIndex;
  }

private:
  NodeIndex na, nb;
  double L_;
  int branchIdx_ = -1;

  // Transient stepping fields
  double R_eff_ = 0.0;
  double rhs_ = 0.0;

  // History
  double iPrev_ = 0.0;
  double vPrev_ = 0.0;
};
