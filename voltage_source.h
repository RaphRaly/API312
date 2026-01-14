#pragma once
#include "elements.h"
#include "mna_types.h"
#include <stdexcept>

// Independent voltage source between a and b.
// Adds one extra branch variable (current through the source).
class VoltageSource final : public IElement, public IBranchElement {
public:
  VoltageSource(NodeIndex a, NodeIndex b, double V) : na(a), nb(b), V_(V) {}
  VoltageSource(const std::string &name, NodeIndex p, NodeIndex n, double V)
      : na(p), nb(n), V_(V) {}

  void setVoltage(double V) { V_ = V; }

  // IBranchElement
  int branchCount() const override { return 1; }

  void setBranchIndex(int firstBranchIndex) override {
    branchIndex = firstBranchIndex;
  }

  void stamp(StampContext &ctx) const override {
    if (branchIndex < 0)
      throw std::runtime_error("VoltageSource: branchIndex not set (did you "
                               "call Circuit::finalize?)");

    // MNA stamping:
    // Va - Vb = V
    // Add branch current variable iVs at row/col = branchIndex.
    const int k = branchIndex;

    if (na != GND) {
      ctx.sys.addA(na, k, +1.0);
      ctx.sys.addA(k, na, +1.0);
    }
    if (nb != GND) {
      ctx.sys.addA(nb, k, -1.0);
      ctx.sys.addA(k, nb, -1.0);
    }

    ctx.sys.addZ(k, V_ * ctx.scale);
  }

  void getDcConnections(
      std::vector<std::pair<int, int>> &connections) const override {
    connections.push_back({na, nb});
  }

private:
  NodeIndex na, nb;
  double V_;
  int branchIndex = -1;
};
