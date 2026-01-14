#pragma once
#include "elements.h"
#include "mna_types.h"
#include "stamping.h"

class CurrentSource final : public IElement {
public:
  // Current from a -> b
  CurrentSource(NodeIndex a, NodeIndex b, double I) : na(a), nb(b), I_(I) {}
  CurrentSource(const std::string & /*name*/, NodeIndex a, NodeIndex b,
                double I)
      : na(a), nb(b), I_(I) {}

  void setCurrent(double I) { I_ = I; }

  void stamp(StampContext &ctx) const override {
    stampCurrentSource(ctx.sys, na, nb, I_ * ctx.scale);
  }

  void getDcConnections(
      std::vector<std::pair<int, int>> & /*connections*/) const override {
    // Ideal current sources are infinite impedance, so they don't provide a DC
    // path for node referencing. However, for connectivity, they connect two
    // nodes. But SPICE usually treats them as open for "floating node" checks.
    // We follow the strict rule: only conduction paths (R, V, D-on) count.
  }

private:
  NodeIndex na, nb;
  double I_;
};
