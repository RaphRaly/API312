#pragma once
#include "elements.h"
#include "mna_types.h"
#include "stamping.h"
#include <stdexcept>

class Resistor final : public IElement {
public:
  Resistor(NodeIndex a, NodeIndex b, double R) : na(a), nb(b), R_(R) {
    if (R_ <= 0.0)
      throw std::invalid_argument("Resistor: R must be > 0");
  }

  Resistor(const std::string &name, NodeIndex a, NodeIndex b, double R)
      : na(a), nb(b), R_(R) {
    if (R_ <= 0.0)
      throw std::invalid_argument("Resistor: R must be > 0");
  }

  void stamp(StampContext &ctx) const override {
    const double g = 1.0 / R_;
    stampConductance(ctx.sys, na, nb, g);
  }

  void getDcConnections(
      std::vector<std::pair<int, int>> &connections) const override {
    connections.push_back({na, nb});
  }

private:
  NodeIndex na, nb;
  double R_;
};
