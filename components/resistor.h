#pragma once
#include "elements.h"
#include "mna_types.h"
#include "stamping.h"
#include <stdexcept>

using namespace std;

class Resistor final : public IElement {
public:
  Resistor(NodeIndex a, NodeIndex b, double R) : na(a), nb(b), R_(R) {
    if (R_ <= 0.0)
      throw invalid_argument("Resistor: R must be > 0");
  }

  Resistor(const string & /*name*/, NodeIndex a, NodeIndex b, double R)
      : na(a), nb(b), R_(R) {
    if (R_ <= 0.0)
      throw invalid_argument("Resistor: R must be > 0");
  }

  void stamp(StampContext &ctx) const override {
    const double g = 1.0 / R_;
    stampConductance(ctx.sys, na, nb, g);
  }

  void getDcConnections(
      vector<pair<int, int>> &connections) const override {
    connections.push_back({na, nb});
  }

  double getR() const { return R_; }
  NodeIndex getNa() const { return na; }
  NodeIndex getNb() const { return nb; }

private:
  NodeIndex na, nb;
  double R_;
};
