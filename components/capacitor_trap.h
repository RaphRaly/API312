#pragma once
#include "elements.h"
#include "mna_types.h"
#include "stamping.h"
#include <stdexcept>

using namespace std;

// Trapezoidal companion model capacitor:
// i = G*v + Ieq, where G = 2C/dt and Ieq = -(iPrev + G*vPrev)
class CapacitorTrap final : public IElement, public IDynamicElement {
public:
  CapacitorTrap(NodeIndex a, NodeIndex b, double C) : na(a), nb(b), C_(C) {
    if (C_ < 0.0)
      throw invalid_argument("CapacitorTrap: C must be >= 0");
  }

  CapacitorTrap(const string & /*name*/, NodeIndex a, NodeIndex b,
                double C)
      : na(a), nb(b), C_(C) {
    if (C_ < 0.0)
      throw invalid_argument("CapacitorTrap: C must be >= 0");
  }

  void beginStep(double dt) override {
    if (dt <= 0.0) {
      // DC analysis: capacitor is open circuit
      G_ = 0.0;
      Ieq_ = 0.0;
      return;
    }
    G_ = 2.0 * C_ / dt;
    Ieq_ = -(iPrev_ + G_ * vPrev_);
  }

  void commitStep(const vector<double> &xSolved) override {
    // Correct version using nodeVoltage:
    const double v_new = nodeVoltage(xSolved, na) - nodeVoltage(xSolved, nb);

    if (G_ == 0.0) {
      vPrev_ = v_new;
      iPrev_ = 0.0; // DC steady state
      return;
    }

    const double i = G_ * v_new + Ieq_;
    vPrev_ = v_new;
    iPrev_ = i;
  }

  void stamp(StampContext &ctx) const override {
    // Stamp G and Ieq for current time step.
    stampConductance(ctx.sys, na, nb, G_);
    stampCurrentSource(ctx.sys, na, nb, Ieq_);
  }

  void getDcConnections(
      vector<pair<int, int>> & /*connections*/) const override {
    // Capacitor is open at DC
  }

private:
  NodeIndex na, nb;
  double C_;

  // Companion parameters for current step
  double G_ = 0.0;
  double Ieq_ = 0.0;

  // History state
  double vPrev_ = 0.0;
  double iPrev_ = 0.0;
};
