#pragma once
#include "elements.h"
#include "mna_types.h"
#include "stamping.h"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

// Shockley diode with Newton-Raphson linearization and PN junction limiting.
// Current flows from anode -> cathode.
// Supports zener breakdown modeling.
class DiodeShockleyNR final : public IElement, public INewtonElement {
public:
  // Compute limited junction voltage for Newton linearization
  void computeLimitedVoltages(const LimitContext &ctx) override {
    const double Va = nodeVoltage(ctx.x, a);
    const double Vc = nodeVoltage(ctx.x, c);
    double vd_new = Va - Vc; // Anode-Cathode voltage

    // Use previous converged value as reference for limiting
    const double Va_old = nodeVoltage(ctx.xOld, a);
    const double Vc_old = nodeVoltage(ctx.xOld, c);
    double vd_old = Va_old - Vc_old;

    double nVt = n_ * Vt_;
    double vcrit = nVt * log(nVt / (1.414 * Is_));

    limitedVd = pnjlim(vd_new, vd_old, nVt, vcrit);
  }

  void limitUpdate(const vector<double> &xNew) override {
    // Update limiting reference
    const double Va = nodeVoltage(xNew, a);
    const double Vc = nodeVoltage(xNew, c);
    limitedVd = Va - Vc;
  }

  DiodeShockleyNR(const string & /*name*/, NodeIndex anode,
                  NodeIndex cathode, double Is, double n, double Vt = 0.02585,
                  double gmin = 1e-12, double BV = 0.0, double IBV = 1e-3)
      : a(anode), c(cathode), Is_(Is), n_(n), Vt_(Vt), gmin_(gmin), BV_(BV),
        IBV_(IBV) {
    if (Is_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: Is must be > 0");
    if (n_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: n must be > 0");
    if (Vt_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: Vt must be > 0");
    if (gmin_ < 0.0)
      throw invalid_argument("DiodeShockleyNR: gmin must be >= 0");
    if (BV_ < 0.0)
      throw invalid_argument("DiodeShockleyNR: BV must be >= 0");
    if (IBV_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: IBV must be > 0");
  }

  DiodeShockleyNR(NodeIndex anode, NodeIndex cathode, double Is, double n,
                  double Vt = 0.02585, double gmin = 1e-12, double BV = 0.0,
                  double IBV = 1e-3)
      : a(anode), c(cathode), Is_(Is), n_(n), Vt_(Vt), gmin_(gmin), BV_(BV),
        IBV_(IBV) {
    if (Is_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: Is must be > 0");
    if (n_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: n must be > 0");
    if (Vt_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: Vt must be > 0");
    if (gmin_ < 0.0)
      throw invalid_argument("DiodeShockleyNR: gmin must be >= 0");
    if (BV_ < 0.0)
      throw invalid_argument("DiodeShockleyNR: BV must be >= 0");
    if (IBV_ <= 0.0)
      throw invalid_argument("DiodeShockleyNR: IBV must be > 0");
  }

  // Nonlinear element does nothing in the pure linear pass.
  void stamp(StampContext & /*ctx*/) const override {}

  void getDcConnections(
      vector<pair<int, int>> &connections) const override {
    connections.push_back({a, c});
  }

  void stampNewton(StampContext &ctx,
                   const vector<double> & /*xGuess*/) const override {
    // Use limited diode voltage for linearization
    const double v = limitedVd;

    // Handle zener breakdown if BV > 0
    if (BV_ > 0.0 && v < -BV_) {
      // Zener breakdown region: piecewise linear approximation
      // I = IBV * ((v + BV) / BV)  (linear approximation)
      const double gd_zener = IBV_ / BV_; // conductance in breakdown region
      const double Ieq_zener = -gd_zener * BV_; // intercept current

      stampConductance(ctx.sys, a, c, gd_zener + gmin_);
      stampCurrentSource(ctx.sys, a, c, Ieq_zener);
      return;
    }

    // Normal forward/reverse diode region
    const double Vte = n_ * Vt_;

    // Prevent exp overflow/underflow.
    double v_scaled = v / Vte;
    if (v_scaled < -40.0)
      v_scaled = -40.0;
    if (v_scaled > 40.0)
      v_scaled = 40.0;
    const double arg = v_scaled;
    const double ev = exp(arg);

    const double Id = Is_ * (ev - 1.0);

    // Differential conductance
    double gd = (Is_ / Vte) * ev;

    // Add gmin for numerical stability (SPICE-style trick)
    gd += gmin_;

    // Norton intercept: I = gd*v + Ieq
    const double Ieq = Id - gd * v;

    stampConductance(ctx.sys, a, c, gd);
    stampCurrentSource(ctx.sys, a, c, Ieq);
  }

private:
  NodeIndex a, c;
  double Is_, n_, Vt_, gmin_;
  double BV_, IBV_; // Zener breakdown parameters

  // Limited diode voltage for Newton linearization
  mutable double limitedVd = 0.0;
};
