#pragma once
#include "bjt_params.h"
#include "capacitor_trap.h"
#include "circuit.h"
#include "elements.h"
#include "mna_types.h"
#include "resistor.h"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace std;

// ------------------------------
// Utility: safe exponential
// ------------------------------
inline double safeExp(double x) {
  constexpr double LIM = 40.0;
  if (x < -LIM)
    return exp(-LIM);
  if (x > LIM)
    return exp(LIM);
  return exp(x);
}

// ------------------------------
// Minimal Ebers-Moll BJT (NPN)
// Uses the Transport Model for better numerical stability.
// ------------------------------
class BjtNpnEbersMoll final : public IElement, public INewtonElement {
public:
  BjtNpnEbersMoll(NodeIndex c, NodeIndex b, NodeIndex e, BjtParams p)
      : nc(c), nb(b), ne(e), par(p) {}

  BjtNpnEbersMoll(const string & /*name*/, NodeIndex c, NodeIndex b,
                  NodeIndex e, BjtParams p)
      : nc(c), nb(b), ne(e), par(p) {}

  void computeLimitedVoltages(const LimitContext &ctx) override {
    const double Vb = nodeVoltage(ctx.x, nb);
    const double Vc = nodeVoltage(ctx.x, nc);
    const double Ve = nodeVoltage(ctx.x, ne);

    double vbe_new = Vb - Ve;
    double vbc_new = Vb - Vc;

    const double Vb_old = nodeVoltage(ctx.xOld, nb);
    const double Vc_old = nodeVoltage(ctx.xOld, nc);
    const double Ve_old = nodeVoltage(ctx.xOld, ne);
    double vbe_old = Vb_old - Ve_old;
    double vbc_old = Vb_old - Vc_old;

    double vcrit = par.nVt * log(par.nVt / (1.414 * par.Is));

    limitedState.vbe_lim = pnjlim(vbe_new, vbe_old, par.nVt, vcrit);
    limitedState.vbc_lim = pnjlim(vbc_new, vbc_old, par.nVt, vcrit);
  }

  void stamp(StampContext & /*ctx*/) const override {}

  void getDcConnections(vector<pair<int, int>> &connections) const override {
    connections.push_back({nb, ne});
    connections.push_back({nb, nc});
  }

  NodeIndex getC() const { return nc; }
  NodeIndex getB() const { return nb; }
  NodeIndex getE() const { return ne; }

  void stampNewton(StampContext &ctx,
                   const vector<double> & /*xGuess*/) const override {
    const double Vbe = limitedState.vbe_lim;
    const double Vbc = limitedState.vbc_lim;

    const double expBE = safeExp(Vbe / par.nVt);
    const double expBC = safeExp(Vbc / par.nVt);

    // Currents (without Early effect first)
    const double I_tran = par.Is * (expBE - expBC);
    const double I_be_diode =
        (par.Is / par.betaF) * (expBE - 1.0) + par.gmin * Vbe;
    const double I_bc_diode =
        (par.Is / par.betaR) * (expBC - 1.0) + par.gmin * Vbc;

    // Vce for Early effect (Vce = Vbe - Vbc)
    const double Vce = Vbe - Vbc;

    // Base collector current without Early
    const double Ic_base = I_tran - I_bc_diode;

    // Early effect: output conductance go = |Ic|/VAF
    // Ic_total = Ic_base * (1 + Vce/VAF) ≈ Ic_base + Ic_base*Vce/VAF
    // For small-signal: go = dIc/dVce = |Ic_base|/VAF
    const double go = (par.VAF > 0.0) ? abs(Ic_base) / par.VAF : 0.0;
    const double Ic = Ic_base + go * Vce;

    const double Ib = I_be_diode + I_bc_diode;
    const double Ie = -(Ic + Ib);

    // Conductances
    const double g_tran_f = (par.Is / par.nVt) * expBE;
    const double g_tran_r = (par.Is / par.nVt) * expBC;
    const double g_be = (par.Is / (par.betaF * par.nVt)) * expBE + par.gmin;
    const double g_bc = (par.Is / (par.betaR * par.nVt)) * expBC + par.gmin;

    // Jacobian dI/dVnode
    // Vbe = Vb - Ve, Vbc = Vb - Vc, Vce = Vc - Ve (but Vce = Vbe - Vbc)
    // dVce/dVc = +1, dVce/dVe = -1, dVce/dVb = 0
    // Ic row (including Early effect: go contributes to dIc/dVc and dIc/dVe)
    const double dIc_dVb = g_tran_f - g_tran_r - g_bc;
    const double dIc_dVc = g_tran_r + g_bc + go; // +go from dVce/dVc = +1
    const double dIc_dVe = -g_tran_f - go;       // -go from dVce/dVe = -1

    // Ib row (no Early effect on base current)
    const double dIb_dVb = g_be + g_bc;
    const double dIb_dVc = -g_bc;
    const double dIb_dVe = -g_be;

    // Ie row (derived from Ie = -(Ic + Ib))
    const double dIe_dVb = -(dIc_dVb + dIb_dVb);
    const double dIe_dVc = -(dIc_dVc + dIb_dVc);
    const double dIe_dVe = -(dIc_dVe + dIb_dVe);

    // RHS = J*V_op - I_op
    // JV_Ic includes Early effect contribution: go*Vce
    const double JV_Ic = g_tran_f * Vbe + (-g_tran_r - g_bc) * Vbc + go * Vce;
    const double JV_Ib = g_be * Vbe + g_bc * Vbc;
    const double JV_Ie = -(JV_Ic + JV_Ib);

    auto stampRow = [&](int row, double I_op, double d_dVc, double d_dVb,
                        double d_dVe, double JV) {
      if (row == GND)
        return;
      if (nc != GND)
        ctx.sys.addA(row, nc, d_dVc);
      if (nb != GND)
        ctx.sys.addA(row, nb, d_dVb);
      if (ne != GND)
        ctx.sys.addA(row, ne, d_dVe);
      ctx.sys.addZ(row, JV - I_op);
    };

    stampRow(nc, Ic, dIc_dVc, dIc_dVb, dIc_dVe, JV_Ic);
    stampRow(nb, Ib, dIb_dVc, dIb_dVb, dIb_dVe, JV_Ib);
    stampRow(ne, Ie, dIe_dVc, dIe_dVb, dIe_dVe, JV_Ie);
  }

  void limitUpdate(const vector<double> & /*xNew*/) override {}

private:
  int nc, nb, ne;
  BjtParams par;
  LimitedState limitedState;
};

// ------------------------------
// Minimal Ebers-Moll BJT (PNP)
// ------------------------------
class BjtPnpEbersMoll final : public IElement, public INewtonElement {
public:
  BjtPnpEbersMoll(NodeIndex c, NodeIndex b, NodeIndex e, BjtParams p)
      : nc(c), nb(b), ne(e), par(p) {}

  BjtPnpEbersMoll(const string & /*name*/, NodeIndex c, NodeIndex b,
                  NodeIndex e, BjtParams p)
      : nc(c), nb(b), ne(e), par(p) {}

  void computeLimitedVoltages(const LimitContext &ctx) override {
    const double Vb = nodeVoltage(ctx.x, nb);
    const double Vc = nodeVoltage(ctx.x, nc);
    const double Ve = nodeVoltage(ctx.x, ne);

    double veb_new = Ve - Vb;
    double vcb_new = Vc - Vb;

    const double Vb_old = nodeVoltage(ctx.xOld, nb);
    const double Vc_old = nodeVoltage(ctx.xOld, nc);
    const double Ve_old = nodeVoltage(ctx.xOld, ne);
    double veb_old = Ve_old - Vb_old;
    double vcb_old = Vc_old - Vb_old;

    double vcrit = par.nVt * log(par.nVt / (1.414 * par.Is));

    limitedState.vbe_lim =
        pnjlim(veb_new, veb_old, par.nVt, vcrit); // reusing vbe_lim for veb
    limitedState.vbc_lim =
        pnjlim(vcb_new, vcb_old, par.nVt, vcrit); // reusing vbc_lim for vcb
  }

  void stamp(StampContext & /*ctx*/) const override {}

  void getDcConnections(vector<pair<int, int>> &connections) const override {
    connections.push_back({nb, ne});
    connections.push_back({nb, nc});
  }

  NodeIndex getC() const { return nc; }
  NodeIndex getB() const { return nb; }
  NodeIndex getE() const { return ne; }

  void stampNewton(StampContext &ctx,
                   const vector<double> & /*xGuess*/) const override {
    const double Veb = limitedState.vbe_lim;
    const double Vcb = limitedState.vbc_lim;

    const double expEB = safeExp(Veb / par.nVt);
    const double expCB = safeExp(Vcb / par.nVt);

    const double I_tran = par.Is * (expEB - expCB);
    const double I_eb_diode =
        (par.Is / par.betaF) * (expEB - 1.0) + par.gmin * Veb;
    const double I_cb_diode =
        (par.Is / par.betaR) * (expCB - 1.0) + par.gmin * Vcb;

    // Vec for Early effect (Vec = Veb - Vcb = Ve - Vc, which is -Vce for PNP)
    const double Vec = Veb - Vcb;

    // Base collector current without Early
    // For PNP: Ic flows INTO collector (opposite convention)
    const double Ic_base = -I_tran + I_cb_diode;

    // Early effect: output conductance go = |Ic|/VAF
    // Model as resistor ro between C and E:
    //   Iro entering collector = go*(Vc - Ve) = -go*Vec
    //   Iro entering emitter   = go*(Ve - Vc) = +go*Vec
    const double go = (par.VAF > 0.0) ? abs(Ic_base) / par.VAF : 0.0;
    const double Ic = Ic_base - go * Vec; // FIX: ro current into C

    // Currents entering terminals (Ie includes ro current)
    const double Ie = I_tran + I_eb_diode + go * Vec; // FIX: ro current into E
    const double Ib = -(Ie + Ic); // go terms cancel, base current unchanged

    const double g_tran_f = (par.Is / par.nVt) * expEB;
    const double g_tran_r = (par.Is / par.nVt) * expCB;
    const double g_eb = (par.Is / (par.betaF * par.nVt)) * expEB + par.gmin;
    const double g_cb = (par.Is / (par.betaR * par.nVt)) * expCB + par.gmin;

    // dI/dVnode
    // Veb = Ve - Vb, Vcb = Vc - Vb, Vec = Ve - Vc
    // dVec/dVe = +1, dVec/dVc = -1, dVec/dVb = 0
    // Ie row (includes +go*Vec, so dIe/dVe += go, dIe/dVc -= go)
    const double dIe_dVe = g_tran_f + g_eb + go; // FIX: +go from dVec/dVe = +1
    const double dIe_dVc = -g_tran_r - go;       // FIX: -go from dVec/dVc = -1
    const double dIe_dVb = -(g_tran_f + g_eb - g_tran_r); // unchanged

    // Ic row (includes -go*Vec, so dIc/dVe -= go, dIc/dVc += go)
    const double dIc_dVe = -g_tran_f - go;       // FIX: -go from -go*dVec/dVe
    const double dIc_dVc = g_tran_r + g_cb + go; // FIX: +go from -go*dVec/dVc
    const double dIc_dVb = g_tran_f - (g_tran_r + g_cb); // unchanged

    // Ib row (derived from Ib = -(Ie + Ic))
    const double dIb_dVe = -(dIe_dVe + dIc_dVe);
    const double dIb_dVc = -(dIe_dVc + dIc_dVc);
    const double dIb_dVb = -(dIe_dVb + dIc_dVb);

    // RHS contributions (JV = J * V_op)
    const double JV_Ie =
        (g_tran_f + g_eb) * Veb + (-g_tran_r) * Vcb + go * Vec; // FIX: +go*Vec
    const double JV_Ic =
        (-g_tran_f) * Veb + (g_tran_r + g_cb) * Vcb - go * Vec; // FIX: -go*Vec
    const double JV_Ib = -(JV_Ie + JV_Ic);

    auto stampRow = [&](int row, double I_op, double d_dVc, double d_dVb,
                        double d_dVe, double JV) {
      if (row == GND)
        return;
      if (nc != GND)
        ctx.sys.addA(row, nc, d_dVc);
      if (nb != GND)
        ctx.sys.addA(row, nb, d_dVb);
      if (ne != GND)
        ctx.sys.addA(row, ne, d_dVe);
      ctx.sys.addZ(row, JV - I_op);
    };

    stampRow(ne, Ie, dIe_dVc, dIe_dVb, dIe_dVe, JV_Ie);
    stampRow(nc, Ic, dIc_dVc, dIc_dVb, dIc_dVe, JV_Ic);
    stampRow(nb, Ib, dIb_dVc, dIb_dVb, dIb_dVe, JV_Ib);
  }

  void limitUpdate(const vector<double> & /*xNew*/) override {}

private:
  int nc, nb, ne;
  BjtParams par;
  LimitedState limitedState;
};

// ------------------------------
// BJT Extended Builder
// ------------------------------
struct BjtExtendedNodes {
  NodeIndex c_int, b_int, e_int;
};

inline BjtExtendedNodes addBjtExtended(Circuit &c, NodeIndex c_ext,
                                       NodeIndex b_ext, NodeIndex e_ext,
                                       const BjtParams &p, bool isPnp,
                                       const string &name = "Q") {
  NodeIndex c_int = (p.RC > 0.0) ? c.createNode(name + "_Ci") : c_ext;
  NodeIndex b_int = (p.RB > 0.0) ? c.createNode(name + "_Bi") : b_ext;
  NodeIndex e_int = (p.RE > 0.0) ? c.createNode(name + "_Ei") : e_ext;

  if (p.RB > 0.0)
    c.addElement<Resistor>(name + "_RB", b_ext, b_int, p.RB);
  if (p.RC > 0.0)
    c.addElement<Resistor>(name + "_RC", c_ext, c_int, p.RC);
  if (p.RE > 0.0)
    c.addElement<Resistor>(name + "_RE", e_ext, e_int, p.RE);

  if (p.CJE > 0.0)
    c.addElement<CapacitorTrap>(name + "_CJE", b_int, e_int, p.CJE);
  if (p.CJC > 0.0)
    c.addElement<CapacitorTrap>(name + "_CJC", b_int, c_int, p.CJC);

  if (isPnp) {
    c.addElement<BjtPnpEbersMoll>(name, c_int, b_int, e_int, p);
  } else {
    c.addElement<BjtNpnEbersMoll>(name, c_int, b_int, e_int, p);
  }
  return {c_int, b_int, e_int};
}
