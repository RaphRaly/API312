#pragma once

struct BjtParams {
  double Is;
  double nVt;
  double betaF;
  double betaR;
  double VAF;
  double gmin;

  // Parasitic resistances (ohms)
  double RB;
  double RC;
  double RE;

  // Junction capacitances (farads)
  double CJE;
  double CJC;

  BjtParams()
      : Is(1e-15), nVt(0.02585), betaF(200.0), betaR(2.0), VAF(100.0),
        gmin(1e-12), RB(0.0), RC(0.0), RE(0.0), CJE(0.0), CJC(0.0) {}
};
