#pragma once
#include "bjt_params.h"
#include <string>
#include <unordered_map>

using namespace std;

struct SpiceBjtModel {
  double Is;
  double Ne;
  double Bf;
  double Br;
  double Vaf;
  double RB;
  double RC;
  double RE;
  double CJE;
  double CJC;

  double nVt() const { return Ne * 0.02585; }
};

struct SpiceDiodeModel {
  double Is;
  double N;
  double Rs;
  double BV;
  double IBV;
  double nVt() const { return N * 0.02585; }
};

// Global accessor for BJT models
inline SpiceBjtModel getSpiceBjtModel(const string &name) {
  // Order: Is, Ne, Bf, Br, Vaf, RB, RC, RE, CJE, CJC
  if (name == "SKA4693" || name == "BC414C")
    return {5.911e-15, 1.0, 394.1, 10.35, 44.95, 10.0, 0.5, 0.1, 0, 0};
  if (name == "BC416C") // PNP version of BC414C for current mirror
    return {5.911e-15, 1.0, 394.1, 10.35, 44.95, 10.0, 0.5, 0.1, 0, 0};
  if (name == "TIS98")
    return {4.872e-15, 1.0, 96.00, 6.935,     100.0,
            10.0,      0.5, 0.1,   10.49e-12, 8.866e-12};
  if (name == "2N5087")
    return {7.373e-15, 1.0, 923.4, 3.745,   80.3,
            10.0,      0.5, 0.1,   5.0e-12, 14.76e-12};
  if (name == "2N3053")
    return {5.911e-15, 1.0, 394.1, 10.35,     44.95,
            10.0,      0.5, 0.1,   8.882e-12, 7.306e-12};
  if (name == "2N4036")
    return {1.0e-14, 1.0, 100.0, 1.0, 50.0, 10.0, 0.5, 0.1, 10.0e-12, 5.0e-12};
  return {1e-15, 1.0, 100.0, 2.0, 100.0, 0, 0, 0, 0, 0}; // Default
}

// Fixed Diode models
static const SpiceDiodeModel diode_1N22361 = {2.682e-9, 1.836, 0.6453, 0.0,
                                              1e-3};
static const SpiceDiodeModel diode_1N4829 = {1.0e-9, 1.5, 1.0, 0.0, 1e-3};
static const SpiceDiodeModel diode_1N4727 = {1.0e-9, 1.5, 1.0, 3.0, 1e-3};

inline BjtParams spiceToBjtParams(const SpiceBjtModel &spice) {
  BjtParams p;
  p.Is = spice.Is;
  p.nVt = spice.nVt();
  p.betaF = spice.Bf;
  p.betaR = spice.Br;
  p.VAF = spice.Vaf;
  p.gmin = 1e-12;
  p.RB = spice.RB;
  p.RC = spice.RC;
  p.RE = spice.RE;
  p.CJE = spice.CJE;
  p.CJC = spice.CJC;
  return p;
}
