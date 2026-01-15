#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <cmath>
#include <iostream>

using namespace std;

// ==========================================
// A) VoltageSource Micro-Tests
// ==========================================

bool testVoltageSourceSigns() {
  cout << "\n[Test] VoltageSource Signs..." << endl;
  bool all_ok = true;

  // Case 1: V(GND) - V(VEE) = 15 => VEE should be -15
  {
    Circuit c;
    NodeIndex nVEE = c.createNode();
    // VoltageSource(na, nb, V) means V(na) - V(nb) = V
    // VoltageSource(GND, nVEE, 15.0) => 0 - V(nVEE) = 15 => V(nVEE) = -15
    c.addElement<VoltageSource>(GND, nVEE, 15.0);

    vector<double> x;
    if (!c.solveDc(x, 10, 1e-9, true)) {
      cout << "  FAIL: Case 1 solve failed." << endl;
      all_ok = false;
    } else {
      double v_vee = x[nVEE];
      if (abs(v_vee - (-15.0)) > 1e-6) {
        cout << "  FAIL: Case 1 VEE=" << v_vee << " expected -15.0"
                  << endl;
        all_ok = false;
      } else {
        cout << "  PASS: Case 1 VEE=" << v_vee << endl;
      }
    }
  }

  // Case 2: V(VCC) - V(GND) = 15 => VCC should be +15
  {
    Circuit c;
    NodeIndex nVCC = c.createNode();
    // VoltageSource(nVCC, GND, 15.0) => V(nVCC) - 0 = 15
    c.addElement<VoltageSource>(nVCC, GND, 15.0);

    vector<double> x;
    if (!c.solveDc(x, 10, 1e-9, true)) {
      cout << "  FAIL: Case 2 solve failed." << endl;
      all_ok = false;
    } else {
      double v_vcc = x[nVCC];
      if (abs(v_vcc - 15.0) > 1e-6) {
        cout << "  FAIL: Case 2 VCC=" << v_vcc << " expected 15.0"
                  << endl;
        all_ok = false;
      } else {
        cout << "  PASS: Case 2 VCC=" << v_vcc << endl;
      }
    }
  }

  return all_ok;
}

// ==========================================
// B) BJT Extended Micro-Tests
// ==========================================

bool testBjtExtendedBuilder() {
  cout << "\n[Test] BJT Extended Builder (2N3053)..." << endl;
  Circuit c;
  NodeIndex nC_ext = c.createNode();
  NodeIndex nB_ext = c.createNode();
  NodeIndex nE_ext = c.createNode();

  // Use a known model or default if not found (assuming map is populated)
  BjtParams params;
  try {
    params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
  } catch (...) {
    cout << "  WARNING: 2N3053 not found, using default params."
              << endl;
    params.Is = 1e-15;
    params.betaF = 100;
    params.betaR = 1;
  }

  auto internal_nodes =
      addBjtExtended(c, nC_ext, nB_ext, nE_ext, params, false);

  // Sanity check on node creation logic
  if (params.RB > 0.0 && internal_nodes.b_int == nB_ext) {
    cout << "  FAIL: RB > 0 but no internal base node created."
              << endl;
    return false;
  }

  // Bias it: Vc=5, Vb=0.7, Ve=0 => Active region
  c.addElement<VoltageSource>(nC_ext, GND, 5.0);
  c.addElement<VoltageSource>(nB_ext, GND, 0.7);
  c.addElement<VoltageSource>(nE_ext, GND, 0.0);

  // c.finalize(); is called inside solveDc usually or implicit
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "  FAIL: DC Solve failed for BJT Extended." << endl;
    return false;
  }
  cout << "  PASS: DC Solve converged." << endl;
  return true;
}

bool testBjtExtendedNoParasitics() {
  cout << "\n[Test] BJT Extended No Parasitics..." << endl;
  Circuit c;
  NodeIndex nC_ext = c.createNode();
  NodeIndex nB_ext = c.createNode();
  NodeIndex nE_ext = c.createNode();

  BjtParams params;
  params.Is = 1e-15;
  params.nVt = 0.02585;
  params.betaF = 100;
  params.betaR = 1;
  params.VAF = 50;
  params.RB = 0;
  params.RC = 0;
  params.RE = 0;
  params.CJE = 0;
  params.CJC = 0;

  auto internal_nodes =
      addBjtExtended(c, nC_ext, nB_ext, nE_ext, params, false);
  if (internal_nodes.c_int != nC_ext || internal_nodes.b_int != nB_ext ||
      internal_nodes.e_int != nE_ext) {
    cout
        << "  FAIL: Parasitics are 0 but internal nodes != external nodes."
        << endl;
    return false;
  }

  c.addElement<VoltageSource>(nC_ext, GND, 5.0);
  c.addElement<VoltageSource>(nB_ext, GND, 0.7);
  c.addElement<VoltageSource>(nE_ext, GND, 0.0);

  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "  FAIL: DC Solve failed for Ideal BJT." << endl;
    return false;
  }
  cout << "  PASS: DC Solve converged." << endl;
  return true;
}

bool testBjtPnp() {
  cout << "\n[Test] BJT PNP (Active Mode)..." << endl;
  Circuit c;
  NodeIndex nC = c.createNode();
  NodeIndex nB = c.createNode();
  NodeIndex nE = c.createNode();

  BjtParams params;
  params.Is = 1e-15;
  params.nVt = 0.02585;
  params.betaF = 100;
  params.betaR = 1;
  // Ideal PNP

  // Active Bias: Ve=5, Vb=4.3, Vc=0
  c.addElement<VoltageSource>(nE, GND, 5.0);
  c.addElement<VoltageSource>(nB, GND, 4.3);
  c.addElement<VoltageSource>(nC, GND, 0.0);

  // E, B, C order for PNP extended? No, addBjtExtended takes C, B, E.
  // For PNP, isPnp=true.
  // addBjtExtended(c, nC_ext, nB_ext, nE_ext...)
  addBjtExtended(c, nC, nB, nE, params, true);

  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "  FAIL: DC Solve failed for PNP." << endl;
    return false;
  }
  cout << "  PASS: DC Solve converged for PNP." << endl;
  return true;
}

int main() {
  bool ok = true;
  if (!testVoltageSourceSigns())
    ok = false;
  if (!testBjtExtendedBuilder())
    ok = false;
  if (!testBjtExtendedNoParasitics())
    ok = false;
  if (!testBjtPnp())
    ok = false;

  if (ok)
    cout << "\nALL DIAGNOSTICS PASSED.\n";
  else
    cout << "\nSOME DIAGNOSTICS FAILED.\n";
  return ok ? 0 : 1;
}
