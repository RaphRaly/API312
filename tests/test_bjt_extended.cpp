#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <iostream>

using namespace std;

bool testBjtExtendedBuilder() {
  cout << "Testing BJT Extended Builder..." << endl;

  Circuit c;

  // Create external nodes
  NodeIndex nC_ext = c.createNode();
  NodeIndex nB_ext = c.createNode();
  NodeIndex nE_ext = c.createNode();

  // Add extended BJT with parasitics using the builder
  BjtParams params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
  auto internal_nodes =
      addBjtExtended(c, nC_ext, nB_ext, nE_ext, params, false);

  // Check that internal nodes were created properly (should be different from
  // externals when resistors > 0)
  if (params.RB > 0.0 && internal_nodes.b_int == nB_ext) {
    cout
        << "Internal base node should be different from external when RB > 0"
        << endl;
    return false;
  }
  if (params.RC > 0.0 && internal_nodes.c_int == nC_ext) {
    cout << "Internal collector node should be different from external "
                 "when RC > 0"
              << endl;
    return false;
  }
  if (params.RE > 0.0 && internal_nodes.e_int == nE_ext) {
    cout
        << "Internal emitter node should be different from external when RE > 0"
        << endl;
    return false;
  }

  // Add supplies and test basic functionality
  c.addElement<VoltageSource>(nC_ext, GND, 5.0);
  c.addElement<VoltageSource>(nB_ext, GND, 0.7);
  c.addElement<VoltageSource>(nE_ext, GND, 0.0);

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "DC solve failed" << endl;
    return false;
  }

  cout << "BJT Extended Builder test PASSED" << endl;
  return true;
}

bool testBjtExtendedNoParasitics() {
  cout << "Testing BJT Extended Builder with RB=RC=RE=0..." << endl;

  Circuit c;

  // Create external nodes
  NodeIndex nC_ext = c.createNode();
  NodeIndex nB_ext = c.createNode();
  NodeIndex nE_ext = c.createNode();

  // Create BJT params with NO parasitic resistances
  BjtParams params;
  params.Is = 1e-15;
  params.nVt = 0.02585;
  params.betaF = 100.0;
  params.betaR = 1.0;
  params.VAF = 50.0;
  params.RB = 0.0; // No parasitic resistances
  params.RC = 0.0;
  params.RE = 0.0;
  params.CJE = 0.0; // No capacitances for this test
  params.CJC = 0.0;

  auto internal_nodes =
      addBjtExtended(c, nC_ext, nB_ext, nE_ext, params, false);

  // When RB=RC=RE=0, internal nodes should be the same as external nodes
  if (internal_nodes.c_int != nC_ext || internal_nodes.b_int != nB_ext ||
      internal_nodes.e_int != nE_ext) {
    cout << "Internal nodes should equal external nodes when parasitic "
                 "resistors = 0"
              << endl;
    return false;
  }

  // Add supplies and test basic functionality
  c.addElement<VoltageSource>(nC_ext, GND, 5.0);
  c.addElement<VoltageSource>(nB_ext, GND, 0.7);
  c.addElement<VoltageSource>(nE_ext, GND, 0.0);

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "DC solve failed" << endl;
    return false;
  }

  cout << "BJT Extended No Parasitics test PASSED" << endl;
  return true;
}

int main() {
  bool all_passed = true;

  if (!testBjtExtendedBuilder())
    all_passed = false;
  if (!testBjtExtendedNoParasitics())
    all_passed = false;

  if (all_passed) {
    cout << "All BJT extended tests PASSED!" << endl;
    return 0;
  } else {
    cout << "BJT extended tests FAILED!" << endl;
    return 1;
  }
}
