#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "diode_shockley_nr.h"
#include "voltage_source.h"
#include <iostream>

using namespace std;

bool testPnpDiodeConnected() {
  cout << "Testing PNP diode-connected..." << endl;

  Circuit c;
  NodeIndex n1 = c.createNode();
  NodeIndex n2 = c.createNode();

  // PNP diode-connected: B and C shorted to GND, Emitter at n1 pulled by
  // current
  BjtParams pnp_params;
  pnp_params.Is = 7.373e-15; // 2N5087
  pnp_params.nVt = 0.02585;  // Standard Vt
  pnp_params.betaF = 923.4;
  pnp_params.betaR = 3.745;
  pnp_params.VAF = 80.3;

  // C=GND, B=GND, E=n1
  c.addElement<BjtPnpEbersMoll>(GND, GND, n1, pnp_params);

  // Drive emitter with 1mA (approx)
  c.addElement<VoltageSource>(n2, GND, 5.0);
  c.addElement<Resistor>(n2, n1, 4.3e3); // ~1mA current

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true, 50)) {
    cout << "PNP diode solve failed" << endl;
    return false;
  }

  double v_diode = x[n1];
  cout << "PNP diode voltage (Veb): " << v_diode << "V (expected ~0.7V)"
            << endl;

  if (v_diode < 0.5 || v_diode > 0.85) {
    cout << "PNP diode voltage out of range: " << v_diode << endl;
    return false;
  }

  cout << "PNP diode test PASSED" << endl;
  return true;
}

bool testDiodeForwardBias() {
  cout << "Testing diode forward bias..." << endl;

  Circuit c;
  NodeIndex n1 = c.createNode();

  // Diode with placeholder parameters (similar to 1N4148)
  c.addElement<DiodeShockleyNR>(n1, GND, 2.682e-9, 1.836, 0.02585, 1e-12);

  // Drive with small voltage
  c.addElement<VoltageSource>(n1, GND, 0.7);

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true, 50)) {
    cout << "Diode solve failed" << endl;
    return false;
  }

  double v_diode = x[n1];
  cout << "Diode voltage: " << v_diode << "V (expected ~0.7V)"
            << endl;

  if (v_diode < 0.65 || v_diode > 0.75) {
    cout << "Diode voltage out of range" << endl;
    return false;
  }

  cout << "Diode test PASSED" << endl;
  return true;
}

bool testPnpCommonEmitter() {
  cout << "Testing PNP common emitter..." << endl;

  Circuit c;
  NodeIndex nBase = c.createNode();
  NodeIndex nCollector = c.createNode();

  // PNP common emitter: base driven, collector loaded
  BjtParams pnp_params;
  pnp_params.Is = 7.373e-15;
  pnp_params.nVt = 0.0328;
  pnp_params.betaF = 923.4;
  pnp_params.betaR = 3.745;
  pnp_params.VAF = 80.3;

  c.addElement<BjtPnpEbersMoll>(nCollector, nBase, GND, pnp_params);

  // Base drive
  c.addElement<VoltageSource>(nBase, GND, 0.0); // Will sweep this

  // Collector load
  c.addElement<VoltageSource>(nCollector, GND,
                               -5.0); // Negative voltage for PNP

  c.finalize();
  vector<double> x;

  // Test at Vbe = 0.7V
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "PNP CE solve failed" << endl;
    return false;
  }

  double v_collector = x[nCollector];
  cout << "PNP CE collector voltage: " << v_collector << "V" << endl;

  // PNP should conduct when base is driven positive relative to emitter
  // With Vbe = 0.7V and Vce = 5V, it should conduct

  cout << "PNP CE test PASSED" << endl;
  return true;
}

int main() {
  bool all_passed = true;

  if (!testPnpDiodeConnected())
    all_passed = false;
  if (!testDiodeForwardBias())
    all_passed = false;
  if (!testPnpCommonEmitter())
    all_passed = false;

  if (all_passed) {
    cout << "\n🎉 All PNP/Diode tests PASSED! Ready for 2520 builder."
              << endl;
    return 0;
  } else {
    cout << "\n❌ Some PNP/Diode tests FAILED." << endl;
    return 1;
  }
}
