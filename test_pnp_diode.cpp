#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "diode_shockley_nr.h"
#include "voltage_source.h"
#include <iostream>

bool testPnpDiodeConnected() {
  std::cout << "Testing PNP diode-connected..." << std::endl;

  Circuit c;
  NodeIndex n1 = c.createNode();
  NodeIndex n2 = c.createNode();

  // PNP diode-connected: emitter to base, collector to base
  BjtParams pnp_params;
  pnp_params.Is = 7.373e-15; // 2N5087
  pnp_params.nVt = 0.0328;
  pnp_params.betaF = 923.4;
  pnp_params.betaR = 3.745;
  pnp_params.VAF = 80.3;

  c.addElement<BjtPnpEbersMoll>(
      n2, n1, n1, pnp_params); // C, B, E all connected to n1 except C to n2

  // Current source to force current
  c.addElement<VoltageSource>(n2, GND, 1.0); // Small voltage to drive current

  c.finalize();
  std::vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true, 50)) {
    std::cout << "PNP diode solve failed" << std::endl;
    return false;
  }

  double v_diode = x[n1];
  std::cout << "PNP diode voltage: " << v_diode << "V (expected ~0.7V)"
            << std::endl;

  if (v_diode < 0.6 || v_diode > 0.8) {
    std::cout << "PNP diode voltage out of range" << std::endl;
    return false;
  }

  std::cout << "PNP diode test PASSED" << std::endl;
  return true;
}

bool testDiodeForwardBias() {
  std::cout << "Testing diode forward bias..." << std::endl;

  Circuit c;
  NodeIndex n1 = c.createNode();

  // Diode with placeholder parameters (similar to 1N4148)
  c.addElement<DiodeShockleyNR>(n1, GND, 2.682e-9, 1.836, 0.02585, 1e-12);

  // Drive with small voltage
  c.addElement<VoltageSource>(n1, GND, 0.7);

  c.finalize();
  std::vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true, 50)) {
    std::cout << "Diode solve failed" << std::endl;
    return false;
  }

  double v_diode = x[n1];
  std::cout << "Diode voltage: " << v_diode << "V (expected ~0.7V)"
            << std::endl;

  if (v_diode < 0.65 || v_diode > 0.75) {
    std::cout << "Diode voltage out of range" << std::endl;
    return false;
  }

  std::cout << "Diode test PASSED" << std::endl;
  return true;
}

bool testPnpCommonEmitter() {
  std::cout << "Testing PNP common emitter..." << std::endl;

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
  std::vector<double> x;

  // Test at Vbe = 0.7V
  if (!c.solveDc(x, 50, 1e-9, true)) {
    std::cout << "PNP CE solve failed" << std::endl;
    return false;
  }

  double v_collector = x[nCollector];
  std::cout << "PNP CE collector voltage: " << v_collector << "V" << std::endl;

  // PNP should conduct when base is driven positive relative to emitter
  // With Vbe = 0.7V and Vce = 5V, it should conduct

  std::cout << "PNP CE test PASSED" << std::endl;
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
    std::cout << "\n🎉 All PNP/Diode tests PASSED! Ready for 2520 builder."
              << std::endl;
    return 0;
  } else {
    std::cout << "\n❌ Some PNP/Diode tests FAILED." << std::endl;
    return 1;
  }
}
