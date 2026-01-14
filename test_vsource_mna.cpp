#include "circuit.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
  std::cout << "--- MNA Voltage Source Sign Test ---" << std::endl;

  Circuit c;
  NodeIndex n1 = c.createNode("N1");

  // V1: 10V between N1 and GND
  // Convention: VoltageSource(pos, neg, value) => V(pos) - V(neg) = value
  c.addElement<VoltageSource>("V1", n1, GND, 10.0);

  // R1: 1000 ohms between N1 and GND
  c.addElement<Resistor>("R1", n1, GND, 1000.0);

  c.finalize();

  std::vector<double> x;
  bool ok = c.solveDc(x, 2, 1e-9, false);

  if (!ok) {
    std::cerr << "Solver failed to converge on linear circuit!" << std::endl;
    return 1;
  }

  std::cout << "\nUnknown Map:" << std::endl;
  for (int i = 0; i < (int)x.size(); ++i) {
    std::cout << "Index [" << i << "]: " << std::left << std::setw(15)
              << c.getUnknownMeaning(i) << " = " << x[i] << std::endl;
  }

  // Expected:
  // V(N1) = 10.0
  // I(V1) = ?
  // KCL at N1: I(R1) + I(V1) = 0
  // I(R1) = V(N1) / R1 = 10 / 1000 = 10mA
  // So I(V1) should be -10mA if I(V1) is current entering N1 from source.
  // Let's re-verify sign in voltage_source.h:
  // A[na][k] += 1.0  => i flows OUT of na.
  // So KCL at na: sum(i_others) + i_branch = 0.
  // I(R1) + I(V1) = 0 => 10mA + I(V1) = 0 => I(V1) = -10mA.

  double v_n1 = x[n1];
  double i_v1 = x[c.getNumNodes()]; // First and only branch variable

  std::cout << "\nResults:" << std::endl;
  std::cout << "V(N1)  = " << v_n1 << " V (Expected: 10.0)" << std::endl;
  std::cout << "I(V1)  = " << i_v1 * 1000.0 << " mA (Expected: -10.0)"
            << std::endl;

  bool pass = true;
  if (std::abs(v_n1 - 10.0) > 1e-6)
    pass = false;
  if (std::abs(i_v1 - (-0.010)) > 1e-6)
    pass = false;

  if (pass) {
    std::cout << "\nSUCCESS: MNA signs and values verified." << std::endl;
    return 0;
  } else {
    std::cout << "\nFAILURE: Value mismatch." << std::endl;
    return 1;
  }
}
