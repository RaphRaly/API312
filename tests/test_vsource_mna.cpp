#include "circuit.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

int main() {
  cout << "--- MNA Voltage Source Sign Test ---" << endl;

  Circuit c;
  NodeIndex n1 = c.createNode("N1");

  // V1: 10V between N1 and GND
  // Convention: VoltageSource(pos, neg, value) => V(pos) - V(neg) = value
  c.addElement<VoltageSource>("V1", n1, GND, 10.0);

  // R1: 1000 ohms between N1 and GND
  c.addElement<Resistor>("R1", n1, GND, 1000.0);

  c.finalize();

  vector<double> x;
  bool ok = c.solveDc(x, 2, 1e-9, false);

  if (!ok) {
    cerr << "Solver failed to converge on linear circuit!" << endl;
    return 1;
  }

  cout << "\nUnknown Map:" << endl;
  for (int i = 0; i < (int)x.size(); ++i) {
    cout << "Index [" << i << "]: " << left << setw(15)
              << c.getUnknownMeaning(i) << " = " << x[i] << endl;
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

  cout << "\nResults:" << endl;
  cout << "V(N1)  = " << v_n1 << " V (Expected: 10.0)" << endl;
  cout << "I(V1)  = " << i_v1 * 1000.0 << " mA (Expected: -10.0)"
            << endl;

  bool pass = true;
  if (abs(v_n1 - 10.0) > 1e-6)
    pass = false;
  if (abs(i_v1 - (-0.010)) > 1e-6)
    pass = false;

  if (pass) {
    cout << "\nSUCCESS: MNA signs and values verified." << endl;
    return 0;
  } else {
    cout << "\nFAILURE: Value mismatch." << endl;
    return 1;
  }
}
