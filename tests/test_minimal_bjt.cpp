#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "connectivity_audit.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cassert>
#include <iostream>
#include <vector>

using namespace std;

int main() {
  cout << "--- Minimal BJT Test Case ---" << endl;
  Circuit c;

  // Standard NPN params
  BjtParams p;
  p.Is = 1e-14;
  p.betaF = 100;
  p.betaR = 1;
  p.nVt = 0.02585;

  NodeIndex vcc = c.createNode("VCC");
  NodeIndex base = c.createNode("BASE");
  NodeIndex out = c.createNode("OUT");

  // VCC = 10V
  c.addElement<VoltageSource>("VccSource", vcc, GND, 10.0);
  // Base Bias = 0.8V
  c.addElement<VoltageSource>("VbSource", base, GND, 0.8);

  // Collector Resistor = 1k
  c.addElement<Resistor>("Rc", vcc, out, 1000.0);

  // Q1: Collector=out, Base=base, Emitter=GND
  addBjtExtended(c, out, base, GND, p, false, "Q1");

  c.finalize();

  // Run Audit
  if (!ConnectivityAudit::run(c)) {
    cerr << "Audit FAILED: Floating nodes in minimal test!" << endl;
    // We expect success now that 'out' is connected.
    return 1;
  }

  cout << "Starting DC solve..." << endl;
  vector<double> x;
  bool converged = c.solveDc(x, 100, 1e-6);

  if (converged) {
    cout << "SUCCESS: Minimal BJT converged." << endl;
    cout << "V(VCC) = " << x[vcc] << " V" << endl;
    cout << "V(BASE) = " << x[base] << " V" << endl;
    cout << "V(OUT) = " << x[out] << " V" << endl;

    // Simple check: V(OUT) should be VCC - Ic*Rc.
    // If BJT is on, V(OUT) should be lower than 10V.
    if (x[out] < 9.9) {
      cout << "BJT is conducting as expected." << endl;
    } else {
      cout << "BJT is NOT conducting (V(OUT) ~ VCC)." << endl;
    }
  } else {
    cerr << "FAILURE: Minimal BJT did not converge." << endl;
    return 1;
  }

  return 0;
}
