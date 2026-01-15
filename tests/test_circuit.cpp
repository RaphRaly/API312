#include "capacitor_trap.h"
#include "circuit.h"
#include "current_source.h"
#include "diode_shockley_nr.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;

// Test utilities
#define TEST_ASSERT(cond, msg)                                                 \
  if (!(cond)) {                                                               \
    cerr << "TEST FAILED: " << msg << endl;                          \
    return false;                                                              \
  }

bool test_resistor_voltage_divider() {
  Circuit c;

  NodeIndex n1 = c.createNode();
  NodeIndex n2 = c.createNode();

  c.addElement<VoltageSource>(n1, GND, 10.0);
  c.addElement<Resistor>(n1, n2, 1000.0);
  c.addElement<Resistor>(n2, GND, 1000.0);

  c.finalize();
  vector<double> x;
  bool ok = c.step(1.0, x, 10, 1e-6, 1e-9);

  TEST_ASSERT(ok, "Convergence failed");
  TEST_ASSERT(abs(x[n2] - 5.0) < 1e-6, "Wrong voltage divider result");

  return true;
}

bool test_capacitor_dc() {
  Circuit c;

  NodeIndex n1 = c.createNode();
  c.addElement<VoltageSource>(n1, GND, 5.0);
  c.addElement<CapacitorTrap>(n1, GND, 1e-6); // 1uF

  c.finalize();
  vector<double> x;
  bool ok = c.step(1e-3, x, 10, 1e-6, 1e-9); // 1ms step

  TEST_ASSERT(ok, "DC convergence failed");
  TEST_ASSERT(abs(x[n1] - 5.0) < 1e-6,
              "Capacitor should charge to source voltage in DC");

  return true;
}

bool test_diode_forward_bias() {
  Circuit c;

  NodeIndex n1 = c.createNode();
  NodeIndex n2 = c.createNode();

  c.addElement<VoltageSource>(n1, GND, 1.0); // Forward bias
  c.addElement<Resistor>(n1, n2, 1000.0);
  c.addElement<DiodeShockleyNR>(n2, GND, 1e-15, 1.0, 0.02585);

  c.finalize();
  vector<double> x;
  bool ok = c.step(1.0, x, 20, 1e-6, 1e-9);

  TEST_ASSERT(ok, "Diode convergence failed");
  TEST_ASSERT(x[n2] > 0.6 && x[n2] < 0.8,
              "Diode voltage should be around 0.7V");

  return true;
}

int main() {
  cout << "Running MNA Circuit Tests..." << endl;

  bool all_passed = true;

  if (!test_resistor_voltage_divider())
    all_passed = false;
  if (!test_capacitor_dc())
    all_passed = false;
  if (!test_diode_forward_bias())
    all_passed = false;

  if (all_passed) {
    cout << "All tests PASSED!" << endl;
    return 0;
  } else {
    cout << "Some tests FAILED!" << endl;
    return 1;
  }
}
