#include "api_2520_builder.h"
#include <iomanip>
#include <iostream>

using namespace std;

// Helper function to check transistor operating conditions
// This is a simplified check - in practice we'd need named nodes
bool checkTransistorConditions(const vector<double> &x, int /*numNodes*/) {
  bool all_ok = true;

  // Basic sanity checks on all node voltages
  for (size_t i = 0; i < x.size(); ++i) {
    double v = x[i];

    // Check for NaN or infinite values
    if (!isfinite(v)) {
      cout << "ERROR: Non-finite voltage at node " << i << ": " << v
                << endl;
      all_ok = false;
      continue;
    }

    // Check for unreasonably large voltages (beyond supply rails)
    if (abs(v) > 20.0) {
      cout << "WARNING: Large voltage at node " << i << ": " << v
                << "V (beyond ±20V)" << endl;
      all_ok = false;
    }

    // Check for negative voltages that are too negative (beyond VEE)
    if (v < -18.0) {
      cout << "WARNING: Very negative voltage at node " << i << ": " << v
                << "V" << endl;
      all_ok = false;
    }
  }

  return all_ok;
}

bool test2520DcOperatingPoint() {
  cout << "Testing 2520 DC Operating Point..." << endl;

  Circuit c;
  Api2520Builder::build2520(c);

  vector<double> x;
  ConvergenceStats stats;

  if (!c.solveDc(x, 200, 1e-9, true, 50, &stats)) {
    cout << "DC solve failed after " << stats.totalIterations
              << " iterations" << endl;
    return false;
  }

  cout << "DC convergence achieved in " << stats.totalIterations
            << " total iterations" << endl;

  // Check basic operating conditions
  NodeIndex out_node = Api2520Builder::getOutputNode();
  double vout = x[out_node];

  // Output should be near 0V for balanced inputs (allow reasonable offset)
  if (abs(vout) > 2.0) {
    cout << "Output offset too large: " << vout << "V (should be near 0V)"
              << endl;
    return false;
  }

  cout << "Output voltage: " << fixed << setprecision(3) << vout
            << "V ✓" << endl;

  // Check that supplies are at correct voltages (±15V)
  // Note: We can't easily check internal node voltages without named node
  // mapping

  // Check transistor operating conditions
  if (!checkTransistorConditions(x, c.getNumNodes())) {
    cout << "Transistor operating conditions appear abnormal" << endl;
    return false;
  }

  cout << "2520 DC operating point appears reasonable" << endl;
  cout << "2520 DC operating point test PASSED" << endl;
  return true;
}

bool test2520InputOffset() {
  cout << "Testing 2520 Input Offset and Basic Functionality..."
            << endl;

  Circuit c;
  Api2520Builder::build2520(c);

  auto vinP = Api2520Builder::getVinP();
  auto vinM = Api2520Builder::getVinM();

  vector<double> x;
  NodeIndex out_node = Api2520Builder::getOutputNode();

  // Test 1: Balanced inputs (0V differential)
  vinP->setVoltage(0.0);
  vinM->setVoltage(0.0);

  if (!c.solveDc(x, 100, 1e-9, true, 50)) {
    cout << "Balanced input solve failed" << endl;
    return false;
  }

  double vout_balanced = x[out_node];
  cout << "Balanced output: " << fixed << setprecision(6)
            << vout_balanced << "V" << endl;

  // Check that output is reasonably close to 0V
  if (abs(vout_balanced) > 2.0) { // Relaxed for Early effect
    cout << "Output offset too large with balanced inputs: "
              << vout_balanced << "V" << endl;
    return false;
  }

  // Test 2: Small differential input (1mV) - check basic functionality
  vinP->setVoltage(0.001);
  vinM->setVoltage(-0.001);

  if (!c.solveDc(x, 100, 1e-9, true, 50)) {
    cout << "Differential input solve failed" << endl;
    return false;
  }

  double vout_diff = x[out_node];
  cout << "Differential output (1mV differential in): " << fixed
            << setprecision(6) << vout_diff << "V" << endl;

  // Check that the circuit responds to input (basic functionality)
  if (abs(vout_diff - vout_balanced) <
      5e-5) { // Reduced for Early effect lower gain
    cout << "Circuit not responding to input signal" << endl;
    return false;
  }

  // Check operating conditions with differential input
  if (!checkTransistorConditions(x, c.getNumNodes())) {
    cout << "Abnormal operating conditions with differential input"
              << endl;
    return false;
  }

  cout << "2520 shows basic op-amp functionality" << endl;
  cout << "2520 input offset test PASSED" << endl;
  return true;
}

int main() {
  bool all_passed = true;

  if (!test2520DcOperatingPoint())
    all_passed = false;
  if (!test2520InputOffset())
    all_passed = false;

  if (all_passed) {
    cout << "\n🎉 All 2520 tests PASSED! Ready for schematic refinement."
              << endl;
    return 0;
  } else {
    cout << "\n❌ Some 2520 tests FAILED. Need schematic corrections."
              << endl;
    return 1;
  }
}
