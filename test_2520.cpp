#include "api_2520_builder.h"
#include <iomanip>
#include <iostream>

// Helper function to check transistor operating conditions
// This is a simplified check - in practice we'd need named nodes
bool checkTransistorConditions(const std::vector<double> &x, int /*numNodes*/) {
  bool all_ok = true;

  // Basic sanity checks on all node voltages
  for (size_t i = 0; i < x.size(); ++i) {
    double v = x[i];

    // Check for NaN or infinite values
    if (!std::isfinite(v)) {
      std::cout << "ERROR: Non-finite voltage at node " << i << ": " << v
                << std::endl;
      all_ok = false;
      continue;
    }

    // Check for unreasonably large voltages (beyond supply rails)
    if (std::abs(v) > 20.0) {
      std::cout << "WARNING: Large voltage at node " << i << ": " << v
                << "V (beyond ±20V)" << std::endl;
      all_ok = false;
    }

    // Check for negative voltages that are too negative (beyond VEE)
    if (v < -18.0) {
      std::cout << "WARNING: Very negative voltage at node " << i << ": " << v
                << "V" << std::endl;
      all_ok = false;
    }
  }

  return all_ok;
}

bool test2520DcOperatingPoint() {
  std::cout << "Testing 2520 DC Operating Point..." << std::endl;

  Circuit c;
  Api2520Builder::build2520(c);

  std::vector<double> x;
  ConvergenceStats stats;

  if (!c.solveDc(x, 200, 1e-9, true, 50, &stats)) {
    std::cout << "DC solve failed after " << stats.totalIterations
              << " iterations" << std::endl;
    return false;
  }

  std::cout << "DC convergence achieved in " << stats.totalIterations
            << " total iterations" << std::endl;

  // Check basic operating conditions
  NodeIndex out_node = Api2520Builder::getOutputNode();
  double vout = x[out_node];

  // Output should be near 0V for balanced inputs (allow reasonable offset)
  if (std::abs(vout) > 2.0) {
    std::cout << "Output offset too large: " << vout << "V (should be near 0V)"
              << std::endl;
    return false;
  }

  std::cout << "Output voltage: " << std::fixed << std::setprecision(3) << vout
            << "V ✓" << std::endl;

  // Check that supplies are at correct voltages (±15V)
  // Note: We can't easily check internal node voltages without named node
  // mapping

  // Check transistor operating conditions
  if (!checkTransistorConditions(x, c.getNumNodes())) {
    std::cout << "Transistor operating conditions appear abnormal" << std::endl;
    return false;
  }

  std::cout << "2520 DC operating point appears reasonable" << std::endl;
  std::cout << "2520 DC operating point test PASSED" << std::endl;
  return true;
}

bool test2520InputOffset() {
  std::cout << "Testing 2520 Input Offset and Basic Functionality..."
            << std::endl;

  Circuit c;
  Api2520Builder::build2520(c);

  auto vinP = Api2520Builder::getVinP();
  auto vinM = Api2520Builder::getVinM();

  std::vector<double> x;
  NodeIndex out_node = Api2520Builder::getOutputNode();

  // Test 1: Balanced inputs (0V differential)
  vinP->setVoltage(0.0);
  vinM->setVoltage(0.0);

  if (!c.solveDc(x, 100, 1e-9, true, 50)) {
    std::cout << "Balanced input solve failed" << std::endl;
    return false;
  }

  double vout_balanced = x[out_node];
  std::cout << "Balanced output: " << std::fixed << std::setprecision(6)
            << vout_balanced << "V" << std::endl;

  // Check that output is reasonably close to 0V
  if (std::abs(vout_balanced) > 1.0) {
    std::cout << "Output offset too large with balanced inputs: "
              << vout_balanced << "V" << std::endl;
    return false;
  }

  // Test 2: Small differential input (1mV) - check basic functionality
  vinP->setVoltage(0.001);
  vinM->setVoltage(-0.001);

  if (!c.solveDc(x, 100, 1e-9, true, 50)) {
    std::cout << "Differential input solve failed" << std::endl;
    return false;
  }

  double vout_diff = x[out_node];
  std::cout << "Differential output (1mV differential in): " << std::fixed
            << std::setprecision(6) << vout_diff << "V" << std::endl;

  // Check that the circuit responds to input (basic functionality)
  if (std::abs(vout_diff - vout_balanced) <
      0.001) { // At least 1mV change for 2mV input
    std::cout << "Circuit not responding to input signal" << std::endl;
    return false;
  }

  // Check operating conditions with differential input
  if (!checkTransistorConditions(x, c.getNumNodes())) {
    std::cout << "Abnormal operating conditions with differential input"
              << std::endl;
    return false;
  }

  std::cout << "2520 shows basic op-amp functionality" << std::endl;
  std::cout << "2520 input offset test PASSED" << std::endl;
  return true;
}

int main() {
  bool all_passed = true;

  if (!test2520DcOperatingPoint())
    all_passed = false;
  if (!test2520InputOffset())
    all_passed = false;

  if (all_passed) {
    std::cout << "\n🎉 All 2520 tests PASSED! Ready for schematic refinement."
              << std::endl;
    return 0;
  } else {
    std::cout << "\n❌ Some 2520 tests FAILED. Need schematic corrections."
              << std::endl;
    return 1;
  }
}
