#include "api_2520_builder.h"
#include "circuit.h"
#include "plausibility.h"
#include <iostream>
#include <vector>

int main() {
  std::cout << "API 2520 Supply Robustness Sweep\n";
  std::cout << "Sweeping from +/-12V to +/-18V...\n";

  bool allStepsPass = true;
  std::vector<double> x; // For continuation

  for (double v = 12.0; v <= 18.01; v += 1.0) {
    std::cout << "\nTesting Supply: +/-" << v << "V\n";
    Circuit c;
    Api2520Builder::build2520(c, v);
    c.finalize();

    // Use previous x as guess if available
    bool converged = c.solveDc(x, 400, 1e-6, !x.empty(), 50);

    if (!converged) {
      std::cout << "FATAL: DC Convergence Failure at +/-" << v << "V\n";
      allStepsPass = false;
      break;
    }

    PlausibilityReport report = PlausibilityChecker::check(c, x, v);
    PlausibilityChecker::printReport(report);

    if (!report.overallPass) {
      std::cout << "FAIL: Plausibility checks failed at +/-" << v << "V\n";
      allStepsPass = false;
    }
  }

  if (allStepsPass) {
    std::cout << "\n[SUCCESS] Entire supply sweep passed.\n";
    return 0;
  } else {
    std::cout << "\n[FAILURE] Robustness sweep failed.\n";
    return 1;
  }
}
