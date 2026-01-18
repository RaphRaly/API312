#include "api_2520_builder.h"
#include "circuit.h"
#include "plausibility.h"
#include <iostream>
#include <vector>

using namespace std;

int main() {
  cout << "API 2520 Supply Robustness Sweep\n";
  cout << "Sweeping from +/-12V to +/-18V...\n";

  bool allStepsPass = true;
  vector<double> x; // For continuation

  for (double v = 12.0; v <= 18.01; v += 1.0) {
    cout << "\nTesting Supply: +/-" << v << "V\n";
    Circuit c;
    Api2520Builder::build2520(c, v);
    c.finalize();

    // Use pseudo-transient continuation for robust DC solve each step
    // disableSourceStepping=true to solve purely by transient settling
    // Don't use warm-start for PT because previous DC point might be in a
    // different basin
    bool converged = c.solveDcPseudoTransient(x, 1e-3, 1e-6, true);

    if (!converged) {
      cout << "FATAL: DC Convergence Failure at +/-" << v << "V\n";
      allStepsPass = false;
      break;
    }

    PlausibilityReport report = PlausibilityChecker::check(c, x, v);
    PlausibilityChecker::printReport(report);

    if (!report.overallPass) {
      cout << "FAIL: Plausibility checks failed at +/-" << v << "V\n";
      allStepsPass = false;
    }
  }

  if (allStepsPass) {
    cout << "\n[SUCCESS] Entire supply sweep passed.\n";
    return 0;
  } else {
    cout << "\n[FAILURE] Robustness sweep failed.\n";
    return 1;
  }
}
