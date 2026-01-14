#include "api_2520_builder.h"
#include "circuit.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double getUnknownValue(const Circuit &c, const std::vector<double> &x,
                       const std::string &name) {
  for (int i = 0; i < (int)x.size(); ++i) {
    if (c.getUnknownMeaning(i) == name) {
      return x[i];
    }
  }
  return 0.0;
}

double getNodeVoltage(const Circuit &c, const std::vector<double> &x,
                      const std::string &name) {
  for (const auto &pair : c.getNodeNames()) {
    if (pair.second == name) {
      return x[pair.first];
    }
  }
  return 0.0;
}

void printDcReport(const Circuit &c, const std::vector<double> &x,
                   bool converged) {
  std::cout << "\n============================================\n";
  std::cout << (converged ? "      API 2520 DC OPERATING POINT REPORT      "
                          : "      API 2520 LAST GUESS (FAILURE) REPORT    ")
            << "\n";
  std::cout << "============================================\n";

  std::vector<std::string> keyNodes = {
      "VCC",     "VEE",      "INP",     "INM",     "OUT",    "Diff_Tail",
      "VAS_Col", "VAS_High", "VAS_Low", "Q7_Emit", "Q8_Emit"};
  for (const auto &name : keyNodes) {
    std::cout << std::left << std::setw(15) << name << " | " << std::fixed
              << std::setprecision(4) << std::right << std::setw(10)
              << getNodeVoltage(c, x, name) << "\n";
  }
}

void runTransientAnalysis(double supplyV) {
  std::cout << "\nStarting Transient Analysis (1kHz Sine, +/-" << supplyV
            << "V)" << std::endl;
  Circuit c;
  Api2520Builder::build2520(c, supplyV);
  c.finalize();

  std::vector<double> x;
  if (!c.solveDc(x, 400, 1e-6, false, 20)) {
    std::cout << "Transient failed: DC pre-solve did not converge."
              << std::endl;
    return;
  }
  c.initializeDynamics(x);

  VoltageSource *vinP = Api2520Builder::getVinP();
  double dt = 5.0e-6;
  double totalTime = 4.0e-3;
  double freq = 1000.0;
  double amp = 0.5; // 0.5V peak input (1V peak output)

  std::cout << "TIME(ms) | VIN(V)  | VOUT(V) | GAIN\n";
  std::cout << "---------|---------|---------|------\n";

  for (double t = 0; t <= totalTime; t += dt) {
    double vIn = amp * std::sin(2.0 * M_PI * freq * t);
    vinP->setVoltage(vIn);
    if (!c.step(dt, x, 100, 1e-6, 1e-9)) {
      std::cout << "!!! Transient Convergence Failure at t=" << t * 1000.0
                << "ms !!!\n";
      printDcReport(c, x, false);
      break;
    }

    double vOut = getNodeVoltage(c, x, "OUT");
    double gain = (std::abs(vIn) > 0.02) ? (vOut / vIn) : 2.0;

    if (int(t / dt + 0.5) % 10 == 0) {
      std::cout << std::fixed << std::setprecision(3) << std::setw(8)
                << t * 1000.0 << " | " << std::setw(7) << vIn << " | "
                << std::setw(7) << vOut << " | " << std::setprecision(2)
                << std::setw(4) << gain << std::endl;
    }
  }
}

int main() {
  runTransientAnalysis(15.0);
  return 0;
}
