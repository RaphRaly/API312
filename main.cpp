#include "api_2520_builder.h"
#include "circuit.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double getUnknownValue(const Circuit &c, const vector<double> &x,
                       const string &name) {
  for (int i = 0; i < (int)x.size(); ++i) {
    if (c.getUnknownMeaning(i) == name) {
      return x[i];
    }
  }
  return 0.0;
}

double getNodeVoltage(const Circuit &c, const vector<double> &x,
                      const string &name) {
  for (const auto &pair : c.getNodeNames()) {
    if (pair.second == name) {
      return x[pair.first];
    }
  }
  return 0.0;
}

void printDcReport(const Circuit &c, const vector<double> &x,
                   bool converged) {
  cout << "\n============================================\n";
  cout << (converged ? "      API 2520 DC OPERATING POINT REPORT      "
                          : "      API 2520 LAST GUESS (FAILURE) REPORT    ")
            << "\n";
  cout << "============================================\n";

  vector<string> keyNodes = {
      "VCC",     "VEE",      "INP",     "INM",     "OUT",    "Diff_Tail",
      "VAS_Col", "VAS_High", "VAS_Low", "Q7_Emit", "Q8_Emit"};
  for (const auto &name : keyNodes) {
    cout << left << setw(15) << name << " | " << fixed
              << setprecision(4) << right << setw(10)
              << getNodeVoltage(c, x, name) << "\n";
  }
}

void runTransientAnalysis(double supplyV) {
  cout << "\nStarting Transient Analysis (1kHz Sine, +/-" << supplyV
            << "V)" << endl;
  Circuit c;
  Api2520Builder::build2520(c, supplyV);
  c.finalize();

  vector<double> x;
  if (!c.solveDc(x, 400, 1e-6, false, 20)) {
    cout << "Transient failed: DC pre-solve did not converge."
              << endl;
    return;
  }
  c.initializeDynamics(x);

  VoltageSource *vinP = Api2520Builder::getVinP();
  double dt = 5.0e-6;
  double totalTime = 4.0e-3;
  double freq = 1000.0;
  double amp = 0.5; // 0.5V peak input (1V peak output)

  cout << "TIME(ms) | VIN(V)  | VOUT(V) | GAIN\n";
  cout << "---------|---------|---------|------\n";

  for (double t = 0; t <= totalTime; t += dt) {
    double vIn = amp * sin(2.0 * M_PI * freq * t);
    vinP->setVoltage(vIn);
    if (!c.step(dt, x, 100, 1e-6, 1e-9)) {
      cout << "!!! Transient Convergence Failure at t=" << t * 1000.0
                << "ms !!!\n";
      printDcReport(c, x, false);
      break;
    }

    double vOut = getNodeVoltage(c, x, "OUT");
    double gain = (abs(vIn) > 0.02) ? (vOut / vIn) : 2.0;

    if (int(t / dt + 0.5) % 10 == 0) {
      cout << fixed << setprecision(3) << setw(8)
                << t * 1000.0 << " | " << setw(7) << vIn << " | "
                << setw(7) << vOut << " | " << setprecision(2)
                << setw(4) << gain << endl;
    }
  }
}

int main() {
  runTransientAnalysis(15.0);
  return 0;
}
