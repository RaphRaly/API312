#include "api_2520_builder.h"
#include "circuit.h"
#include <chrono>
#include <iostream>
#include <vector>

using namespace std;

int main() {
  cout << "API 2520 Micro-Benchmark\n";
  cout << "Running 1000 transient steps...\n";

  Circuit c;
  Api2520Builder::build2520(c, 15.0);
  c.finalize();

  vector<double> x;
  if (!c.solveDc(x, 400, 1e-6, false, 50)) {
    cerr << "DC solve failed, cannot benchmark.\n";
    return 1;
  }
  c.initializeDynamics(x);

  auto start = chrono::high_resolution_clock::now();

  int steps = 1000;
  double dt = 2.0e-6;

  for (int i = 0; i < steps; ++i) {
    // Run with fixed iter count for benchmark consistency
    if (!c.step(dt, x, 8, 1e-6, 1e-9)) {
      // We don't break on failure for benchmark unless it's catastrophic
    }
  }

  auto end = chrono::high_resolution_clock::now();
  chrono::duration<double> diff = end - start;

  cout << "\n--- RESULTS ---\n";
  cout << "Total Time: " << diff.count() << " s\n";
  cout << "Throughput: " << (steps / diff.count()) << " steps/sec\n";
  cout << "Performance: " << (diff.count() / steps * 1000000.0)
            << " us/step\n";

  return 0;
}
