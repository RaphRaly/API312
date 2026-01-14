#include "api_2520_builder.h"
#include "circuit.h"
#include <chrono>
#include <iostream>
#include <vector>

int main() {
  std::cout << "API 2520 Micro-Benchmark\n";
  std::cout << "Running 1000 transient steps...\n";

  Circuit c;
  Api2520Builder::build2520(c, 15.0);
  c.finalize();

  std::vector<double> x;
  if (!c.solveDc(x, 400, 1e-6, false, 50)) {
    std::cerr << "DC solve failed, cannot benchmark.\n";
    return 1;
  }
  c.initializeDynamics(x);

  auto start = std::chrono::high_resolution_clock::now();

  int steps = 1000;
  double dt = 2.0e-6;

  for (int i = 0; i < steps; ++i) {
    // Run with fixed iter count for benchmark consistency
    if (!c.step(dt, x, 8, 1e-6, 1e-9)) {
      // We don't break on failure for benchmark unless it's catastrophic
    }
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end - start;

  std::cout << "\n--- RESULTS ---\n";
  std::cout << "Total Time: " << diff.count() << " s\n";
  std::cout << "Throughput: " << (steps / diff.count()) << " steps/sec\n";
  std::cout << "Performance: " << (diff.count() / steps * 1000000.0)
            << " us/step\n";

  return 0;
}
