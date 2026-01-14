#include "api_2520_builder.h"
#include "circuit.h"
#include "connectivity_audit.h"
#include <iostream>

int main() {
  std::cout << "--- 2520 Topology Connectivity Audit ---" << std::endl;
  Circuit c;
  try {
    Api2520Builder::build2520(c);
    c.finalize();

    bool success = ConnectivityAudit::run(c);

    if (success) {
      std::cout << "\nSUCCESS: 2520 circuit has no floating nodes."
                << std::endl;
      return 0;
    } else {
      std::cout << "\nFAILURE: Floating nodes detected in 2520 circuit!"
                << std::endl;
      return 1;
    }
  } catch (const std::exception &e) {
    std::cerr << "EXCEPTION during build/audit: " << e.what() << std::endl;
    return 1;
  }
}
