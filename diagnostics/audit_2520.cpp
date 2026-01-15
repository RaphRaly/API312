#include "api_2520_builder.h"
#include "circuit.h"
#include "connectivity_audit.h"
#include <iostream>

using namespace std;

int main() {
  cout << "--- 2520 Topology Connectivity Audit ---" << endl;
  Circuit c;
  try {
    Api2520Builder::build2520(c);
    c.finalize();

    bool success = ConnectivityAudit::run(c);

    if (success) {
      cout << "\nSUCCESS: 2520 circuit has no floating nodes."
                << endl;
      return 0;
    } else {
      cout << "\nFAILURE: Floating nodes detected in 2520 circuit!"
                << endl;
      return 1;
    }
  } catch (const exception &e) {
    cerr << "EXCEPTION during build/audit: " << e.what() << endl;
    return 1;
  }
}
