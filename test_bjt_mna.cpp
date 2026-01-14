#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
  std::cout << "--- BJT NPN MNA Sign & Residual Test ---" << std::endl;

  Circuit c;
  NodeIndex nC = c.createNode("C");
  NodeIndex nB = c.createNode("B");
  NodeIndex nE = c.createNode("E");

  BjtParams p;
  p.Is = 1e-14;
  p.betaF = 100;
  p.betaR = 1;
  p.nVt = 0.02585;
  p.gmin = 1e-12;

  // NPN BJT
  c.addElement<BjtNpnEbersMoll>("Q1", nC, nB, nE, p);

  // Bias: Vc=5, Vb=0.7, Ve=0 => Active mode
  c.addElement<VoltageSource>("Vc", nC, GND, 5.0);
  c.addElement<VoltageSource>("Vb", nB, GND, 0.7);
  c.addElement<VoltageSource>("Ve", nE, GND, 0.0);

  c.finalize();

  std::vector<double> x;
  bool ok = c.solveDc(x, 20, 1e-9, true);

  if (ok) {
    std::cout << "\nSolution converged." << std::endl;

    // Find branch indices
    int iVc = -1, iVb = -1;
    int numNodes = c.getNumNodes();
    // Since we added Vc, Vb, Ve in order, they should be branches 0, 1, 2
    // But let's check unknown meaning
    for (int i = numNodes; i < x.size(); ++i) {
      std::string mean = c.getUnknownMeaning(i);
      if (mean == "I(Vc)")
        iVc = i;
      if (mean == "I(Vb)")
        iVb = i;
    }

    if (iVc != -1 && iVb != -1) {
      double ic_meas =
          -x[iVc]; // Sign convention: branch current flows pos -> neg
      double ib_meas = -x[iVb];

      std::cout << "Ic (from Vc src): " << ic_meas * 1000.0 << " mA"
                << std::endl;
      std::cout << "Ib (from Vb src): " << ib_meas * 1000.0 << " mA"
                << std::endl;
      std::cout << "Beta = " << ic_meas / ib_meas << std::endl;

      // Theoretical Ic = Is * (exp(Vbe/Vt) - exp(Vbc/Vt))
      // Vbe = 0.7, Vbc = 0.7 - 5 = -4.3.
      double ic_theo =
          p.Is * (std::exp(0.7 / 0.02585) - std::exp(-4.3 / 0.02585));
      std::cout << "Ic Theoretical  : " << ic_theo * 1000.0 << " mA"
                << std::endl;
    }
  } else {
    std::cerr << "BJT Solve failed!" << std::endl;
    return 1;
  }
  return 0;
}
