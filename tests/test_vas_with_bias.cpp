#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// Test VAS with bias chain to see if it fixes the 54V issue
// The key is R_VAS_HI (200Ω) which provides alternate current path

int main() {
    cout << "=== STAGE 3b: VAS + Bias Chain (with R_VAS_HI) ===" << endl;
    
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    NodeIndex nINP = c.createNode("INP");
    NodeIndex nINM = c.createNode("INM");
    NodeIndex nTAIL = c.createNode("TAIL");
    NodeIndex nC1 = c.createNode("C1");
    NodeIndex nC2 = c.createNode("C2");
    NodeIndex nE1 = c.createNode("E1");
    NodeIndex nE2 = c.createNode("E2");
    NodeIndex nE3 = c.createNode("E3");
    NodeIndex nE9 = c.createNode("E9");
    NodeIndex nE4 = c.createNode("E4");
    NodeIndex nC4 = c.createNode("C4");
    NodeIndex nVAS = c.createNode("VAS");
    NodeIndex nE5 = c.createNode("E5");
    NodeIndex nB_HI = c.createNode("B_HI");
    NodeIndex nB_LO = c.createNode("B_LO");
    NodeIndex nB_Mult = c.createNode("B_Mult");
    
    c.addElement<VoltageSource>("VinP", nINP, GND, 0.0);
    c.addElement<VoltageSource>("VinM", nINM, GND, 0.0);
    
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    
    // Mirror
    addBjtExtended(c, nC1, nC1, nE3, q_pnp, true, "Q3");
    c.addElement<Resistor>("R_Deg3", nVCC, nE3, 100.0);
    addBjtExtended(c, nC2, nC1, nE9, q_pnp, true, "Q9");
    c.addElement<Resistor>("R_Deg9", nVCC, nE9, 100.0);
    
    // Diff pair
    addBjtExtended(c, nC1, nINP, nE1, q_npn, false, "Q1");
    addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2");
    c.addElement<Resistor>("R_E1", nE1, nTAIL, 100.0);
    c.addElement<Resistor>("R_E2", nE2, nTAIL, 100.0);
    c.addElement<CurrentSource>("I_Tail", nTAIL, nVEE, 200.0e-6);
    
    // Q4 VAS driver
    addBjtExtended(c, nC4, nC2, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    // Q5 VAS stage
    addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    
    // === THE KEY ADDITION: R_VAS_HI + Bias Chain ===
    c.addElement<Resistor>("R_VAS_HI", nVAS, nB_HI, 200.0);  // This provides current path!
    
    // VBE multiplier
    addBjtExtended(c, nB_HI, nB_Mult, nB_LO, q_vas, false, "Q_Bias");
    c.addElement<Resistor>("R1_Mult", nB_HI, nB_Mult, 1800.0);
    c.addElement<Resistor>("R2_Mult", nB_Mult, nB_LO, 1000.0);
    
    // Bias chain current sink
    c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 0.5e-3);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    
    cout << "Converged: " << (conv ? "YES" : "NO") << ", Final Gmin: " << c.getFinalGmin() << endl;
    
    cout << "\n--- Key Voltages ---" << endl;
    const auto& nodeNames = c.getNodeNames();
    for (const auto& [idx, name] : nodeNames) {
        double v = x[idx];
        cout << "  V(" << setw(10) << left << name << ") = " << setw(10) << right << fixed << setprecision(4) << v << " V";
        if (abs(v) > 15.1) cout << " *** OUTSIDE RAILS! ***";
        cout << endl;
    }
    
    double v_vas = x[nVAS];
    cout << "\n*** V(VAS) = " << v_vas << " V ***" << endl;
    if (abs(v_vas) < 15.0) {
        cout << "SUCCESS! With R_VAS_HI + bias chain, VAS stays within rails." << endl;
        cout << "The 54V in Stage 3 was because I_VAS excess had no path." << endl;
    } else {
        cout << "STILL OUTSIDE RAILS - issue is elsewhere." << endl;
    }
    
    return 0;
}
