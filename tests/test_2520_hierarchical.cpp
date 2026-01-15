#include "circuit.h"
#include "bjt_ebers_moll.h"
#include "spice_models_2520.h"
#include "resistor.h"
#include "current_source.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// Print node voltages and check if any are outside rails
bool printAndCheck(Circuit& c, const vector<double>& x, double railPlus, double railMinus) {
    const auto& nodeNames = c.getNodeNames();
    bool allOk = true;
    
    for (const auto& [idx, name] : nodeNames) {
        double v = x[idx];
        cout << "  V(" << setw(12) << left << name << ") = " << setw(10) << right << fixed << setprecision(4) << v << " V";
        if (v > railPlus + 0.1 || v < railMinus - 0.1) {
            cout << " *** OUTSIDE RAILS! ***";
            allOk = false;
        }
        cout << endl;
    }
    return allOk;
}

// STAGE 1: Diff pair + mirror + tail only
bool test_stage1() {
    cout << "\n========================================" << endl;
    cout << "STAGE 1: Diff Pair + Mirror + Tail" << endl;
    cout << "========================================" << endl;
    
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
    
    // Inputs at 0V
    c.addElement<VoltageSource>("VinP", nINP, GND, 0.0);
    c.addElement<VoltageSource>("VinM", nINM, GND, 0.0);
    
    // Mirror (PNP)
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    addBjtExtended(c, nC1, nC1, nE3, q_pnp, true, "Q3"); // Master
    c.addElement<Resistor>("R_Deg3", nVCC, nE3, 100.0);
    addBjtExtended(c, nC2, nC1, nE9, q_pnp, true, "Q9"); // Slave
    c.addElement<Resistor>("R_Deg9", nVCC, nE9, 100.0);
    
    // Diff pair (NPN)
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    addBjtExtended(c, nC1, nINP, nE1, q_npn, false, "Q1");
    addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2");
    c.addElement<Resistor>("R_E1", nE1, nTAIL, 100.0);
    c.addElement<Resistor>("R_E2", nE2, nTAIL, 100.0);
    
    // Tail current
    c.addElement<CurrentSource>("I_Tail", nTAIL, nVEE, 200.0e-6);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    cout << "Converged: " << (conv ? "YES" : "NO") << ", Final Gmin: " << c.getFinalGmin() << endl;
    
    if (conv) {
        return printAndCheck(c, x, 15.0, -15.0);
    }
    return false;
}

// STAGE 2: Add VAS driver (Q4)
bool test_stage2() {
    cout << "\n========================================" << endl;
    cout << "STAGE 2: + Q4 (VAS Driver)" << endl;
    cout << "========================================" << endl;
    
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
    
    c.addElement<VoltageSource>("VinP", nINP, GND, 0.0);
    c.addElement<VoltageSource>("VinM", nINM, GND, 0.0);
    
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    
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
    
    // Q4 VAS driver (PNP) - base from C2, collector to C4
    addBjtExtended(c, nC4, nC2, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    cout << "Converged: " << (conv ? "YES" : "NO") << ", Final Gmin: " << c.getFinalGmin() << endl;
    
    if (conv) {
        return printAndCheck(c, x, 15.0, -15.0);
    }
    return false;
}

// STAGE 3: Add VAS (Q5 + I_VAS)
bool test_stage3() {
    cout << "\n========================================" << endl;
    cout << "STAGE 3: + Q5 + I_VAS (VAS Stage)" << endl;
    cout << "========================================" << endl;
    
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
    
    vector<double> x;
    bool conv = c.solveDc(x);
    cout << "Converged: " << (conv ? "YES" : "NO") << ", Final Gmin: " << c.getFinalGmin() << endl;
    
    if (conv) {
        return printAndCheck(c, x, 15.0, -15.0);
    }
    return false;
}

int main() {
    cout << "========================================" << endl;
    cout << "  HIERARCHICAL 2520 BLOCK TESTS" << endl;
    cout << "========================================" << endl;
    cout << "Testing incrementally to find where solution goes wrong" << endl;
    
    bool s1 = test_stage1();
    bool s2 = test_stage2();
    bool s3 = test_stage3();
    
    cout << "\n========================================" << endl;
    cout << "SUMMARY:" << endl;
    cout << "  Stage 1 (Diff+Mirror+Tail): " << (s1 ? "OK" : "FAIL") << endl;
    cout << "  Stage 2 (+Q4 VAS Driver): " << (s2 ? "OK" : "FAIL") << endl;
    cout << "  Stage 3 (+Q5 VAS): " << (s3 ? "OK" : "FAIL") << endl;
    cout << "========================================" << endl;
    
    return (s1 && s2 && s3) ? 0 : 1;
}
