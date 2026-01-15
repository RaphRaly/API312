#include "circuit.h"
#include "resistor.h"
#include "current_source.h"
#include "voltage_source.h"
#include "bjt_ebers_moll.h"
#include "spice_models_2520.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

// User Requested Micro-Tests to rule out regressions/bugs

bool testResistorSanity() {
    cout << "\n--- Test 1: Resistor Sanity ---" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nX = c.createNode("X");
    
    // R=10k from X to GND
    c.addElement<Resistor>("R1", nX, GND, 10000.0);
    // I=1mA from GND to X (Source into X)
    // CurrentSource(a,b,I) flows a->b. So we want GND->X.
    c.addElement<CurrentSource>("I1", GND, nX, 1.0e-3);
    
    vector<double> x;
    c.solveDc(x);
    
    double vx = x[nX];
    cout << "V(X) = " << vx << " V (Expected: 10.0V)" << endl;
    
    if (abs(vx - 10.0) < 1e-6) {
        cout << "PASS" << endl;
        return true;
    }
    cout << "FAIL" << endl;
    return false;
}

bool testFeedbackNode() {
    cout << "\n--- Test 2: Feedback Node MNA ---" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nOUT = c.createNode("OUT");
    NodeIndex nINM = c.createNode("INM");
    NodeIndex nREF = c.createNode("INM_Ref");
    
    // OUT fixed at 0V
    c.addElement<VoltageSource>("V_OUT", nOUT, GND, 0.0);
    // REF fixed at 0V
    c.addElement<VoltageSource>("V_REF", nREF, GND, 0.0);
    
    // Feedback Network
    c.addElement<Resistor>("R_Feedback", nOUT, nINM, 10000.0);
    c.addElement<Resistor>("R_Input", nINM, nREF, 10000.0);
    
    // INM should be 0V (pulled by 10k to 0V and 10k to 0V)
    // To prove it's connected, inject small current? 
    // Or just checking if it floats or solves to 0.
    
    vector<double> x;
    c.solveDc(x);
    
    double v_inm = x[nINM];
    cout << "V(INM) = " << v_inm << " V (Expected: 0.0V / near machine precision)" << endl;
    
    if (abs(v_inm) < 1e-9) {
        cout << "PASS" << endl;
        return true;
    }
    cout << "FAIL" << endl;
    return false;
}

bool testPnpSanity() {
    cout << "\n--- Test 3: Q4 PNP Sanity ---" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nE = c.createNode("E");
    NodeIndex nB = c.createNode("B");
    NodeIndex nC = c.createNode("C");
    NodeIndex nVCC = c.createNode("VCC");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    
    // PNP Setup: E at 15V. B at 14.3V (Vbe ~ -0.7). C at 0V (Load).
    c.addElement<VoltageSource>("VE", nE, GND, 15.0);
    c.addElement<VoltageSource>("VB", nB, GND, 14.3);
    c.addElement<Resistor>("RC", nC, GND, 1000.0);
    
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C")); // PNP
    // addBjtExtended params: C, B, E
    addBjtExtended(c, nC, nB, nE, q_pnp, true, "Q4");
    
    vector<double> x;
    c.solveDc(x);
    
    double vc = x[nC];
    double ic = vc / 1000.0;
    
    cout << "V(C) = " << vc << " V" << endl;
    cout << "I(C) = " << ic * 1000.0 << " mA" << endl;
    
    // PNP E->B (15 -> 14.3) is Forward Biased.
    // Current should flow out of Collector (C -> GND).
    // So V(C) should be positive?
    // Wait. PNP C current flows INTO Collector from Emitter? NO.
    // NPN: C -> E.
    // PNP: E -> C.
    // So current flows E -> C.
    // So current LEAVES Collector node.
    // So it flows INTO Resistor (Node C -> GND).
    // So V(C) should be POSITIVE.
    
    if (vc > 0.5 && vc < 15.0) {
        cout << "PASS: PNP conducting, current flows E->C" << endl;
        return true;
    }
    cout << "FAIL: PNP not conducting correctly" << endl;
    return false;
}

int main() {
    bool p1 = testResistorSanity();
    bool p2 = testFeedbackNode();
    bool p3 = testPnpSanity();
    
    if (p1 && p2 && p3) return 0;
    return 1;
}
