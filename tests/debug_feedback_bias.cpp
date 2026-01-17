#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// User Requested 3 Minimal Tests

// 1. Feedback-only: Verify simple resistive divider behavior
bool testFeedbackOnly() {
    cout << "\n=== Test 1: Feedback Only ===" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nOUT = c.createNode("OUT");
    NodeIndex nINM = c.createNode("INM");
    NodeIndex nREF = c.createNode("INM_Ref");
    
    // Force OUT=0V, REF=0V
    c.addElement<VoltageSource>("V_OUT", nOUT, GND, 0.0);
    c.addElement<VoltageSource>("V_REF", nREF, GND, 0.0);
    
    // R=10k each
    c.addElement<Resistor>("R_F", nOUT, nINM, 10000.0);
    c.addElement<Resistor>("R_I", nINM, nREF, 10000.0);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    
    if (!conv) {
        cout << "FAIL: No convergence." << endl;
        return false;
    }
    
    double v_inm = x[nINM];
    cout << "V(INM) = " << v_inm << " V (Expected ~0.0)" << endl;
    
    if (abs(v_inm) < 1e-6) {
        cout << "PASS: Resistive network OK." << endl;
        return true;
    }
    cout << "FAIL: MNA/Resistor bug." << endl;
    return false;
}

// 2. Q2 Base Parasitics: Add Q2 to the feedback network
bool testQ2BaseParasitics() {
    cout << "\n=== Test 2: Q2 Base Parasitics ===" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nOUT = c.createNode("OUT");
    NodeIndex nINM = c.createNode("INM"); // Connects to Q2 Base
    NodeIndex nREF = c.createNode("INM_Ref");
    NodeIndex nC2 = c.createNode("C2");
    NodeIndex nE2 = c.createNode("E2");
    
    // Force OUT=0V, REF=0V
    c.addElement<VoltageSource>("V_OUT", nOUT, GND, 0.0);
    c.addElement<VoltageSource>("V_REF", nREF, GND, 0.0);
    
    c.addElement<Resistor>("R_F", nOUT, nINM, 10000.0);
    c.addElement<Resistor>("R_I", nINM, nREF, 10000.0);
    
    // Add Q2 (NPN, inputs from spice_models_2520.h)
    // "BC414C" or similar
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    // C, B, E
    // For test, let's float C and E effectively, or bias them?
    // User said "only Q2". But Q2 floating might lead to singular matrix if C/E not connected.
    // Let's Put voltages on C and E to simulate active region or cutoff?
    // Or just leave them floating? No, singular.
    // Let's ground E2 and put VCC on C2 to see if B sucks current.
    NodeIndex nVCC = c.createNode("VCC");
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    
    c.addElement<VoltageSource>("V_E2", nE2, GND, -0.6); // Slightly bias? Or 0.
    c.addElement<Resistor>("R_C2_Load", nC2, nVCC, 10000.0); 
    
    // addBjtExtended: C, B, E
    auto q2nodes = addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2");
    
    vector<double> x;
    c.solveDc(x);
    
    double v_inm = x[nINM];
    double v_q2_bi = x[q2nodes.b_int];
    double rx = (v_inm - v_q2_bi) / (q_npn.RB + 1e-9); // Estimate current?
    
    cout << "V(INM) = " << v_inm << " V" << endl;
    cout << "V(Q2_Bi) = " << v_q2_bi << " V" << endl;
    cout << "RB = " << q_npn.RB << " Ohm" << endl;
    
    if (abs(v_inm) < 1e-3) {
        cout << "PASS: Q2 base does not disturb INM significantly." << endl;
        return true;
    }
    cout << "FAIL: Q2 base injection disturbance." << endl;
    return false;
}

// 3. Output/Bias Loop Stability
bool testBiasLoopMatrix() {
    cout << "\n=== Test 3: Output/Bias Loop Stability ===" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    // Nodes for loop
    NodeIndex nC4 = c.createNode("C4_DriverCol"); // Q4 Col, Input to Q5 Base
    NodeIndex nVAS = c.createNode("VAS");
    NodeIndex nE5 = c.createNode("E5");
    NodeIndex nB_HI = c.createNode("B_HI");
    NodeIndex nB_LO = c.createNode("B_LO");
    NodeIndex nB_Mult = c.createNode("B_Mult");
    NodeIndex nOUT = c.createNode("OUT");
    NodeIndex nE7 = c.createNode("E7");
    NodeIndex nE8 = c.createNode("E8");
    NodeIndex nINM = c.createNode("INM");
    NodeIndex nREF = c.createNode("INM_Ref");
    
    // Force inputs for loop
    // Drive C4 (Base of Q5) directly?
    // Q5 Base is nC4.
    // Let's assume Q4 drives nC4. We replace Q4 with a Voltage Source driving nC4.
    // We want to see if setting nC4 to a "reasonable" value centers the output.
    // Reasonable value: V(E5) ~ -15. V(B5)=V(C4) ~ -14.4V.
    double v_drive = -14.4;
    c.addElement<VoltageSource>("V_Drive", nC4, GND, v_drive);
    
    // Components
    // Q5
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);

    // Bias Chain
    c.addElement<Resistor>("R_VAS_HI", nVAS, nB_HI, 200.0);
    auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    addBjtExtended(c, nB_HI, nB_Mult, nB_LO, q_mult, false, "Q_Bias");
    c.addElement<Resistor>("R1_Mult", nB_HI, nB_Mult, 1800.0);
    c.addElement<Resistor>("R2_Mult", nB_Mult, nB_LO, 1000.0);
    c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 0.5e-3);

    // Output Stage
    auto q7 = spiceToBjtParams(getSpiceBjtModel("2N3053"));
    addBjtExtended(c, nVCC, nB_HI, nE7, q7, false, "Q7");
    c.addElement<Resistor>("R_Out7", nE7, nOUT, 0.47);
    
    auto q8 = spiceToBjtParams(getSpiceBjtModel("2N4036"));
    addBjtExtended(c, nVEE, nB_LO, nE8, q8, true, "Q8");
    c.addElement<Resistor>("R_Out8", nE8, nOUT, 0.47);
    
    c.addElement<Resistor>("R_Load", nOUT, GND, 10000.0);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    
    cout << "Drive V(C4) = " << v_drive << " V" << endl;
    cout << "V(VAS) = " << x[nVAS] << " V" << endl;
    cout << "V(OUT) = " << x[nOUT] << " V" << endl;
    
    if (abs(x[nOUT]) < 14.0 && abs(x[nVAS]) < 14.0) {
        cout << "PASS: Loop biased within rails." << endl;
        return true;
    } 
    cout << "FAIL: Output railed/floating." << endl;
    return false;
}

int main() {
    testFeedbackOnly();
    testQ2BaseParasitics();
    testBiasLoopMatrix();
    return 0;
}
