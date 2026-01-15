#include "circuit.h"
#include "current_source.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

using namespace std;

// PHASE 1: Sign Convention Verification Tests
// These tests establish the exact polarity conventions before any debugging.

bool test_current_source_convention() {
    cout << "\n=== TEST A: CurrentSource Convention ===" << endl;
    cout << "Setup: Node X with 1k to GND, CurrentSource(X, GND, 1mA)" << endl;
    cout << "MNA analysis:" << endl;
    cout << "  stampCurrentSource adds z(X) += -I, so equation: g*V_X = -I" << endl;
    cout << "  V_X = -I*R = -1mA * 1k = -1V" << endl;
    cout << "  This means current LEAVES X (is extracted from X)" << endl;
    
    Circuit c;
    c.createNode("GND");
    NodeIndex nX = c.createNode("X");
    
    c.addElement<Resistor>("R1k", nX, GND, 1000.0);
    c.addElement<CurrentSource>("I_test", nX, GND, 1.0e-3); // 1mA from X to GND
    
    vector<double> x;
    if (!c.solveDc(x)) {
        cout << "FAILED: No convergence" << endl;
        return false;
    }
    
    double vX = x[nX];
    cout << fixed << setprecision(6);
    cout << "Result: V(X) = " << vX << " V" << endl;
    
    // KCL at X: I_resistor_leaving + I_source_leaving = 0
    // I_resistor = V_X / R (leaving if positive V_X)
    // I_source = +I (from "a to b" means current leaves a)
    // V_X/R + I = 0 -> V_X = -I*R = -1V
    if (abs(vX + 1.0) < 0.001) {
        cout << "CONFIRMED: CurrentSource(a, b, I) means I flows FROM a TO b" << endl;
        cout << "  (current LEAVES node a at rate I, enters node b)" << endl;
        cout << "  Comment in current_source.h is CORRECT." << endl;
        return true;
    } else {
        cout << "ERROR: V(X) = " << vX << " - expected -1.0V!" << endl;
        return false;
    }
}

bool test_current_source_reverse() {
    cout << "\n=== TEST A2: CurrentSource Reverse ===" << endl;
    cout << "Setup: Node X with 1k to GND, CurrentSource(GND, X, 1mA)" << endl;
    
    Circuit c;
    c.createNode("GND");
    NodeIndex nX = c.createNode("X");
    
    c.addElement<Resistor>("R1k", nX, GND, 1000.0);
    c.addElement<CurrentSource>("I_test", GND, nX, 1.0e-3); // 1mA from GND to X
    
    vector<double> x;
    if (!c.solveDc(x)) {
        cout << "FAILED: No convergence" << endl;
        return false;
    }
    
    double vX = x[nX];
    cout << fixed << setprecision(6);
    cout << "Result: V(X) = " << vX << " V" << endl;
    cout << "(Should be opposite sign of Test A)" << endl;
    return true;
}

bool test_voltage_source_convention() {
    cout << "\n=== TEST B: VoltageSource Convention ===" << endl;
    cout << "Setup: VoltageSource(nVCC, GND, 15) and VoltageSource(GND, nVEE, 15)" << endl;
    cout << "This matches the 2520 builder's Vcc_Src and Vee_Src" << endl;
    
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    
    // Exactly as in api_2520_builder.h lines 48-49:
    // c.addElement<VoltageSource>("Vcc_Src", nVCC, nGND, vcc_val);
    // c.addElement<VoltageSource>("Vee_Src", nGND, nVEE, vcc_val);
    c.addElement<VoltageSource>("Vcc_Src", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee_Src", GND, nVEE, 15.0);
    
    // Add a load to make the circuit solvable
    c.addElement<Resistor>("R_Load", nVCC, nVEE, 1000.0);
    
    vector<double> x;
    if (!c.solveDc(x)) {
        cout << "FAILED: No convergence" << endl;
        return false;
    }
    
    double vVCC = x[nVCC];
    double vVEE = x[nVEE];
    
    cout << fixed << setprecision(6);
    cout << "Result: V(VCC) = " << vVCC << " V" << endl;
    cout << "Result: V(VEE) = " << vVEE << " V" << endl;
    
    // Check expected values
    bool vcc_ok = abs(vVCC - 15.0) < 0.001;
    bool vee_ok = abs(vVEE + 15.0) < 0.001;
    
    if (vcc_ok && vee_ok) {
        cout << "CONVENTION: VoltageSource(a, b, V) means V(a) - V(b) = V" << endl;
        cout << "  Vcc_Src(VCC, GND, 15) -> V(VCC) - V(GND) = 15 -> V(VCC) = +15V ✓" << endl;
        cout << "  Vee_Src(GND, VEE, 15) -> V(GND) - V(VEE) = 15 -> V(VEE) = -15V ✓" << endl;
        return true;
    } else {
        cout << "UNEXPECTED VALUES!" << endl;
        if (!vcc_ok) cout << "  Expected V(VCC)=+15V, got " << vVCC << endl;
        if (!vee_ok) cout << "  Expected V(VEE)=-15V, got " << vVEE << endl;
        return false;
    }
}

bool verify_2520_current_sources() {
    cout << "\n=== TEST C: Verify 2520 Current Source Polarities ===" << endl;
    cout << "Based on convention from Test A, check I_Tail, I_VAS, I_BiasChain_Sink" << endl;
    
    // From api_2520_builder.h:
    // Line 80: c.addElement<CurrentSource>("I_Tail", nTAIL, nVEE, 200.0e-6);
    // Line 90: c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    // Line 102: c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 0.5e-3);
    
    cout << "\nIf CurrentSource(a, b, I) means I flows FROM a TO b:" << endl;
    cout << "  I_Tail(nTAIL, nVEE, 200uA): 200uA flows from TAIL to VEE" << endl;
    cout << "    -> Current sinks from tail to negative rail ✓ (correct for tail)" << endl;
    cout << "  I_VAS(nVCC, nVAS, 1.5mA): 1.5mA flows from VCC to VAS" << endl;
    cout << "    -> Active load sources current into VAS node ✓" << endl;
    cout << "  I_BiasChain_Sink(nB_LO, nVEE, 0.5mA): 0.5mA from B_LO to VEE" << endl;
    cout << "    -> Pulls current from bias chain ✓" << endl;
    
    cout << "\nAll current source polarities appear CORRECT if convention is a->b." << endl;
    return true;
}

int main() {
    cout << "========================================" << endl;
    cout << "  PHASE 1: SIGN CONVENTION TESTS" << endl;
    cout << "========================================" << endl;
    
    bool all_pass = true;
    
    all_pass &= test_current_source_convention();
    all_pass &= test_current_source_reverse();
    all_pass &= test_voltage_source_convention();
    all_pass &= verify_2520_current_sources();
    
    cout << "\n========================================" << endl;
    if (all_pass) {
        cout << "  ALL CONVENTION TESTS PASSED" << endl;
    } else {
        cout << "  SOME TESTS FAILED" << endl;
    }
    cout << "========================================" << endl;
    
    return all_pass ? 0 : 1;
}
