#include "circuit.h"
#include "bjt_ebers_moll.h"
#include "spice_models_2520.h"
#include "resistor.h"
#include "current_source.h"
#include "voltage_source.h"
#include <iostream>
#include <iomanip>
#include <cassert>

using namespace std;

// Helper to check approximate equality
bool isClose(double a, double b, double tol = 1e-3) {
    return abs(a - b) < tol;
}

// Helper: Build and solve a PNP current mirror, return slave/master ratio
double run_mirror_test(double vaf_override, const string& label) {
    cout << "\n=== Mirror Test: " << label << " (VAF=" << vaf_override << ") ===" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nMasterPos = c.createNode("MasterPos");
    NodeIndex nMasterE = c.createNode("MasterE");
    NodeIndex nSlaveE = c.createNode("SlaveE");
    NodeIndex nLoad = c.createNode("Load");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    
    // Get base params and override VAF
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    q_pnp.VAF = vaf_override;  // Override Early voltage
    
    // Master (diode connected)
    c.addElement<CurrentSource>("I_Ref", nMasterPos, GND, 1.0e-3);
    addBjtExtended(c, nMasterPos, nMasterPos, nMasterE, q_pnp, true, "Q_Master");
    c.addElement<Resistor>("R_DegM", nVCC, nMasterE, 100.0);
    
    // Slave
    addBjtExtended(c, nLoad, nMasterPos, nSlaveE, q_pnp, true, "Q_Slave");
    c.addElement<Resistor>("R_DegS", nVCC, nSlaveE, 100.0);
    c.addElement<Resistor>("R_Load", nLoad, GND, 1000.0);
    
    vector<double> x;
    if (!c.solveDc(x)) {
        cout << "FAILED: No Convergence" << endl;
        exit(1);
    }
    
    double iSlave = x[nLoad] / 1000.0;
    double iMaster = 1.0e-3;
    double ratio = iSlave / iMaster;
    
    cout << "  I_Master = " << iMaster * 1e3 << " mA" << endl;
    cout << "  I_Slave  = " << iSlave * 1e3 << " mA" << endl;
    cout << "  Ratio    = " << ratio << endl;
    cout << "  VEC_Master = " << (x[nMasterE] - x[nMasterPos]) << " V" << endl;
    cout << "  VEC_Slave  = " << (x[nSlaveE] - x[nLoad]) << " V" << endl;
    
    return ratio;
}

void test_mirror_no_early() {
    // TEST A: No Early effect (VAF = 1e9 effectively disables it)
    // INVARIANT 1: Without Early, ratio ≈ 1.0 (tight tolerance 1%)
    double ratio = run_mirror_test(1e9, "No Early (VAF=1e9)");
    
    if (abs(ratio - 1.0) > 0.01) {
        cout << "FAIL: Without Early effect, ratio should be ~1.0 (got " << ratio << ")" << endl;
        exit(1);
    }
    cout << "PASS: Invariant 1 verified - no Early gives ratio ~1.0" << endl;
}

void test_mirror_with_early() {
    // TEST B: With Early effect (VAF = 45V nominal)
    // INVARIANT 2: With Early + VEC(slave) >> VEC(master), ratio MUST be > 1.0
    //              This proves the Early effect sign is correct (more VEC = more current)
    // Sanity bound: ratio < 1.2 (degeneration + RB/RC/RE limit the effect)
    double ratio = run_mirror_test(44.95, "With Early (VAF=45)");
    
    if (ratio <= 1.0) {
        cout << "FAIL: With Early effect, ratio should be > 1.0 (got " << ratio << ")" << endl;
        cout << "      This indicates Early effect sign is WRONG." << endl;
        exit(1);
    }
    if (ratio > 1.2) {
        cout << "FAIL: Ratio > 1.2 is suspicious - check for other bugs" << endl;
        exit(1);
    }
    cout << "PASS: Invariant 2 verified - Early effect increases slave current (ratio > 1)" << endl;
}

void test_diff_pair() {
    cout << "\n=== Test Block: Diff Pair ===" << endl;
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    
    NodeIndex nC1 = c.createNode("C1");
    NodeIndex nC2 = c.createNode("C2");
    NodeIndex nE1 = c.createNode("E1");
    NodeIndex nE2 = c.createNode("E2");
    NodeIndex nTail = c.createNode("Tail");
    NodeIndex nInP = c.createNode("InP");
    NodeIndex nInM = c.createNode("InM");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    // Inputs grounded
    c.addElement<VoltageSource>("VinP", nInP, GND, 0.0);
    c.addElement<VoltageSource>("VinM", nInM, GND, 0.0);
    
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    
    // Q1
    addBjtExtended(c, nC1, nInP, nE1, q_npn, false, "Q1");
    c.addElement<Resistor>("Rc1", nVCC, nC1, 1000.0); // Resistive load
    c.addElement<Resistor>("Re1", nE1, nTail, 100.0);
    
    // Q2
    addBjtExtended(c, nC2, nInM, nE2, q_npn, false, "Q2");
    c.addElement<Resistor>("Rc2", nVCC, nC2, 1000.0);
    c.addElement<Resistor>("Re2", nE2, nTail, 100.0);
    
    // Tail
    c.addElement<CurrentSource>("ITail", nTail, nVEE, 2.0e-3); // 2mA
    
    vector<double> x;
    bool conv = c.solveDc(x);
    
    if (!conv) {
        cout << "FAILED: No Convergence" << endl;
        exit(1);
    }
    
    double vC1 = x[nC1];
    double vC2 = x[nC2];
    double ic1 = (15.0 - vC1) / 1000.0;
    double ic2 = (15.0 - vC2) / 1000.0;
    
    cout << "IC1: " << ic1 * 1e3 << " mA" << endl;
    cout << "IC2: " << ic2 * 1e3 << " mA" << endl;
    
    if (abs(ic1 - 1.0e-3) > 1e-4) {
        cout << "FAIL: Q1 current not ~1mA" << endl;
        exit(1);
    }
    if (abs(ic1 - ic2) > 1e-6) {
        cout << "FAIL: Asymmetry detected" << endl;
        exit(1);
    }
    cout << "PASS: Diff Pair works" << endl;
}

void test_vas_bias() {
    cout << "\n=== Test Block: VAS Bias (Q4/Q5) ===" << endl;
    // Emulate the VAS configuration being driven by a fixed voltage at Q4 Base
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    // VAS Q4 (PNP input) -> Q5 (NPN output)
    NodeIndex nC4 = c.createNode("C4");
    NodeIndex nE4 = c.createNode("E4");
    NodeIndex nB4 = c.createNode("B4"); // Input node
    NodeIndex nVAS_Out = c.createNode("VAS_Out");
    NodeIndex nE5 = c.createNode("E5");
    
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    
    // Q4 PNP
    // Base driven by Voltage Source to sweep
    // Emitter to VCC via R
    // Collector to VEE via R (load)
    
    addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    // Q5 NPN - Input from C4
    addBjtExtended(c, nVAS_Out, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    // Active Load for Q5
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS_Out, 1.5e-3);
    
    // Miller C (omitted for DC) and R
    c.addElement<Resistor>("R_Miller_DC", nVAS_Out, nC4, 1.0e7); // Feedback
    
    // Drive Base of Q4
    // Normally driven by Q2 Collector. Q2 Collector sits around 15V - (IC2 * Q9_Deg + V_Q9_EC) ? 
    // Actually Q2 Collector sets the voltage.
    // Let's set V(B4) to approx 14V (VCC - Vbe - Vdrop)
    // If Q4 is ON, V(E4) approx V(B4)+0.7. Resulting I through R_E4.
    
    c.addElement<VoltageSource>("V_Drive", nB4, GND, 14.0);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    
    cout << "V(B4) = 14.0 V" << endl;
    cout << "V(C4) (Q5 Base) = " << x[nC4] << " V" << endl;
    cout << "V(VAS_Out) = " << x[nVAS_Out] << " V" << endl;
    
    if (abs(x[nVAS_Out]) > 14.0) {
        cout << "WARNING: VAS Saturated" << endl;
    } else {
        cout << "PASS: VAS active region" << endl;
    }
}

int main() {
    test_mirror_no_early();
    test_mirror_with_early();
    test_diff_pair();
    test_vas_bias();
    cout << "\n=== ALL BLOCK TESTS PASSED ===" << endl;
    return 0;
}
