#include "bjt_params.h"
#include "bjt_ebers_moll.h"
#include "capacitor_trap.h"
#include "circuit.h"
#include "current_source.h"
#include "resistor.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// PATRON FIX v2: Isolated VAS Stage with Controlled Drive
// The full 2520 has too many feedback paths that can latch.
// This test isolates Q4+Q5 with a controlled base drive from a voltage source.

// Key insight: When Newton converges to the wrong basin, the full circuit
// feedback reinforces the latch. We need to break this by:
// 1. Driving Q4 base with a voltage source (removes diff pair uncertainty)
// 2. Scaling parasitics to reduce stiffness (patron's sensitivity test)

struct BjtNodesInternal {
    NodeIndex c_int, b_int, e_int;
};

// Manual BJT with parasitic resistors for debugging
BjtNodesInternal addBjtManual(Circuit &c, NodeIndex c_ext, NodeIndex b_ext, 
                               NodeIndex e_ext, const BjtParams &p, bool isPnp,
                               const string &name, double parasitic_scale = 1.0) {
    double RB = p.RB * parasitic_scale;
    double RC = p.RC * parasitic_scale;
    double RE = p.RE * parasitic_scale;
    
    NodeIndex c_int = (RC > 0.0) ? c.createNode(name + "_Ci") : c_ext;
    NodeIndex b_int = (RB > 0.0) ? c.createNode(name + "_Bi") : b_ext;
    NodeIndex e_int = (RE > 0.0) ? c.createNode(name + "_Ei") : e_ext;

    if (RB > 0.0)
        c.addElement<Resistor>(name + "_RB", b_ext, b_int, RB);
    if (RC > 0.0)
        c.addElement<Resistor>(name + "_RC", c_ext, c_int, RC);
    if (RE > 0.0)
        c.addElement<Resistor>(name + "_RE", e_ext, e_int, RE);

    if (p.CJE > 0.0)
        c.addElement<CapacitorTrap>(name + "_CJE", b_int, e_int, p.CJE);
    if (p.CJC > 0.0)
        c.addElement<CapacitorTrap>(name + "_CJC", b_int, c_int, p.CJC);

    if (isPnp) {
        c.addElement<BjtPnpEbersMoll>(name, c_int, b_int, e_int, p);
    } else {
        c.addElement<BjtNpnEbersMoll>(name, c_int, b_int, e_int, p);
    }
    return {c_int, b_int, e_int};
}

bool runVASTest(double parasitic_scale, bool verbose = true) {
    if (verbose) {
        cout << "\n==================================================" << endl;
        cout << "  PARASITIC SCALE = " << parasitic_scale << "x" << endl;
        cout << "==================================================" << endl;
    }
    
    Circuit c;
    
    // Supplies
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    // VAS nodes
    NodeIndex nB4 = c.createNode("Q4_Base");  // Driven by voltage source
    NodeIndex nE4 = c.createNode("Q4_Emit");
    NodeIndex nC4 = c.createNode("Q4_Col");   // = Q5 Base
    NodeIndex nVAS = c.createNode("VAS_Out");
    NodeIndex nE5 = c.createNode("Q5_Emit");
    
    // Get BJT params
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    
    // Q4 PNP (VAS driver) - with scaled parasitics
    auto q4_nodes = addBjtManual(c, nC4, nB4, nE4, q_pnp, true, "Q4", parasitic_scale);
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);  // Load for Q4 collector
    
    // Q5 NPN (VAS) - with scaled parasitics
    auto q5_nodes = addBjtManual(c, nVAS, nC4, nE5, q_vas, false, "Q5", parasitic_scale);
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    
    // *** CRITICAL: Drive Q4 base with a voltage source ***
    // This removes the feedback loop from the diff pair.
    // Q4 base should be ~12.9V for proper operation.
    c.addElement<VoltageSource>("V_Drive_Q4", nB4, GND, 12.9);
    
    // ======================================================
    // PHYSICALLY CONSISTENT NODESETS
    // ======================================================
    // Q4 PNP: VEB ≈ 0.7V → V(E4) ≈ V(B4) + 0.7 = 13.6V
    // I_Q4 = (15 - 13.6) / 7000 = 0.2mA
    // V(C4) = -15 + 0.2mA * 10000 = -13V (but we need to consider Q5)
    //
    // Q5 NPN: I_C5 ≈ I_VAS - I_bias ≈ 1.0mA
    // V(E5) = -15 + 1.0mA * 1200 = -13.8V
    // V(B5) = V(C4) = V(E5) + 0.7 = -13.1V
    
    c.setNodeset(nE4, 13.6);
    c.setNodeset(nC4, -13.1);
    c.setNodeset(nVAS, 0.8);
    c.setNodeset(nE5, -13.8);
    
    // Internal nodes with scaled-up values propagated correctly
    c.setNodeset(q4_nodes.e_int, 13.6);
    c.setNodeset(q4_nodes.b_int, 12.9);
    c.setNodeset(q4_nodes.c_int, -13.1);
    
    c.setNodeset(q5_nodes.e_int, -13.8);
    c.setNodeset(q5_nodes.b_int, -13.1);
    c.setNodeset(q5_nodes.c_int, 0.8);
    
    // Solve
    vector<double> x;
    bool success = c.solveDc(x);
    double finalGmin = c.getFinalGmin();
    
    // Extract results
    double v_vas = x[nVAS];
    double v_c4 = x[nC4];
    double v_e4 = x[nE4];
    double v_e5 = x[nE5];
    double v_q4_veb = x[q4_nodes.e_int] - x[q4_nodes.b_int];
    double v_q5_vbe = x[q5_nodes.b_int] - x[q5_nodes.e_int];
    
    // Compute currents
    double i_q4_e = (15.0 - v_e4) / 7000.0;
    double i_q5_e = (v_e5 - (-15.0)) / 1200.0;
    
    if (verbose) {
        cout << fixed << setprecision(4);
        cout << "Result: " << (success ? "SUCCESS" : "FAIL");
        cout << " | Gmin = " << scientific << setprecision(2) << finalGmin << " S" << endl;
        
        cout << fixed << setprecision(4);
        cout << "\nVoltages:" << endl;
        cout << "  V(VAS)  = " << v_vas << " V" << endl;
        cout << "  V(C4)   = " << v_c4 << " V" << endl;
        cout << "  V(E4)   = " << v_e4 << " V" << endl;
        cout << "  V(E5)   = " << v_e5 << " V" << endl;
        
        cout << "\nBJT Operating Points:" << endl;
        cout << "  Q4 (PNP) VEB = " << v_q4_veb << " V";
        cout << (v_q4_veb > 0.5 ? " [ACTIVE]" : " [CUTOFF]") << endl;
        cout << "  Q5 (NPN) VBE = " << v_q5_vbe << " V";
        cout << (v_q5_vbe > 0.5 ? " [ACTIVE]" : " [CUTOFF]") << endl;
        
        cout << "\nCurrents:" << endl;
        cout << "  I(Q4_E) = " << i_q4_e * 1e3 << " mA (in R_E4)" << endl;
        cout << "  I(Q5_E) = " << i_q5_e * 1e3 << " mA (in R_E5)" << endl;
    }
    
    // Success criteria
    bool q4_active = (v_q4_veb > 0.5 && v_q4_veb < 0.9);
    bool q5_active = (v_q5_vbe > 0.5 && v_q5_vbe < 0.9);
    bool gmin_ok = (finalGmin < 1e-8);
    bool voltages_physical = (v_vas > -5.0 && v_vas < 10.0);
    
    bool test_pass = success && q4_active && q5_active && gmin_ok && voltages_physical;
    
    if (verbose) {
        cout << "\n=== VERDICT ===" << endl;
        if (test_pass) {
            cout << "*** PASS: Q4 and Q5 both active, physical solution ***" << endl;
        } else {
            cout << "*** FAIL: ";
            if (!q4_active) cout << "Q4 not active; ";
            if (!q5_active) cout << "Q5 not active; ";
            if (!gmin_ok) cout << "Gmin too high; ";
            if (!voltages_physical) cout << "VAS voltage non-physical; ";
            cout << "***" << endl;
        }
    }
    
    return test_pass;
}

int main() {
    cout << "========================================================" << endl;
    cout << "  PATRON SENSITIVITY TEST: Parasitic Scaling" << endl;
    cout << "========================================================" << endl;
    cout << "\nThis test verifies if the convergence issue is due to" << endl;
    cout << "numerical stiffness from parasitic resistors (RB/RC/RE)." << endl;
    cout << "\nIf high scale works but low scale fails, we have proven" << endl;
    cout << "it's a stiffness/continuation problem, not a circuit bug." << endl;
    
    // Test sequence per patron's instructions
    double scales[] = {100.0, 10.0, 1.0, 0.1};
    bool results[4];
    
    for (int i = 0; i < 4; i++) {
        results[i] = runVASTest(scales[i], true);
    }
    
    cout << "\n========================================================" << endl;
    cout << "  SUMMARY" << endl;
    cout << "========================================================" << endl;
    
    for (int i = 0; i < 4; i++) {
        cout << "Scale " << setw(5) << scales[i] << "x: " 
             << (results[i] ? "PASS" : "FAIL") << endl;
    }
    
    // Analyze results
    cout << "\n=== ANALYSIS ===" << endl;
    
    if (results[0] && !results[2]) {
        cout << "*** CONFIRMED: Stiffness/Continuation Problem ***" << endl;
        cout << "High parasitic scale converges, low scale doesn't." << endl;
        cout << "Solution: Use source stepping or pseudo-transient." << endl;
    } else if (results[0] && results[1] && results[2] && results[3]) {
        cout << "*** ALL PASS: Isolated VAS works at all scales ***" << endl;
        cout << "The problem is in the feedback topology, not VAS." << endl;
    } else if (!results[0]) {
        cout << "*** FUNDAMENTAL PROBLEM ***" << endl;
        cout << "Even high parasitic scale fails." << endl;
        cout << "Check: BJT model, circuit topology, nodesets." << endl;
    } else {
        cout << "*** PARTIAL: Some scales work, investigate gradient ***" << endl;
    }
    
    return (results[2]) ? 0 : 1;  // Return based on scale=1 (normal) test
}
