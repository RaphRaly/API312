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

// VAS + BIAS CHAIN TEST (per patron MUST requirement)
// This replaces the invalid "isolated VAS" test that was missing the bias chain.
// 
// Complete VAS with:
// - Q4 PNP driver (base driven by voltage source to remove diff pair feedback)
// - Q5 NPN (VAS output transistor)
// - I_VAS current source (1.5mA)
// - R_VAS_HI (200Ω) to bias chain
// - VBE Multiplier (Q_Bias + R1_Mult + R2_Mult)  
// - I_BiasChain_Sink (0.5mA)
//
// Expected KCL balance:
// I_VAS (1.5mA) = I_BiasChain (~0.5mA) + I_Q5 (~1.0mA)

int main() {
    cout << "========================================" << endl;
    cout << "  VAS + BIAS CHAIN TEST" << endl;
    cout << "========================================" << endl;
    cout << "\nThis test includes the complete bias chain" << endl;
    cout << "so that KCL balances properly at nVAS." << endl;
    
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
    
    // Bias chain nodes
    NodeIndex nB_HI = c.createNode("VAS_High");
    NodeIndex nB_LO = c.createNode("VAS_Low");
    NodeIndex nB_Mult = c.createNode("D_Bias_mid");
    
    // Get BJT params
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    
    // ======================================================
    // VAS STAGE
    // ======================================================
    // Q4 PNP (VAS driver)
    auto q4_nodes = addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);  // Load for Q4 collector
    
    // Q5 NPN (VAS)
    auto q5_nodes = addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    c.addElement<CapacitorTrap>("C_Miller", nVAS, nC4, 220.0e-12);
    
    // *** CRITICAL: Drive Q4 base with voltage source ***
    // Removes feedback loop uncertainty
    c.addElement<VoltageSource>("V_Drive_Q4", nB4, GND, 12.9);
    
    // ======================================================
    // BIAS CHAIN (the missing piece!)
    // ======================================================
    // VBE Multiplier
    addBjtExtended(c, nB_HI, nB_Mult, nB_LO, q_mult, false, "Q_Bias");
    c.addElement<Resistor>("R1_Mult", nB_HI, nB_Mult, 1800.0);
    c.addElement<Resistor>("R2_Mult", nB_Mult, nB_LO, 1000.0);
    
    // Connection from VAS to bias chain
    c.addElement<Resistor>("R_VAS_HI", nVAS, nB_HI, 200.0);
    
    // Bias chain current sink - THIS ABSORBS ~0.5mA FROM I_VAS!
    c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 0.5e-3);
    
    // *** ADD LOAD: Simulate output stage connection ***
    // In the real 2520, Q7/Q8 emitter followers connect B_HI/B_LO to OUT (at 0V).
    // Without this, the bias chain floats to the negative rail.
    // We add a resistive path to ground to set the DC level.
    NodeIndex nOUT_sim = c.createNode("OUT_sim");
    c.addElement<Resistor>("R_Load_sim", nOUT_sim, GND, 10000.0);  // 10k load to GND
    c.addElement<Resistor>("R_Out_HI", nB_HI, nOUT_sim, 1.0);      // Simulate Q7 emitter follower
    c.addElement<Resistor>("R_Out_LO", nB_LO, nOUT_sim, 1.0);      // Simulate Q8 emitter follower
    
    // ======================================================
    // NODESETS
    // ======================================================
    cout << "\nApplying nodesets..." << endl;
    
    // VAS stage
    c.setNodeset(nE4, 13.6);
    c.setNodeset(nC4, -13.1);
    c.setNodeset(nVAS, 0.8);
    c.setNodeset(nE5, -13.8);
    
    // Bias chain
    c.setNodeset(nB_HI, 0.65);
    c.setNodeset(nB_LO, -0.65);
    c.setNodeset(nB_Mult, 0.0);
    
    // Internal nodes
    c.setNodeset(q4_nodes.e_int, 13.6);
    c.setNodeset(q4_nodes.b_int, 12.9);
    c.setNodeset(q4_nodes.c_int, -13.1);
    
    c.setNodeset(q5_nodes.e_int, -13.8);
    c.setNodeset(q5_nodes.b_int, -13.1);
    c.setNodeset(q5_nodes.c_int, 0.8);
    
    // ======================================================
    // SOLVE DC
    // ======================================================
    vector<double> x;
    bool success = c.solveDc(x);
    double finalGmin = c.getFinalGmin();
    
    cout << "\n=== DC SOLUTION ===" << endl;
    cout << "Result: " << (success ? "SUCCESS" : "FAIL") << endl;
    cout << "Final Gmin: " << scientific << setprecision(2) << finalGmin << " S" << endl;
    
    cout << fixed << setprecision(4);
    
    // Key voltages
    double v_vas = x[nVAS];
    double v_c4 = x[nC4];
    double v_e4 = x[nE4];
    double v_e5 = x[nE5];
    double v_b_hi = x[nB_HI];
    double v_b_lo = x[nB_LO];
    
    cout << "\n=== KEY VOLTAGES ===" << endl;
    cout << "V(VAS)    = " << v_vas << " V (should be ~0.8V)" << endl;
    cout << "V(C4)     = " << v_c4 << " V (Q5 base, should be ~-13V)" << endl;
    cout << "V(E4)     = " << v_e4 << " V (should be ~13.5V)" << endl;
    cout << "V(E5)     = " << v_e5 << " V (should be ~-13.8V)" << endl;
    cout << "V(B_HI)   = " << v_b_hi << " V (should be ~0.65V)" << endl;
    cout << "V(B_LO)   = " << v_b_lo << " V (should be ~-0.65V)" << endl;
    
    // BJT operating points
    double v_q4_veb = x[q4_nodes.e_int] - x[q4_nodes.b_int];
    double v_q5_vbe = x[q5_nodes.b_int] - x[q5_nodes.e_int];
    
    cout << "\n=== BJT OPERATING POINTS ===" << endl;
    cout << "Q4 (PNP) VEB = " << v_q4_veb << " V";
    cout << (v_q4_veb > 0.5 ? " [ACTIVE]" : " [CUTOFF]") << endl;
    cout << "Q5 (NPN) VBE = " << v_q5_vbe << " V";
    cout << (v_q5_vbe > 0.5 ? " [ACTIVE]" : " [CUTOFF]") << endl;
    
    // ======================================================
    // KCL AT nVAS
    // ======================================================
    cout << "\n=== KCL AT nVAS ===" << endl;
    
    double i_vas = 1.5e-3;  // Current source
    double i_r_vas_hi = (v_vas - v_b_hi) / 200.0;  // Through R_VAS_HI
    double i_q5_rc = (v_vas - x[q5_nodes.c_int]) / q_vas.RC;  // Through Q5_RC
    double i_miller = (v_vas - v_c4) / 1.0e7;  // Through R_Miller_DC
    double i_gmin = finalGmin * v_vas;
    
    cout << fixed << setprecision(6);
    cout << "I_VAS (source): +" << i_vas * 1e3 << " mA (enters)" << endl;
    cout << "I_R_VAS_HI:     -" << i_r_vas_hi * 1e3 << " mA (to bias chain)" << endl;
    cout << "I_Q5_RC:        -" << i_q5_rc * 1e3 << " mA (to Q5)" << endl;
    cout << "I_Miller:       -" << i_miller * 1e6 << " uA (to C4)" << endl;
    cout << "I_Gmin:         -" << i_gmin * 1e6 << " uA (to GND)" << endl;
    
    double kcl_error = i_vas - i_r_vas_hi - i_q5_rc - i_miller - i_gmin;
    cout << "\nKCL error: " << kcl_error * 1e6 << " uA (should be ~0)" << endl;
    
    // ======================================================
    // CURRENTS SUMMARY
    // ======================================================
    cout << "\n=== CURRENT BALANCE ===" << endl;
    double i_to_bias = i_r_vas_hi;
    double i_to_q5 = i_q5_rc;
    cout << "I_VAS      = " << i_vas * 1e3 << " mA (injected)" << endl;
    cout << "I_to_bias  = " << i_to_bias * 1e3 << " mA (absorbed by bias chain)" << endl;
    cout << "I_to_Q5    = " << i_to_q5 * 1e3 << " mA (sunk by Q5)" << endl;
    cout << "Ratio: " << (i_to_bias / i_vas * 100) << "% to bias, " 
         << (i_to_q5 / i_vas * 100) << "% to Q5" << endl;
    
    // ======================================================
    // VERDICT
    // ======================================================
    cout << "\n========================================" << endl;
    cout << "  VERDICT" << endl;
    cout << "========================================" << endl;
    
    bool q4_active = (v_q4_veb > 0.5 && v_q4_veb < 0.9);
    bool q5_active = (v_q5_vbe > 0.5 && v_q5_vbe < 0.9);
    bool vas_voltage_ok = (v_vas > -2.0 && v_vas < 5.0);
    bool gmin_ok = (finalGmin < 1e-8);
    bool kcl_ok = (abs(kcl_error) < 1e-6);
    
    bool all_pass = success && q4_active && q5_active && vas_voltage_ok && gmin_ok && kcl_ok;
    
    if (all_pass) {
        cout << "*** PASS: VAS + BIAS CHAIN WORKING ***" << endl;
        cout << "KCL balanced, both transistors active." << endl;
        return 0;
    } else {
        cout << "*** ISSUES FOUND ***" << endl;
        if (!success) cout << "- DC solve failed" << endl;
        if (!q4_active) cout << "- Q4 not active (VEB=" << v_q4_veb << "V)" << endl;
        if (!q5_active) cout << "- Q5 not active (VBE=" << v_q5_vbe << "V)" << endl;
        if (!vas_voltage_ok) cout << "- VAS voltage non-physical (" << v_vas << "V)" << endl;
        if (!gmin_ok) cout << "- Gmin too high (" << finalGmin << " S)" << endl;
        if (!kcl_ok) cout << "- KCL not balanced (error=" << kcl_error * 1e6 << " uA)" << endl;
        return 1;
    }
}
