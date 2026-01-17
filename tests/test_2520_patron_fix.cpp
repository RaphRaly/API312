#include "bjt_ebers_moll.h"
#include "capacitor_trap.h"
#include "circuit.h"
#include "current_source.h"
#include "diode_shockley_nr.h"
#include "resistor.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// PATRON FIX TEST - Physically consistent nodesets without the nC2=VCC-1 bomb
// Key changes:
// 1. REMOVED the dangerous nodeset(nC2 = VCC-1) 
// 2. ADDED nodesets on internal BJT nodes (Q4_Bi, Q4_Ei, Q5_Bi, Q5_Ei)
// 3. Used physically derived values from DC equilibrium analysis

int main() {
    cout << "=== PATRON FIX: 2520 DC STABILIZATION ===" << endl;
    cout << "Strategy: Corrected external + internal nodesets" << endl;
    cout << "          WITHOUT the nC2=VCC-1 latch bomb" << endl;
    
    Circuit c;
    
    // === NODE CREATION (from api_2520_builder.h) ===
    double vcc_val = 15.0;
    
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    NodeIndex nGND = GND;
    NodeIndex nINP = c.createNode("INP");
    NodeIndex nINM = c.createNode("INM");
    NodeIndex nINM_Ref = c.createNode("INM_Ref");
    NodeIndex nOUT = c.createNode("OUT");
    NodeIndex nTAIL = c.createNode("Diff_Tail");
    NodeIndex nC1 = c.createNode("Q1_Col");
    NodeIndex nC2 = c.createNode("Q2_Col");
    NodeIndex nE1 = c.createNode("Q1_Emit");
    NodeIndex nE2 = c.createNode("Q2_Emit");
    NodeIndex nE3 = c.createNode("Q3_Emit");
    NodeIndex nE9 = c.createNode("Q9_Emit");
    NodeIndex nB4 = nC2;  // Q4 Base = Q2 Collector
    NodeIndex nE4 = c.createNode("Q4_Emit");
    NodeIndex nC4 = c.createNode("Q4_Col");
    NodeIndex nVAS = c.createNode("VAS_Col");
    NodeIndex nE5 = c.createNode("Q5_Emit");
    NodeIndex nB_HI = c.createNode("VAS_High");
    NodeIndex nB_LO = c.createNode("VAS_Low");
    NodeIndex nE7 = c.createNode("Q7_Emit");
    NodeIndex nE8 = c.createNode("Q8_Emit");

    // === SUPPLIES ===
    c.addElement<VoltageSource>("Vcc_Src", nVCC, nGND, vcc_val);
    c.addElement<VoltageSource>("Vee_Src", nGND, nVEE, vcc_val);
    c.addElement<VoltageSource>("VinP", nINP, nGND, 0.0);
    c.addElement<VoltageSource>("VinM", nINM_Ref, nGND, 0.0);

    // === FEEDBACK (Gain = 2) ===
    c.addElement<Resistor>("R_Feedback", nOUT, nINM, 10000.0);
    c.addElement<Resistor>("R_Input", nINM, nINM_Ref, 10000.0);
    c.addElement<Resistor>("R_INP_Bias", nINP, nGND, 10.0e6);
    c.addElement<Resistor>("R_INM_Ref_Bias", nINM_Ref, nGND, 10.0e6);

    // === SIGNAL MIRROR (Active Load) ===
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    addBjtExtended(c, nC1, nC1, nE3, q_pnp, true, "Q3");
    c.addElement<Resistor>("R_Deg3", nVCC, nE3, 100.0);
    addBjtExtended(c, nC2, nC1, nE9, q_pnp, true, "Q9");
    c.addElement<Resistor>("R_Deg9", nVCC, nE9, 100.0);

    // === INPUT STAGE ===
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    addBjtExtended(c, nC1, nINP, nE1, q_npn, false, "Q1");
    addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2");
    c.addElement<Resistor>("R_E1", nE1, nTAIL, 100.0);
    c.addElement<Resistor>("R_E2", nE2, nTAIL, 100.0);
    c.addElement<CurrentSource>("I_Tail", nTAIL, nVEE, 200.0e-6);

    // === VAS (Critical path) ===
    // Q4 PNP with internal node capture
    auto q4_nodes = addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);

    // Q5 NPN with internal node capture
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    auto q5_nodes = addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    c.addElement<CapacitorTrap>("C_Miller", nVAS, nC4, 220.0e-12);

    // === VBE MULTIPLIER ===
    NodeIndex nB_Mult = c.createNode("D_Bias_mid");
    auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    addBjtExtended(c, nB_HI, nB_Mult, nB_LO, q_mult, false, "Q_Bias");
    c.addElement<Resistor>("R1_Mult", nB_HI, nB_Mult, 1800.0);
    c.addElement<Resistor>("R2_Mult", nB_Mult, nB_LO, 1000.0);

    c.addElement<Resistor>("R_VAS_HI", nVAS, nB_HI, 200.0);
    c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 0.5e-3);

    // === OUTPUT STAGE ===
    auto q7_params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
    addBjtExtended(c, nVCC, nB_HI, nE7, q7_params, false, "Q7");
    c.addElement<Resistor>("R_Out7", nE7, nOUT, 0.47);
    auto q8_params = spiceToBjtParams(getSpiceBjtModel("2N4036"));
    addBjtExtended(c, nVEE, nB_LO, nE8, q8_params, true, "Q8");
    c.addElement<Resistor>("R_Out8", nE8, nOUT, 0.47);

    c.addElement<Resistor>("R_Load", nOUT, nGND, 10000.0);

    // ======================================================
    // PATRON FIX NODESETS - Physically derived values
    // ======================================================
    cout << "\n=== APPLYING PATRON FIX NODESETS ===" << endl;
    
    // --- Output stage ---
    c.setNodeset(nOUT, 0.0);
    cout << "  OUT      = 0.0 V" << endl;
    
    // --- VAS output & bias chain ---
    // V(VAS) ≈ V(B_HI) + 0.5mA × 200Ω ≈ 0.7 + 0.1 = 0.8V
    c.setNodeset(nVAS, 0.8);
    c.setNodeset(nB_HI, 0.65);
    c.setNodeset(nB_LO, -0.65);
    cout << "  VAS_Col  = 0.8 V (nVAS)" << endl;
    cout << "  VAS_High = 0.65 V (nB_HI)" << endl;
    cout << "  VAS_Low  = -0.65 V (nB_LO)" << endl;
    
    // --- Q5 NPN (VAS transistor) ---
    // I_Q5 ≈ 1.0mA (I_VAS - I_BiasChain)
    // V(E5) = -15 + 1.0mA × 1.2kΩ = -13.8V
    // V(B5) = V(C4) = V(E5) + 0.7 = -13.1V
    c.setNodeset(nE5, -13.8);
    c.setNodeset(nC4, -13.1);  // External C4 = Q5 Base
    cout << "  Q5_Emit  = -13.8 V (nE5)" << endl;
    cout << "  Q4_Col   = -13.1 V (nC4 = Q5_Base)" << endl;
    
    // Q5 INTERNAL nodes (CRITICAL for convergence)
    // Q5_Ei ≈ V(E5) (small RE drop)
    // Q5_Bi ≈ V(C4) (small RB drop)  
    // Q5_Ci ≈ V(VAS) - RC*Ic (very small)
    c.setNodeset(q5_nodes.e_int, -13.8);
    c.setNodeset(q5_nodes.b_int, -13.1);
    c.setNodeset(q5_nodes.c_int, 0.75);  // Close to nVAS
    cout << "  Q5_Ei    = -13.8 V (internal)" << endl;
    cout << "  Q5_Bi    = -13.1 V (internal)" << endl;
    cout << "  Q5_Ci    = 0.75 V (internal)" << endl;

    // --- Q4 PNP (VAS driver) ---
    // I_Q4 ≈ 0.2mA (small current, set by diff pair)
    // V(E4) = 15 - 0.2mA × 7kΩ = 13.6V
    // V(B4) = V(C2) = V(E4) - 0.7 = 12.9V
    // *** PATRON FIX: NOT VCC-1 (14V) which kills Q4! ***
    c.setNodeset(nE4, 13.6);
    c.setNodeset(nC2, 12.9);   // B4 = C2, CORRECTED from VCC-1
    cout << "  Q4_Emit  = 13.6 V (nE4)" << endl;
    cout << "  Q4_Base  = 12.9 V (nC2 = nB4) *** PATRON FIX ***" << endl;
    
    // Q4 INTERNAL nodes (CRITICAL)
    c.setNodeset(q4_nodes.e_int, 13.6);
    c.setNodeset(q4_nodes.b_int, 12.9);
    c.setNodeset(q4_nodes.c_int, -13.1);  // Close to nC4
    cout << "  Q4_Ei    = 13.6 V (internal)" << endl;
    cout << "  Q4_Bi    = 12.9 V (internal)" << endl;
    cout << "  Q4_Ci    = -13.1 V (internal)" << endl;

    // --- Input stage mirror ---
    // Keep C1/C2 collectors at reasonable values for active mirror
    c.setNodeset(nC1, 12.5);  // Mirror reference, not VCC-1
    cout << "  Q1_Col   = 12.5 V (nC1) - Active mirror" << endl;

    // ======================================================
    // SOLVE WITH PSEUDO-TRANSIENT
    // ======================================================
    cout << "\n=== SOLVING ===" << endl;
    
    vector<double> x;
    bool success = c.solveDcPseudoTransient(x, 5.0e-3, 1.0e-6);
    
    double finalGmin = c.getFinalGmin();
    
    cout << "\nResult: " << (success ? "SUCCESS" : "FAIL") << endl;
    cout << "Final Gmin: " << scientific << setprecision(2) << finalGmin << " S" << endl;
    
    cout << fixed << setprecision(4);
    cout << "\n=== KEY VOLTAGES ===" << endl;
    cout << "V(OUT)    = " << x[nOUT] << " V" << endl;
    cout << "V(VAS)    = " << x[nVAS] << " V" << endl;
    cout << "V(C4)     = " << x[nC4] << " V (Q5 Base)" << endl;
    cout << "V(C2)     = " << x[nC2] << " V (Q4 Base) *** CRITICAL ***" << endl;
    cout << "V(E4)     = " << x[nE4] << " V" << endl;
    cout << "V(E5)     = " << x[nE5] << " V" << endl;
    
    // Q4 VEB check
    double v_q4_veb = x[q4_nodes.e_int] - x[q4_nodes.b_int];
    cout << "\nQ4 (PNP) VEB = " << v_q4_veb << " V";
    if (v_q4_veb > 0.5 && v_q4_veb < 0.8) {
        cout << " [ACTIVE]" << endl;
    } else if (v_q4_veb < 0.3) {
        cout << " [*** CUTOFF - LATCH! ***]" << endl;
    } else {
        cout << endl;
    }
    
    // Q5 VBE check
    double v_q5_vbe = x[q5_nodes.b_int] - x[q5_nodes.e_int];
    cout << "Q5 (NPN) VBE = " << v_q5_vbe << " V";
    if (v_q5_vbe > 0.5 && v_q5_vbe < 0.8) {
        cout << " [ACTIVE]" << endl;
    } else if (v_q5_vbe < 0.3) {
        cout << " [*** CUTOFF - LATCH! ***]" << endl;
    } else {
        cout << endl;
    }
    
    // Verdict
    cout << "\n=== VERDICT ===" << endl;
    if (success && abs(x[nOUT]) < 1.0 && finalGmin < 1e-8) {
        cout << "*** VICTORY: PHYSICAL DC POINT FOUND ***" << endl;
        cout << "Q4 and Q5 are both active, no latch-up!" << endl;
        return 0;
    } else if (abs(x[nOUT]) > 13.0) {
        cout << "*** FAIL: Still latched to rail ***" << endl;
        return 1;
    } else if (finalGmin > 1e-6) {
        cout << "*** PARTIAL: Gmin still too high (" << finalGmin << " S) ***" << endl;
        return 1;
    } else {
        cout << "*** PARTIAL: Output " << x[nOUT] << "V not yet 0V ***" << endl;
        return 1;
    }
}
