#pragma once
#include "bjt_ebers_moll.h"
#include "capacitor_trap.h"
#include "circuit.h"
#include "current_source.h"
#include "diode_shockley_nr.h"
#include "resistor.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>

using namespace std;

// API 2520 Continuation Builder
// Builds the circuit with parameterized feedback and supply for homotopy continuation.
//
// Parameters:
//   alpha: Feedback strength [0,1]. R_fb = R_nominal / max(alpha, ALPHA_MIN)
//          At alpha=0.001, R_fb = 10MΩ (quasi-open feedback)
//          At alpha=1.0, R_fb = 10kΩ (nominal)
//   supplyScale: Supply voltage scale [0,1]. VCC = +15*s, VEE = -15*s
//
// The continuation strategy:
//   1. Start with weak feedback (alpha small) so diff pair operates independently
//   2. Ramp supply to full ±15V with feedback weak
//   3. Then ramp feedback strength to 1.0
// This ensures Q2 is already conducting when feedback engages, avoiding latch.

class Api2520Continuation {
public:
    static constexpr double ALPHA_MIN = 1e-6;
    static constexpr double R_FB_NOMINAL = 10000.0;  // 10kΩ
    static constexpr double R_INPUT_NOMINAL = 10000.0;  // 10kΩ
    
    // Node indices exposed for instrumentation
    struct NodeMap {
        NodeIndex VCC, VEE, INP, INM, INM_Ref, OUT;
        NodeIndex C1, C2, C4, E4, E5, VAS;
        NodeIndex B_HI, B_LO, TAIL;
        // Internal BJT nodes for Q2
        NodeIndex Q2_Bi, Q2_Ei, Q2_Ci;
        // Internal BJT nodes for Q4
        NodeIndex Q4_Bi, Q4_Ei, Q4_Ci;
    };
    
    static NodeMap build(Circuit &c, double alpha, double supplyScale = 1.0) {
        NodeMap nodes;
        
        double vcc_val = 15.0 * supplyScale;
        double r_fb = R_FB_NOMINAL / max(alpha, ALPHA_MIN);
        
        // Create nodes
        nodes.VCC = c.createNode("VCC");
        nodes.VEE = c.createNode("VEE");
        nodes.INP = c.createNode("INP");
        nodes.INM = c.createNode("INM");
        nodes.INM_Ref = c.createNode("INM_Ref");
        nodes.OUT = c.createNode("OUT");
        nodes.TAIL = c.createNode("Diff_Tail");
        nodes.C1 = c.createNode("Q1_Col");
        nodes.C2 = c.createNode("Q2_Col");
        NodeIndex nE1 = c.createNode("Q1_Emit");
        NodeIndex nE2 = c.createNode("Q2_Emit");
        NodeIndex nE3 = c.createNode("Q3_Emit");
        NodeIndex nE9 = c.createNode("Q9_Emit");
        nodes.E4 = c.createNode("Q4_Emit");
        nodes.C4 = c.createNode("Q4_Col");
        nodes.VAS = c.createNode("VAS_Col");
        nodes.E5 = c.createNode("Q5_Emit");
        nodes.B_HI = c.createNode("VAS_High");
        nodes.B_LO = c.createNode("VAS_Low");
        NodeIndex nE7 = c.createNode("Q7_Emit");
        NodeIndex nE8 = c.createNode("Q8_Emit");
        
        // Aliases
        NodeIndex nB4 = nodes.C2;  // Q4 Base = Q2 Collector

        // === SUPPLIES ===
        c.addElement<VoltageSource>("Vcc_Src", nodes.VCC, GND, vcc_val);
        c.addElement<VoltageSource>("Vee_Src", GND, nodes.VEE, vcc_val);
        vinP_ptr = &c.addElement<VoltageSource>("VinP", nodes.INP, GND, 0.0);
        vinM_ptr = &c.addElement<VoltageSource>("VinM", nodes.INM_Ref, GND, 0.0);

        // === FEEDBACK (parameterized by alpha) ===
        // R_fb = R_nominal / alpha → at alpha=0.001, R_fb = 10MΩ
        c.addElement<Resistor>("R_Feedback", nodes.OUT, nodes.INM, r_fb);
        c.addElement<Resistor>("R_Input", nodes.INM, nodes.INM_Ref, R_INPUT_NOMINAL);
        
        // Bias resistors for DC stability
        c.addElement<Resistor>("R_INP_Bias", nodes.INP, GND, 10.0e6);
        c.addElement<Resistor>("R_INM_Ref_Bias", nodes.INM_Ref, GND, 10.0e6);

        // === SIGNAL MIRROR (Active Load) ===
        auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
        addBjtExtended(c, nodes.C1, nodes.C1, nE3, q_pnp, true, "Q3");
        c.addElement<Resistor>("R_Deg3", nodes.VCC, nE3, 100.0);
        addBjtExtended(c, nodes.C2, nodes.C1, nE9, q_pnp, true, "Q9");
        c.addElement<Resistor>("R_Deg9", nodes.VCC, nE9, 100.0);

        // === INPUT STAGE ===
        auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
        addBjtExtended(c, nodes.C1, nodes.INP, nE1, q_npn, false, "Q1");
        auto q2_nodes = addBjtExtended(c, nodes.C2, nodes.INM, nE2, q_npn, false, "Q2");
        nodes.Q2_Bi = q2_nodes.b_int;
        nodes.Q2_Ei = q2_nodes.e_int;
        nodes.Q2_Ci = q2_nodes.c_int;
        
        c.addElement<Resistor>("R_E1", nE1, nodes.TAIL, 100.0);
        c.addElement<Resistor>("R_E2", nE2, nodes.TAIL, 100.0);
        c.addElement<CurrentSource>("I_Tail", nodes.TAIL, nodes.VEE, 200.0e-6);

        // === VAS ===
        auto q4_nodes = addBjtExtended(c, nodes.C4, nB4, nodes.E4, q_pnp, true, "Q4");
        nodes.Q4_Bi = q4_nodes.b_int;
        nodes.Q4_Ei = q4_nodes.e_int;
        nodes.Q4_Ci = q4_nodes.c_int;
        c.addElement<Resistor>("R_E4", nodes.VCC, nodes.E4, 7000.0);
        c.addElement<Resistor>("R_C4", nodes.C4, nodes.VEE, 10000.0);

        auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
        addBjtExtended(c, nodes.VAS, nodes.C4, nodes.E5, q_vas, false, "Q5");
        c.addElement<Resistor>("R_E5", nodes.E5, nodes.VEE, 1200.0);
        c.addElement<CurrentSource>("I_VAS", nodes.VCC, nodes.VAS, 1.5e-3);
        c.addElement<Resistor>("R_Miller_DC", nodes.VAS, nodes.C4, 1.0e7);
        c.addElement<CapacitorTrap>("C_Miller", nodes.VAS, nodes.C4, 220.0e-12);

        // === VBE MULTIPLIER ===
        NodeIndex nB_Mult = c.createNode("D_Bias_mid");
        auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
        addBjtExtended(c, nodes.B_HI, nB_Mult, nodes.B_LO, q_mult, false, "Q_Bias");
        c.addElement<Resistor>("R1_Mult", nodes.B_HI, nB_Mult, 1800.0);
        c.addElement<Resistor>("R2_Mult", nB_Mult, nodes.B_LO, 1000.0);

        c.addElement<Resistor>("R_VAS_HI", nodes.VAS, nodes.B_HI, 200.0);
        c.addElement<CurrentSource>("I_BiasChain_Sink", nodes.B_LO, nodes.VEE, 0.5e-3);

        // === OUTPUT STAGE ===
        auto q7_params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
        addBjtExtended(c, nodes.VCC, nodes.B_HI, nE7, q7_params, false, "Q7");
        c.addElement<Resistor>("R_Out7", nE7, nodes.OUT, 0.47);
        auto q8_params = spiceToBjtParams(getSpiceBjtModel("2N4036"));
        addBjtExtended(c, nodes.VEE, nodes.B_LO, nE8, q8_params, true, "Q8");
        c.addElement<Resistor>("R_Out8", nE8, nodes.OUT, 0.47);

        c.addElement<Resistor>("R_Load", nodes.OUT, GND, 10000.0);

        // === NODESETS (initial guesses, help first alpha step) ===
        // These are derived from physical DC analysis (see walkthrough)
        c.setNodeset(nodes.OUT, 0.0);
        c.setNodeset(nodes.VAS, 0.8);
        c.setNodeset(nodes.B_HI, 0.65);
        c.setNodeset(nodes.B_LO, -0.65);
        c.setNodeset(nodes.C4, -13.1);
        c.setNodeset(nodes.E5, -13.8);
        c.setNodeset(nodes.E4, 13.6);
        c.setNodeset(nodes.C2, 12.9);  // CRITICAL: NOT VCC-1
        c.setNodeset(nodes.INM, 0.0);
        c.setNodeset(nodes.INP, 0.0);
        c.setNodeset(nodes.TAIL, -14.3);  // Help diff pair startup
        
        return nodes;
    }
    
    static VoltageSource* getVinP() { return vinP_ptr; }
    static VoltageSource* getVinM() { return vinM_ptr; }

private:
    static inline VoltageSource* vinP_ptr = nullptr;
    static inline VoltageSource* vinM_ptr = nullptr;
};
