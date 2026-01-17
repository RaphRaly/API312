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

// =============================================================================
// API 2520 Homotopy Builder
// =============================================================================
// Builds the circuit ONCE with all sources and a bleeder accessible for
// in-place parameter updates. Supports 2D homotopy:
//   - sourceScale: 0→1 scales ALL independent sources (VCC, VEE, I_Tail, I_VAS, I_Bias)
//   - alpha: 0→1 controls feedback strength (R_fb = R_nom / max(alpha, ALPHA_MIN))
//   - bleederG: Gmin-like conductance from nC2→VEE to prevent latch
//
// WHY each change helps:
// 1. sourceScale from 0: At s=0 all sources are off, circuit is trivially at 0V.
//    As s increases, transistors gradually turn on from a balanced state.
//    Current sources (I_Tail, I_VAS) scaling is CRITICAL - if they're full while
//    supplies are ramping, the transistors try to sink current that doesn't exist.
//
// 2. bleeder at nC2: When Q2 (diff pair NPN) is weakly conducting (low source scale),
//    mirror node nC2 has no sink path and may float toward VCC. A small bleeder
//    conductance nC2→VEE provides a DC path that prevents the latch, then is
//    homotopied to zero so final circuit is unchanged.
//
// 3. alpha=0 during source ramp: Feedback loop is open, so diff pair operates
//    independently. Each stage biases itself without feedback instability.
//
// 4. In-place updates: Same node indices throughout, warm-start always valid.
// =============================================================================

class Api2520Homotopy {
public:
    static constexpr double ALPHA_MIN = 1e-6;
    static constexpr double R_FB_NOMINAL = 10000.0;  // 10kΩ
    
    // All modifiable sources exposed
    struct Sources {
        VoltageSource* Vcc;
        VoltageSource* Vee;
        VoltageSource* VinP;
        VoltageSource* VinM;
        CurrentSource* I_Tail;
        CurrentSource* I_VAS;
        CurrentSource* I_BiasChain;
        Resistor* R_Feedback;
        Resistor* R_Bleeder;  // nC2→VEE anti-latch bleeder
    };
    
    // Key node indices for instrumentation
    struct Nodes {
        NodeIndex VCC, VEE, INP, INM, INM_Ref, OUT;
        NodeIndex C1, C2, C4, E4, E5, VAS;
        NodeIndex B_HI, B_LO, TAIL;
        NodeIndex Q2_Bi, Q2_Ei, Q2_Ci;
        NodeIndex Q4_Bi, Q4_Ei, Q4_Ci;
    };
    
    // Nominal source values (at sourceScale=1)
    static constexpr double VCC_NOM = 15.0;
    static constexpr double VEE_NOM = 15.0;  // Magnitude, actual is -15
    static constexpr double I_TAIL_NOM = 200.0e-6;
    static constexpr double I_VAS_NOM = 1.5e-3;
    static constexpr double I_BIAS_NOM = 0.5e-3;
    
    // Build circuit once, return sources and nodes for manipulation
    static void build(Circuit &c, Sources &src, Nodes &nodes) {
        // Create nodes (order is DETERMINISTIC - same indices every time)
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
        NodeIndex nB4 = nodes.C2;

        // === SUPPLIES (initially at 0, scaled later) ===
        src.Vcc = &c.addElement<VoltageSource>("Vcc_Src", nodes.VCC, GND, 0.0);
        src.Vee = &c.addElement<VoltageSource>("Vee_Src", GND, nodes.VEE, 0.0);
        src.VinP = &c.addElement<VoltageSource>("VinP", nodes.INP, GND, 0.0);
        src.VinM = &c.addElement<VoltageSource>("VinM", nodes.INM_Ref, GND, 0.0);

        // === FEEDBACK (initially very high R = open) ===
        src.R_Feedback = &c.addElement<Resistor>("R_Feedback", nodes.OUT, nodes.INM, 
                                                   R_FB_NOMINAL / ALPHA_MIN);  // Starts as ~10GΩ
        c.addElement<Resistor>("R_Input", nodes.INM, nodes.INM_Ref, 10000.0);
        c.addElement<Resistor>("R_INP_Bias", nodes.INP, GND, 10.0e6);
        c.addElement<Resistor>("R_INM_Ref_Bias", nodes.INM_Ref, GND, 10.0e6);

        // === BLEEDER: Anti-latch path nC2→VEE ===
        // This prevents nC2 from floating to VCC when Q2 is weakly conducting
        // Start with moderate conductance, homotopy to 0
        src.R_Bleeder = &c.addElement<Resistor>("R_Bleeder_C2", nodes.C2, nodes.VEE, 1e6);

        // === SIGNAL MIRROR ===
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
        src.I_Tail = &c.addElement<CurrentSource>("I_Tail", nodes.TAIL, nodes.VEE, 0.0);

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
        src.I_VAS = &c.addElement<CurrentSource>("I_VAS", nodes.VCC, nodes.VAS, 0.0);
        c.addElement<Resistor>("R_Miller_DC", nodes.VAS, nodes.C4, 1.0e7);
        c.addElement<CapacitorTrap>("C_Miller", nodes.VAS, nodes.C4, 220.0e-12);

        // === VBE MULTIPLIER ===
        NodeIndex nB_Mult = c.createNode("D_Bias_mid");
        auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
        addBjtExtended(c, nodes.B_HI, nB_Mult, nodes.B_LO, q_mult, false, "Q_Bias");
        c.addElement<Resistor>("R1_Mult", nodes.B_HI, nB_Mult, 1800.0);
        c.addElement<Resistor>("R2_Mult", nB_Mult, nodes.B_LO, 1000.0);
        c.addElement<Resistor>("R_VAS_HI", nodes.VAS, nodes.B_HI, 200.0);
        src.I_BiasChain = &c.addElement<CurrentSource>("I_BiasChain_Sink", nodes.B_LO, nodes.VEE, 0.0);

        // === OUTPUT STAGE ===
        auto q7_params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
        addBjtExtended(c, nodes.VCC, nodes.B_HI, nE7, q7_params, false, "Q7");
        c.addElement<Resistor>("R_Out7", nE7, nodes.OUT, 0.47);
        auto q8_params = spiceToBjtParams(getSpiceBjtModel("2N4036"));
        addBjtExtended(c, nodes.VEE, nodes.B_LO, nE8, q8_params, true, "Q8");
        c.addElement<Resistor>("R_Out8", nE8, nodes.OUT, 0.47);
        c.addElement<Resistor>("R_Load", nodes.OUT, GND, 10000.0);

        // === NODESETS (physical initial guesses) ===
        c.setNodeset(nodes.OUT, 0.0);
        c.setNodeset(nodes.VAS, 0.0);
        c.setNodeset(nodes.B_HI, 0.0);
        c.setNodeset(nodes.B_LO, 0.0);
        c.setNodeset(nodes.C4, 0.0);
        c.setNodeset(nodes.E5, 0.0);
        c.setNodeset(nodes.E4, 0.0);
        c.setNodeset(nodes.C2, 0.0);
        c.setNodeset(nodes.C1, 0.0);
        c.setNodeset(nodes.INM, 0.0);
        c.setNodeset(nodes.TAIL, 0.0);
    }
    
    // Update all source values for homotopy step
    // sourceScale: 0→1 scales VCC, VEE, I_Tail, I_VAS, I_Bias
    // alpha: 0→1 controls R_Feedback = R_nom / max(alpha, ALPHA_MIN)
    // bleederG: conductance of bleeder resistor (1/R), 0 = bleeder off
    static void updateSources(Sources &src, double sourceScale, double alpha, double bleederG) {
        // Scale supplies
        src.Vcc->setVoltage(VCC_NOM * sourceScale);
        src.Vee->setVoltage(VEE_NOM * sourceScale);
        
        // Scale current sources
        src.I_Tail->setCurrent(I_TAIL_NOM * sourceScale);
        src.I_VAS->setCurrent(I_VAS_NOM * sourceScale);
        src.I_BiasChain->setCurrent(I_BIAS_NOM * sourceScale);
        
        // Set feedback resistance
        double r_fb = R_FB_NOMINAL / max(alpha, ALPHA_MIN);
        src.R_Feedback->setResistance(r_fb);
        
        // Set bleeder resistance (high resistance = low conductance = off)
        // bleederG is conductance in Siemens. R = 1/G.
        // For bleederG=0, we set a very high resistance (effectively open)
        double r_bleeder = (bleederG > 1e-15) ? (1.0 / bleederG) : 1e15;
        src.R_Bleeder->setResistance(r_bleeder);
    }
    
    // Compute diagnostic values
    struct Diagnostics {
        double V_OUT, V_INM, V_INP, V_C2, V_C1, V_TAIL;
        double VBE_Q2;
        double finalGmin;
        bool isLatched;
        
        void print() const {
            cout << fixed << setprecision(4);
            cout << "  V(OUT)=" << V_OUT << " V(INM)=" << V_INM 
                 << " V(C2)=" << V_C2 << " VBE_Q2=" << VBE_Q2;
            cout << " Gmin=" << scientific << setprecision(1) << finalGmin;
            if (isLatched) cout << " [LATCH]";
            cout << endl;
            cout.flush();
        }
    };
    
    static Diagnostics getDiagnostics(const vector<double> &x, const Nodes &nodes, double finalGmin) {
        Diagnostics d;
        d.V_OUT = x[nodes.OUT];
        d.V_INM = x[nodes.INM];
        d.V_INP = x[nodes.INP];
        d.V_C2 = x[nodes.C2];
        d.V_C1 = x[nodes.C1];
        d.V_TAIL = x[nodes.TAIL];
        d.VBE_Q2 = x[nodes.Q2_Bi] - x[nodes.Q2_Ei];
        d.finalGmin = finalGmin;
        
        // Latch detection during ramp: focus on physical symptoms, not Gmin
        // Q2 deep cutoff (VBE < 0.1V) or nC2 pinned to VCC or OUT severely railed
        d.isLatched = (d.VBE_Q2 < 0.1) || (d.V_C2 > 14.0) || (abs(d.V_OUT) > 8.0);
        
        return d;
    }
    
    // Check if solution is acceptable (strict criteria)
    static bool isAcceptable(const Diagnostics &d) {
        return !d.isLatched && 
               (d.finalGmin <= 1e-12) && 
               (d.VBE_Q2 > 0.45 && d.VBE_Q2 < 0.85) &&
               (abs(d.V_OUT) < 2.0);
    }
};
