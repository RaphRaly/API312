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
// API 2520 Homotopy Builder - v2 with INM CLAMP
// =============================================================================
// PATRON'S ANALYSIS:
// 1. Old bleeder on nC2→VEE was WRONG: it pulled C2 negative, causing Q4 overdrive
//    and OUT to saturate negative.
// 2. The correct strategy is to CLAMP INM close to INM_Ref (=0) at startup.
//    This keeps Q2 conducting, which prevents the latch cascade:
//    Q2 on → nC2 controlled → Q4 balanced → OUT stable
// 3. Never open the feedback completely (alpha_min > 0)
// 4. Use PT (pseudo-transient) then DC at each step
//
// Homotopy elements:
//   - G_clamp_INM: conductance from INM to INM_Ref (strong at start, fades to 0)
//   - R_Feedback: controlled by alpha (never fully open)
// =============================================================================

class Api2520HomotopyV2 {
public:
    static constexpr double ALPHA_MIN = 0.01;  // Never fully open fb (was 1e-6, now 0.01)
    static constexpr double R_FB_NOMINAL = 10000.0;  // 10kΩ
    
    struct Sources {
        VoltageSource* Vcc;
        VoltageSource* Vee;
        VoltageSource* VinP;
        VoltageSource* VinM;
        CurrentSource* I_Tail;
        CurrentSource* I_VAS;
        CurrentSource* I_BiasChain;
        Resistor* R_Feedback;
        Resistor* R_Clamp_INM;  // INM clamp (replaces bad bleeder)
    };
    
    struct Nodes {
        NodeIndex VCC, VEE, INP, INM, INM_Ref, OUT;
        NodeIndex C1, C2, C4, E4, E5, VAS;
        NodeIndex B_HI, B_LO, TAIL;
        NodeIndex Q2_Bi, Q2_Ei, Q2_Ci;
        NodeIndex Q9_Bi, Q9_Ei, Q9_Ci;
        NodeIndex Q4_Bi, Q4_Ei, Q4_Ci;
    };
    
    static constexpr double VCC_NOM = 15.0;
    static constexpr double VEE_NOM = 15.0;
    static constexpr double I_TAIL_NOM = 200.0e-6;
    static constexpr double I_VAS_NOM = 1.5e-3;
    static constexpr double I_BIAS_NOM = 0.5e-3;
    
    static void build(Circuit &c, Sources &src, Nodes &nodes) {
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
        NodeIndex nB4 = nodes.C2;

        // === SUPPLIES ===
        src.Vcc = &c.addElement<VoltageSource>("Vcc_Src", nodes.VCC, GND, 0.0);
        src.Vee = &c.addElement<VoltageSource>("Vee_Src", GND, nodes.VEE, 0.0);
        src.VinP = &c.addElement<VoltageSource>("VinP", nodes.INP, GND, 0.0);
        src.VinM = &c.addElement<VoltageSource>("VinM", nodes.INM_Ref, GND, 0.0);

        // === FEEDBACK ===
        // R_fb = R_nom / alpha. At alpha=0.01, R_fb = 1MΩ
        src.R_Feedback = &c.addElement<Resistor>("R_Feedback", nodes.OUT, nodes.INM, 
                                                   R_FB_NOMINAL / ALPHA_MIN);
        c.addElement<Resistor>("R_Input", nodes.INM, nodes.INM_Ref, 10000.0);
        c.addElement<Resistor>("R_INP_Bias", nodes.INP, GND, 10.0e6);
        c.addElement<Resistor>("R_INM_Ref_Bias", nodes.INM_Ref, GND, 10.0e6);

        // === INM CLAMP (replaces bad bleeder) ===
        // This clamps INM to INM_Ref (≈0V) to prevent Q2 from going into cutoff
        // Strong at start (low R = high G), fades to very high R (low G) at end
        src.R_Clamp_INM = &c.addElement<Resistor>("R_Clamp_INM", nodes.INM, nodes.INM_Ref, 100.0);

        // === SIGNAL MIRROR ===
        auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
        addBjtExtended(c, nodes.C1, nodes.C1, nE3, q_pnp, true, "Q3");
        c.addElement<Resistor>("R_Deg3", nodes.VCC, nE3, 100.0);
        auto q9_nodes = addBjtExtended(c, nodes.C2, nodes.C1, nE9, q_pnp, true, "Q9");
        nodes.Q9_Bi = q9_nodes.b_int;
        nodes.Q9_Ei = q9_nodes.e_int;
        nodes.Q9_Ci = q9_nodes.c_int;
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
        // I_Tail: current flows from TAIL node toward VEE (sinks current from diff pair)
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
        // I_VAS: current flows from VCC into VAS node
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
        // I_BiasChain: sinks current from B_LO toward VEE
        src.I_BiasChain = &c.addElement<CurrentSource>("I_BiasChain_Sink", nodes.B_LO, nodes.VEE, 0.0);

        // === OUTPUT STAGE ===
        auto q7_params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
        addBjtExtended(c, nodes.VCC, nodes.B_HI, nE7, q7_params, false, "Q7");
        c.addElement<Resistor>("R_Out7", nE7, nodes.OUT, 0.47);
        auto q8_params = spiceToBjtParams(getSpiceBjtModel("2N4036"));
        addBjtExtended(c, nodes.VEE, nodes.B_LO, nE8, q8_params, true, "Q8");
        c.addElement<Resistor>("R_Out8", nE8, nodes.OUT, 0.47);
        c.addElement<Resistor>("R_Load", nodes.OUT, GND, 10000.0);

        // === DC BLEEDER RESISTORS ===
        // Ultra-high resistance paths to prevent quasi-floating nodes when Gmin drops.
        // Without these, current sources (ideal, infinite impedance) leave nodes
        // referenced only by Gmin, causing matrix singularity at low Gmin.
        // At 15V: ~15pA per bleeder - negligible for bias, but eliminates singularity.
        c.addElement<Resistor>("R_TAIL_BLEED", nodes.TAIL, nodes.VEE, 1e12);
        c.addElement<Resistor>("R_VAS_BLEED", nodes.VAS, nodes.VCC, 1e12);
        c.addElement<Resistor>("R_BLO_BLEED", nodes.B_LO, nodes.VEE, 1e12);

        // === NODESETS ===
        c.setNodeset(nodes.OUT, 0.0);
        c.setNodeset(nodes.VAS, 0.5);
        c.setNodeset(nodes.B_HI, 0.65);
        c.setNodeset(nodes.B_LO, -0.65);
        c.setNodeset(nodes.C4, -13.0);
        c.setNodeset(nodes.E5, -13.8);
        c.setNodeset(nodes.E4, 13.5);
        c.setNodeset(nodes.C2, 12.0);  // Not VCC!
        c.setNodeset(nodes.C1, 12.0);
        c.setNodeset(nodes.INM, 0.0);
        c.setNodeset(nodes.INP, 0.0);
        c.setNodeset(nodes.TAIL, -14.3);
    }
    
    // Update sources with homotopy parameters
    // s: source scale 0→1
    // alpha: feedback strength (alpha >= ALPHA_MIN always)
    // clampG: INM clamp conductance (strong at start, fades to 0)
    static void updateSources(Sources &src, double s, double alpha, double clampG) {
        src.Vcc->setVoltage(VCC_NOM * s);
        src.Vee->setVoltage(VEE_NOM * s);
        
        src.I_Tail->setCurrent(I_TAIL_NOM * s);
        src.I_VAS->setCurrent(I_VAS_NOM * s);
        src.I_BiasChain->setCurrent(I_BIAS_NOM * s);
        
        // Feedback: R_fb = R_nom / alpha
        double alpha_eff = max(alpha, ALPHA_MIN);
        src.R_Feedback->setResistance(R_FB_NOMINAL / alpha_eff);
        
        // INM clamp: R = 1/G (high G = strong clamp = low R)
        double r_clamp = (clampG > 1e-12) ? (1.0 / clampG) : 1e15;
        src.R_Clamp_INM->setResistance(r_clamp);
    }
    
    struct Diagnostics {
        double V_OUT, V_INM, V_INP, V_C2, VBE_Q2;
        double V_Q2_Ci, V_Q9_Ci;
        double finalGmin;
        bool isLatched;
        
        void print() const {
            cout << fixed << setprecision(4);
            cout << "OUT=" << V_OUT << " INM=" << V_INM << " C2=" << V_C2 
                 << " VBE=" << VBE_Q2;
            cout << " Q2_Ci=" << V_Q2_Ci << " Q9_Ci=" << V_Q9_Ci;
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
        d.VBE_Q2 = x[nodes.Q2_Bi] - x[nodes.Q2_Ei];
        d.V_Q2_Ci = x[nodes.Q2_Ci];
        d.V_Q9_Ci = x[nodes.Q9_Ci];
        d.finalGmin = finalGmin;
        
        // Latch criteria: Q2 deep cutoff OR C2 at VCC OR OUT severely railed
        d.isLatched = (d.VBE_Q2 < 0.1) || (d.V_C2 > 14.0) || (abs(d.V_OUT) > 10.0);
        
        return d;
    }
    
    // Strict acceptance for final solution
    static bool isAcceptable(const Diagnostics &d) {
        return (d.finalGmin <= 1e-12) && 
               (d.VBE_Q2 > 0.45 && d.VBE_Q2 < 0.85) &&
               (abs(d.V_OUT) < 2.0) &&
               (d.V_C2 < 14.0);
    }
};
