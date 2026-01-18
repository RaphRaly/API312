#pragma once
#include "bjt_ebers_moll.h"
#include "capacitor_trap.h"
#include "circuit.h"
#include "current_source.h"
#include "diode_shockley_nr.h"
#include "resistor.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <iostream>

using namespace std;

// API 2520 Builder - Step D: Final Fine-Tuning & Parasitics
class Api2520Builder {
public:
  static constexpr double VCC = 15.0;
  static constexpr double VEE = -15.0;

  static void build2520(Circuit &c, double supplyVoltage = 15.0) {
    double vcc_val = abs(supplyVoltage);
    // Nodes
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    NodeIndex nGND = GND;
    NodeIndex nINP = c.createNode("INP");
    NodeIndex nINM = c.createNode("INM");
    NodeIndex nINM_Ref = c.createNode("INM_Ref"); // Stabilized reference node
    NodeIndex nOUT = c.createNode("OUT");
    NodeIndex nTAIL = c.createNode("Diff_Tail");
    NodeIndex nC1 = c.createNode("Q1_Col");
    NodeIndex nC2 = c.createNode("Q2_Col");
    NodeIndex nE1 = c.createNode("Q1_Emit");
    NodeIndex nE2 = c.createNode("Q2_Emit");
    NodeIndex nE3 = c.createNode("Q3_Emit");
    NodeIndex nE9 = c.createNode("Q9_Emit");
    NodeIndex nB4 = nC2;
    NodeIndex nE4 = c.createNode("Q4_Emit");
    NodeIndex nC4 = c.createNode("Q4_Col");
    NodeIndex nVAS = c.createNode("VAS_Col");
    NodeIndex nE5 = c.createNode("Q5_Emit");
    NodeIndex nB_HI = c.createNode("VAS_High");
    NodeIndex nB_LO = c.createNode("VAS_Low");
    NodeIndex nE7 = c.createNode("Q7_Emit");
    NodeIndex nE8 = c.createNode("Q8_Emit");

    // Supplies
    c.addElement<VoltageSource>("Vcc_Src", nVCC, nGND, vcc_val);
    c.addElement<VoltageSource>("Vee_Src", nGND, nVEE, vcc_val);
    vinP_ref = &c.addElement<VoltageSource>("VinP", nINP, nGND, 0.0);
    vinM_ref = &c.addElement<VoltageSource>("VinM", nINM_Ref, nGND, 0.0);
    out_node = nOUT;

    // ----- FEEDBACK (Gain = 2) -----
    // Standard inverting feedback configuration
    c.addElement<Resistor>("R_Feedback", nOUT, nINM, 10000.0);
    c.addElement<Resistor>("R_Input", nINM, nINM_Ref, 10000.0);

    // Symmetrical bias resistors for DC stability
    // These provide DC paths without unbalancing the diff pair
    c.addElement<Resistor>("R_INP_Bias", nINP, nGND, 10.0e6); // 10MΩ to GND
    c.addElement<Resistor>("R_INM_Ref_Bias", nINM_Ref, nGND,
                           10.0e6); // 10MΩ to GND

    // ----- SIGNAL MIRROR (Active Load) -----
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    // Master on Q1
    addBjtExtended(c, nC1, nC1, nE3, q_pnp, true, "Q3");
    c.addElement<Resistor>("R_Deg3", nVCC, nE3, 100.0);
    // Slave on Q2
    addBjtExtended(c, nC2, nC1, nE9, q_pnp, true, "Q9");
    c.addElement<Resistor>("R_Deg9", nVCC, nE9, 100.0);

    // ----- INPUT STAGE (with Degeneration) -----
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    addBjtExtended(c, nC1, nINP, nE1, q_npn, false, "Q1");
    addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2");

    c.addElement<Resistor>("R_E1", nE1, nTAIL, 100.0);
    c.addElement<Resistor>("R_E2", nE2, nTAIL, 100.0);
    c.addElement<CurrentSource>("I_Tail", nTAIL, nVEE, 200.0e-6);

    // ----- VAS -----
    addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);

    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    c.addElement<CapacitorTrap>("C_Miller", nVAS, nC4, 220.0e-12);

    // ----- VBE MULTIPLIER (Tuned for ~1.3V) -----
    NodeIndex nB_Mult = c.createNode("D_Bias_mid");
    auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    addBjtExtended(c, nB_HI, nB_Mult, nB_LO, q_mult, false, "Q_Bias");
    c.addElement<Resistor>("R1_Mult", nB_HI, nB_Mult, 1800.0);
    c.addElement<Resistor>("R2_Mult", nB_Mult, nB_LO, 1000.0);

    c.addElement<Resistor>("R_VAS_HI", nVAS, nB_HI, 200.0);
    c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 0.5e-3);

    // ----- OUTPUT STAGE -----
    auto q7_params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
    addBjtExtended(c, nVCC, nB_HI, nE7, q7_params, false, "Q7");
    c.addElement<Resistor>("R_Out7", nE7, nOUT, 0.47);
    auto q8_params = spiceToBjtParams(getSpiceBjtModel("2N4036"));
    addBjtExtended(c, nVEE, nB_LO, nE8, q8_params, true, "Q8");
    c.addElement<Resistor>("R_Out8", nE8, nOUT, 0.47);

    c.addElement<Resistor>("R_Load", nOUT, nGND, 10000.0);

    // === ANTI-LATCH STABILIZERS REMOVED ===
    // Relying on solveDcPseudoTransient() instead of circuit modification.

    // NODESETS: Guide Newton to the correct DC operating point
    // Without these, the solver may converge to a saturated/cutoff state
    c.setNodeset(nOUT, 0.0); // Output should be near 0V
    c.setNodeset(nC1,
                 vcc_val - 1.0); // Q1/Q2 collectors near VCC-1V (mirror active)
    c.setNodeset(nC2, vcc_val - 1.0);
    c.setNodeset(nVAS, 0.0);   // VAS output near 0V
    c.setNodeset(nB_HI, 0.5);  // Output stage bias ~0.5V above output
    c.setNodeset(nB_LO, -0.5); // Output stage bias ~0.5V below output

    cout << "2520 built (Step D: Final Parasitics & Tuning)" << endl;
  }

  static VoltageSource *getVinP() { return vinP_ref; }
  static VoltageSource *getVinM() { return vinM_ref; }
  static NodeIndex getOutputNode() { return out_node; }

private:
  static inline VoltageSource *vinP_ref = nullptr;
  static inline VoltageSource *vinM_ref = nullptr;
  static inline NodeIndex out_node = -1;
};
