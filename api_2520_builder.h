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


// API 2520 Builder - Step D: Final Fine-Tuning & Parasitics
class Api2520Builder {
public:
  static constexpr double VCC = 15.0;
  static constexpr double VEE = -15.0;

  static void build2520(Circuit &c, double supplyVoltage = 15.0) {
    double vcc_val = std::abs(supplyVoltage);
    double vee_val = -vcc_val;
    // Nodes
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    NodeIndex nGND = GND;
    NodeIndex nINP = c.createNode("INP");
    NodeIndex nINM = c.createNode("INM");
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
    out_node = nOUT;

    // ----- FEEDBACK (Gain = 2) -----
    c.addElement<Resistor>("R_Feedback", nOUT, nINM, 10000.0);
    c.addElement<Resistor>("R_Input", nINM, nGND, 10000.0);

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
    // Softeners for better matrix conditioning
    NodeIndex nC1_soft = c.createNode("Q1_Col_soft");
    NodeIndex nC2_soft = c.createNode("Q2_Col_soft");
    addBjtExtended(c, nC1_soft, nINP, nE1, q_npn, false, "Q1");
    addBjtExtended(c, nC2_soft, nINM, nE2, q_npn, false, "Q2");
    c.addElement<Resistor>("R_Soft1", nC1_soft, nC1, 100.0);
    c.addElement<Resistor>("R_Soft2", nC2_soft, nC2, 100.0);

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

    std::cout << "2520 built (Step D: Final Parasitics & Tuning)" << std::endl;
  }

  static VoltageSource *getVinP() { return vinP_ref; }
  static NodeIndex getOutputNode() { return out_node; }

private:
  static inline VoltageSource *vinP_ref = nullptr;
  static inline NodeIndex out_node = -1;
};
