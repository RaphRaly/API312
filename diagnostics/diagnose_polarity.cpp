#include "circuit.h"
#include "bjt_ebers_moll.h"
#include "spice_models_2520.h"
#include "resistor.h"
#include "current_source.h"
#include "capacitor_trap.h"
#include "voltage_source.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>

using namespace std;

ofstream out;

// Helper to get V from solution
double getV(const Circuit &c, const vector<double> &x, const string &name) {
    for (const auto &pair : c.getNodeNames()) {
        if (pair.second == name) return x[pair.first];
    }
    return 999.999;
}

void auditNode(const Circuit& c, const vector<double>& x, const string& nodeName, int targetNode) {
    if (targetNode == GND) return; // Don't audit GND
    out << "\n--- AUDIT FOR NODE: " << nodeName << " (" << (targetNode < x.size() ? x[targetNode] : 0.0) << " V) ---" << endl;
    out << left << setw(20) << "Element" << setw(15) << "Type" << "Connection/Current" << endl;
    
    double totalCurrent = 0.0;
    double nodeV = (targetNode < x.size()) ? x[targetNode] : 0.0;
    
    // Gmin Leakage
    // MUST 1: Explicit Gmin accounting
    double gmin = c.getFinalGmin();
    double i_gmin = nodeV * gmin;
    out << setw(20) << "GMIN_SHUNT" << setw(15) << "SYSTEM"
        << "G=" << gmin << " S, I_leave=" << i_gmin * 1e3 << " mA" << endl;
    totalCurrent += i_gmin;

    for (const auto& e : c.elements) {
        // Resistor
        if (auto* r = dynamic_cast<Resistor*>(e.get())) {
            if (r->getNa() == targetNode || r->getNb() == targetNode) {
                int otherNode = (r->getNa() == targetNode) ? r->getNb() : r->getNa();
                double otherV = (otherNode == GND) ? 0.0 : ((otherNode < x.size()) ? x[otherNode] : 0.0);
                
                double current = (nodeV - otherV) / r->getR();
                
                out << setw(20) << c.getElementName(e.get()) << setw(15) << "RESISTOR"
                          << "R=" << r->getR() << ", To=" << (otherNode==GND ? "GND" : (c.getNodeNames().count(otherNode) ? c.getNodeNames().at(otherNode) : to_string(otherNode)))
                          << " (" << otherV << "V), I_leave=" << current * 1e3 << " mA" << endl;
                 totalCurrent += current;
            }
        }
        // Current Source
        else if (auto* cs = dynamic_cast<CurrentSource*>(e.get())) {
             if (cs->getNa() == targetNode) {
                 // Current starts here, so it leaves.
                 out << setw(20) << c.getElementName(e.get()) << setw(15) << "ISOURCE"
                           << "I=" << cs->getI()*1e3 << "mA (Source LEAVES node)" << endl;
                 totalCurrent += cs->getI();
             } else if (cs->getNb() == targetNode) {
                 // Current ends here, so it enters. Leaving = -I.
                 out << setw(20) << c.getElementName(e.get()) << setw(15) << "ISOURCE"
                           << "I=" << cs->getI()*1e3 << "mA (Source ENTERS node)" << endl;
                 totalCurrent -= cs->getI();
             }
        }
        // BJT NPN
        else if (auto* bjt = dynamic_cast<BjtNpnEbersMoll*>(e.get())) {
            string term;
            if (bjt->getC() == targetNode) term = "COLLECTOR";
            else if (bjt->getB() == targetNode) term = "BASE";
            else if (bjt->getE() == targetNode) term = "EMITTER";
            
            if (!term.empty()) {
                out << setw(20) << c.getElementName(e.get()) << setw(15) << "NPN BJT"
                          << "Connected at " << term << endl;
            }
        }
        // BJT PNP
        else if (auto* bjt = dynamic_cast<BjtPnpEbersMoll*>(e.get())) {
            string term;
            if (bjt->getC() == targetNode) term = "COLLECTOR";
            else if (bjt->getB() == targetNode) term = "BASE";
            else if (bjt->getE() == targetNode) term = "EMITTER";
             if (!term.empty()) {
                out << setw(20) << c.getElementName(e.get()) << setw(15) << "PNP BJT"
                          << "Connected at " << term << endl;
            }
        }
    }
    out << "Sum of Currents (should be 0): " << totalCurrent * 1e3 << " mA" << endl;
}

int main() {
    out.open("polarity_test.txt");
    
    auto runSim = [](double vin_val, const string& label) -> double {
        Circuit c;
        // ... (existing setup code is fine, will be rebuilt) ...
        double vcc_val = 15.0;
        NodeIndex nGND = GND;
        NodeIndex nVCC = c.createNode("VCC");
        NodeIndex nVEE = c.createNode("VEE");
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
        NodeIndex nVAS = c.createNode("VAS_Col");
        NodeIndex nC4 = c.createNode("Q4_Col");
        NodeIndex nB4 = nC2;
        NodeIndex nE4 = c.createNode("Q4_Emit");
        NodeIndex nE5 = c.createNode("Q5_Emit");
        NodeIndex nB_HI = c.createNode("VAS_High");
        NodeIndex nB_LO = c.createNode("VAS_Low");
        NodeIndex nE7 = c.createNode("Q7_Emit");
        NodeIndex nE8 = c.createNode("Q8_Emit");

        c.addElement<VoltageSource>("Vcc_Src", nVCC, nGND, vcc_val);
        c.addElement<VoltageSource>("Vee_Src", nGND, nVEE, vcc_val); 
        c.addElement<VoltageSource>("VinP_Step", nINP, nGND, vin_val);

        // Standard inverting feedback configuration
        c.addElement<Resistor>("R_Feedback", nOUT, nINM, 10000.0);
        c.addElement<Resistor>("R_Input", nINM, nINM_Ref, 10000.0);
        
        // Symmetrical bias resistors for DC stability
        c.addElement<Resistor>("R_INP_Bias", nINP, nGND, 10.0e6);
        c.addElement<Resistor>("R_INM_Ref_Bias", nINM_Ref, nGND, 10.0e6);

        auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
        addBjtExtended(c, nC1, nC1, nE3, q_pnp, true, "Q3");
        c.addElement<Resistor>("R_Deg3", nVCC, nE3, 10.0);
        addBjtExtended(c, nC2, nC1, nE9, q_pnp, true, "Q9");
        c.addElement<Resistor>("R_Deg9", nVCC, nE9, 10.0);

        auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
        addBjtExtended(c, nC1, nINP, nE1, q_npn, false, "Q1");
        addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2");
        c.addElement<Resistor>("R_E1", nE1, nTAIL, 10.0);
        c.addElement<Resistor>("R_E2", nE2, nTAIL, 10.0);
        c.addElement<CurrentSource>("I_Tail", nTAIL, nVEE, 2000.0e-6);

        addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
        c.addElement<Resistor>("R_E4", nVCC, nE4, 700.0);
        c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e6); 
        c.addElement<CapacitorTrap>("C_Miller", nVAS, nC4, 220.0e-12);
        c.addElement<Resistor>("R_C4", nC4, nVEE, 1000.0);

        auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
        addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
        c.addElement<Resistor>("R_E5", nE5, nVEE, 120.0); 
        c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 15.0e-3);

        NodeIndex nB_Mult = c.createNode("D_Bias_mid");
        auto q_mult = spiceToBjtParams(getSpiceBjtModel("TIS98"));
        addBjtExtended(c, nB_HI, nB_Mult, nB_LO, q_mult, false, "Q_Bias");
        c.addElement<Resistor>("R1_Mult", nB_HI, nB_Mult, 180.0);
        c.addElement<Resistor>("R2_Mult", nB_Mult, nB_LO, 100.0);
        c.addElement<Resistor>("R_VAS_HI", nVAS, nB_HI, 20.0);
        c.addElement<CurrentSource>("I_BiasChain_Sink", nB_LO, nVEE, 5.0e-3);

        auto q7_params = spiceToBjtParams(getSpiceBjtModel("2N3053"));
        addBjtExtended(c, nVCC, nB_HI, nE7, q7_params, false, "Q7");
        c.addElement<Resistor>("R_Out7", nE7, nOUT, 0.047); 
        auto q8_params = spiceToBjtParams(getSpiceBjtModel("2N4036"));
        addBjtExtended(c, nVEE, nB_LO, nE8, q8_params, true, "Q8");
        c.addElement<Resistor>("R_Out8", nE8, nOUT, 0.047);
        c.addElement<Resistor>("R_Load", nOUT, nGND, 1000.0); 

        c.finalize();
        vector<double> x;
        
        // MUST 3: Use Pseudo-Transient
        // This attempts to settle to DC using time-stepping
        c.solveDcPseudoTransient(x, 1e-4, 1e-8); 
        
        if (vin_val == 0.0) {
             out << "\n=== DETAILED CONNECTIVITY AUDIT (INP=0V, Pseudo-Transient) ===" << endl;
             // Remove passed Gmin, use internal
             
             auditNode(c, x, "Q2_Col (Q4 Base)", nC2);
             auditNode(c, x, "Q4_Col (Q5 Base)", nC4);
             auditNode(c, x, "VAS_Col (Output Driver)", nVAS);
             auditNode(c, x, "Diff_Tail", nTAIL);
             
             out << "\n--- MIRROR DIAGNOSTICS ---" << endl;
             auditNode(c, x, "Q1_Col (Mirror Ref)", nC1);
             auditNode(c, x, "Q3_Emit", nE3);
             auditNode(c, x, "Q9_Emit", nE9);
        }
        
        return getV(c, x, "OUT");
    };

    double vOut0 = runSim(0.0, "0V");
    double vOut1 = runSim(0.001, "1mV");
    
    out << "=== DELTA GAIN TEST ===" << endl;
    out << "OUT(0V)  = " << vOut0 << " V" << endl;
    out << "OUT(1mV) = " << vOut1 << " V" << endl;
    double gain = (vOut1 - vOut0) / 0.001;
    out << "Open Loop DC Gain = " << gain << endl;
    out << "Polarity: " << (gain > 0 ? "NON-INVERTING (+)" : "INVERTING (-)") << endl;
    out << "CONCLUSION: " << (gain > 0 ? "Negative Feedback Compatible" : "POSITIVE FEEDBACK ERROR") << endl;

    // Run detailed diagnostics for 0V case to see offset cause
    out << "\n--- Offset Diags (0V Input) ---" << endl;
    // ... (re-run runSim logic or just trust simple output here for now)
    
    out.close();
    cout << "Analysis written to polarity_test.txt" << endl;
    return 0;
}
