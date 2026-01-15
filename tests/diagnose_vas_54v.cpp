#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// Diagnose why VAS_Out goes to 54V
// Key question: Is Q5 (NPN) conducting? What is its collector current?

int main() {
    cout << "=== VAS_Out 54V DIAGNOSIS ===" << endl;
    cout << "Building isolated VAS block and analyzing Q5 NPN" << endl;
    
    Circuit c;
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    NodeIndex nC4 = c.createNode("C4");
    NodeIndex nE4 = c.createNode("E4");
    NodeIndex nB4 = c.createNode("B4");
    NodeIndex nVAS = c.createNode("VAS_Out");
    NodeIndex nE5 = c.createNode("E5");
    
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    
    // Note the BJT params
    cout << "\nQ5 (TIS98) parameters:" << endl;
    cout << "  Is = " << q_vas.Is << " A" << endl;
    cout << "  betaF = " << q_vas.betaF << endl;
    cout << "  nVt = " << q_vas.nVt << " V" << endl;
    cout << "  VAF = " << q_vas.VAF << " V (Early voltage)" << endl;
    
    // Q4 PNP
    addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    // Q5 NPN - this is the problem area
    auto q5_nodes = addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    
    c.addElement<VoltageSource>("V_Drive", nB4, GND, 14.0);
    
    vector<double> x;
    c.solveDc(x);
    
    cout << "\n--- SOLUTION ---" << endl;
    double v_vas = x[nVAS];
    double v_c4 = x[nC4];
    double v_e5 = x[nE5];
    double v_q5_b = x[q5_nodes.b_int];
    double v_q5_c = x[q5_nodes.c_int];
    double v_q5_e = x[q5_nodes.e_int];
    
    cout << "V(VAS_Out) = " << v_vas << " V" << endl;
    cout << "V(C4) = " << v_c4 << " V" << endl;
    cout << "V(E5) = " << v_e5 << " V" << endl;
    
    cout << "\n--- Q5 NPN Internal Nodes ---" << endl;
    cout << "V(Q5_Bi) = " << v_q5_b << " V (internal base)" << endl;
    cout << "V(Q5_Ci) = " << v_q5_c << " V (internal collector)" << endl;
    cout << "V(Q5_Ei) = " << v_q5_e << " V (internal emitter)" << endl;
    
    double Vbe = v_q5_b - v_q5_e;
    double Vbc = v_q5_b - v_q5_c;
    double Vce = v_q5_c - v_q5_e;
    
    cout << "\n--- Q5 Junction Voltages ---" << endl;
    cout << "Vbe = " << Vbe << " V";
    if (Vbe > 0.5) cout << " (FORWARD BIASED - Q5 should be ON)";
    else if (Vbe < 0.3) cout << " (REVERSE/OFF - Q5 in cutoff!)";
    cout << endl;
    
    cout << "Vbc = " << Vbc << " V";
    if (Vbc > 0.5) cout << " (B-C forward - Q5 saturated)";
    else cout << " (B-C reverse - normal active region)";
    cout << endl;
    
    cout << "Vce = " << Vce << " V";
    if (Vce < 0.2) cout << " (near saturation)";
    else if (Vce < 0) cout << " (INVERTED!)";
    cout << endl;
    
    // Calculate expected Ic using Ebers-Moll
    double expBE = exp(Vbe / q_vas.nVt);
    double expBC = exp(Vbc / q_vas.nVt);
    double I_tran_expected = q_vas.Is * (expBE - expBC);
    double Ib_expected = (q_vas.Is / q_vas.betaF) * (expBE - 1.0);
    double Ic_expected = I_tran_expected - (q_vas.Is / q_vas.betaR) * (expBC - 1.0);
    
    cout << "\n--- Expected Q5 Currents (from Ebers-Moll) ---" << endl;
    cout << "I_tran = " << I_tran_expected * 1e3 << " mA" << endl;
    cout << "Ib = " << Ib_expected * 1e6 << " uA" << endl;
    cout << "Ic = " << Ic_expected * 1e3 << " mA" << endl;
    
    // KCL at VAS_Out node
    cout << "\n--- KCL at VAS_Out ---" << endl;
    cout << "I_VAS (from VCC): 1.5 mA INTO node" << endl;
    cout << "I_Q5_collector: " << Ic_expected * 1e3 << " mA (expected)" << endl;
    cout << "I_R_Miller: " << (v_vas - v_c4) / 1e7 * 1e6 << " uA" << endl;
    
    double i_gmin_equivalent = v_vas * c.getFinalGmin();
    cout << "I_Gmin (leakage via Gmin): " << i_gmin_equivalent * 1e3 << " mA" << endl;
    
    if (abs(Ic_expected) < 1e-3) {
        cout << "\n*** Q5 IS NOT SINKING ENOUGH CURRENT! ***" << endl;
        cout << "The 1.5mA from I_VAS has nowhere to go except Gmin leakage." << endl;
        cout << "This creates the 54V artifact." << endl;
    }
    
    return 0;
}
