#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// CORRECTED KCL AUDIT: Separate audits for nVAS and Q5_Ci
// Q5_RC (0.5Ω) connects nVAS to Q5_Ci - must account for this!

int main() {
    cout << "========================================" << endl;
    cout << "  CORRECTED KCL AUDIT (with Q5_RC)" << endl;
    cout << "========================================" << endl;
    
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
    
    cout << "\nQ5 (TIS98) parasitic resistors:" << endl;
    cout << "  RC = " << q_vas.RC << " Ohm" << endl;
    cout << "  RB = " << q_vas.RB << " Ohm" << endl;
    cout << "  RE = " << q_vas.RE << " Ohm" << endl;
    
    // Q4 PNP
    addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    // Q5 NPN - returns internal nodes
    auto q5_nodes = addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    
    c.addElement<VoltageSource>("V_Drive", nB4, GND, 14.0);
    
    vector<double> x;
    c.solveDc(x);
    double finalGmin = c.getFinalGmin();
    
    double v_vas = x[nVAS];
    double v_c4 = x[nC4];
    double v_q5_ci = x[q5_nodes.c_int];
    double v_q5_bi = x[q5_nodes.b_int];
    double v_q5_ei = x[q5_nodes.e_int];
    
    cout << "\n--- NODE VOLTAGES ---" << endl;
    cout << "  V(VAS_Out)  = " << v_vas << " V (external collector)" << endl;
    cout << "  V(Q5_Ci)    = " << v_q5_ci << " V (internal collector)" << endl;
    cout << "  V(Q5_Bi)    = " << v_q5_bi << " V (internal base)" << endl;
    cout << "  V(Q5_Ei)    = " << v_q5_ei << " V (internal emitter)" << endl;
    cout << "  V(C4)       = " << v_c4 << " V" << endl;
    cout << "  Final Gmin  = " << finalGmin << " S" << endl;
    
    // ========================================
    // KCL AUDIT AT nVAS (external collector)
    // ========================================
    cout << "\n========================================" << endl;
    cout << "  KCL AT nVAS (external collector)" << endl;
    cout << "========================================" << endl;
    cout << "Convention: + = enters node, - = leaves node" << endl;
    
    double i_vas = 1.5e-3;  // CurrentSource VCC->VAS: enters VAS
    double i_miller = (v_vas - v_c4) / 1.0e7;  // Leaves VAS toward C4
    double i_q5_rc = (v_vas - v_q5_ci) / q_vas.RC;  // Leaves VAS toward Q5_Ci
    double i_gmin_vas = finalGmin * v_vas;  // Gmin shunt
    
    cout << fixed << setprecision(6);
    cout << "  I_VAS        = +" << i_vas * 1e3 << " mA (enters)" << endl;
    cout << "  I_Miller_DC  = -" << i_miller * 1e6 << " uA (leaves to C4)" << endl;
    cout << "  I_Q5_RC      = -" << i_q5_rc * 1e3 << " mA (leaves to Q5_Ci)" << endl;
    cout << "  I_Gmin(VAS)  = -" << i_gmin_vas * 1e3 << " mA (leaves)" << endl;
    
    double kcl_vas = i_vas - i_miller - i_q5_rc - i_gmin_vas;
    cout << "\n  KCL balance: " << kcl_vas * 1e6 << " uA (should be ~0)" << endl;
    
    // ========================================
    // KCL AUDIT AT Q5_Ci (internal collector)
    // ========================================
    cout << "\n========================================" << endl;
    cout << "  KCL AT Q5_Ci (internal collector)" << endl;
    cout << "========================================" << endl;
    
    // Current from Q5_RC (enters Q5_Ci from VAS)
    double i_q5_rc_in = i_q5_rc;  // Same magnitude, enters Q5_Ci
    
    // BJT collector current (Ebers-Moll)
    double Vbe = v_q5_bi - v_q5_ei;
    double Vbc = v_q5_bi - v_q5_ci;
    double Vce = v_q5_ci - v_q5_ei;
    
    cout << "  Vbe = " << Vbe << " V" << endl;
    cout << "  Vbc = " << Vbc << " V" << endl;
    cout << "  Vce = " << Vce << " V" << endl;
    
    double expBE = min(exp(Vbe / q_vas.nVt), 1e15);  // Limit for display
    double expBC = exp(Vbc / q_vas.nVt);
    double I_tran = q_vas.Is * (expBE - expBC);
    double I_bc_diode = (q_vas.Is / q_vas.betaR) * (expBC - 1.0);
    double Ic_bjt = I_tran - I_bc_diode;  // BJT collector current
    
    // For NPN: positive Ic means current flows INTO collector from external
    // So from KCL perspective at Q5_Ci: Ic_bjt LEAVES Q5_Ci (into emitter path)
    
    double i_gmin_q5ci = finalGmin * v_q5_ci;
    
    cout << "\n  I_Q5_RC (from VAS) = +" << i_q5_rc_in * 1e3 << " mA (enters)" << endl;
    cout << "  I_BJT_collector   = -" << Ic_bjt * 1e3 << " mA (leaves to E)" << endl;
    cout << "  I_Gmin(Q5_Ci)     = -" << i_gmin_q5ci * 1e3 << " mA (leaves)" << endl;
    
    double kcl_q5ci = i_q5_rc_in - Ic_bjt - i_gmin_q5ci;
    cout << "\n  KCL balance: " << kcl_q5ci * 1e6 << " uA (should be ~0)" << endl;
    
    // ========================================
    // SUMMARY
    // ========================================
    cout << "\n========================================" << endl;
    cout << "  SUMMARY" << endl;
    cout << "========================================" << endl;
    
    double total_gmin = i_gmin_vas + i_gmin_q5ci;
    double gmin_pct = total_gmin / i_vas * 100;
    
    cout << "Total Gmin absorption: " << total_gmin * 1e3 << " mA" << endl;
    cout << "As % of I_VAS: " << gmin_pct << "%" << endl;
    cout << "\nQ5 BJT Ic = " << Ic_bjt * 1e3 << " mA (what Q5 can sink)" << endl;
    cout << "I_VAS = " << i_vas * 1e3 << " mA (what's forced in)" << endl;
    cout << "Deficit = " << (i_vas - Ic_bjt) * 1e3 << " mA (must be absorbed elsewhere)" << endl;
    
    if (gmin_pct > 30) {
        cout << "\n*** CONFIRMED: Solution is Gmin-contaminated! ***" << endl;
        cout << "The 54V voltage is an artifact of Gmin absorbing excess current." << endl;
        cout << "Isolated VAS test is non-physical without a proper DC load." << endl;
    }
    
    return 0;
}
