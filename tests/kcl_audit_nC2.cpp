#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

using namespace std;

// KCL AUDIT ON nC2 (Q4 Base / Q2 Collector)
// Per patron's MUST requirement: 
// "à nC2, faire KCL complet à k=0 et à la solution latch,
//  lister les top 3 courants qui tirent nC2 vers VCC"

// This identifies WHO is pushing nC2 to 15V (latch)

int main() {
    cout << "========================================" << endl;
    cout << "  KCL AUDIT ON nC2 (Q4 Base)" << endl;
    cout << "========================================" << endl;
    cout << "\nObjective: Identify which element pushes nC2 to VCC" << endl;
    cout << "at the latched solution where V(nC2) = ~15V" << endl;
    
    Circuit c;
    Api2520Builder::build2520(c);
    
    // Get node indices
    const auto& names = c.getNodeNames();
    NodeIndex nC2 = -1, nC1 = -1, nVCC = -1, nVEE = -1, nINM = -1;
    NodeIndex nE9 = -1, nE2 = -1;
    
    // Also find internal nodes for Q9, Q2, Q4
    NodeIndex nQ9_Bi = -1, nQ9_Ci = -1, nQ9_Ei = -1;
    NodeIndex nQ2_Bi = -1, nQ2_Ci = -1, nQ2_Ei = -1;
    NodeIndex nQ4_Bi = -1;
    
    for (const auto& [idx, name] : names) {
        if (name == "Q2_Col") nC2 = idx;
        if (name == "Q1_Col") nC1 = idx;
        if (name == "VCC") nVCC = idx;
        if (name == "VEE") nVEE = idx;
        if (name == "INM") nINM = idx;
        if (name == "Q9_Emit") nE9 = idx;
        if (name == "Q2_Emit") nE2 = idx;
        // Internal nodes
        if (name == "Q9_Bi") nQ9_Bi = idx;
        if (name == "Q9_Ci") nQ9_Ci = idx;
        if (name == "Q9_Ei") nQ9_Ei = idx;
        if (name == "Q2_Bi") nQ2_Bi = idx;
        if (name == "Q2_Ci") nQ2_Ci = idx;
        if (name == "Q2_Ei") nQ2_Ei = idx;
        if (name == "Q4_Bi") nQ4_Bi = idx;
    }
    
    cout << "\nNode indices:" << endl;
    cout << "  nC2 (Q2_Col/Q4_Base) = " << nC2 << endl;
    cout << "  nC1 (Q1_Col/mirror) = " << nC1 << endl;
    cout << "  nQ9_Ci (Q9 internal collector) = " << nQ9_Ci << endl;
    cout << "  nQ2_Ci (Q2 internal collector) = " << nQ2_Ci << endl;
    cout << "  nQ4_Bi (Q4 internal base) = " << nQ4_Bi << endl;
    
    // Solve DC - this will likely latch
    vector<double> x;
    c.solveDc(x);
    double finalGmin = c.getFinalGmin();
    
    cout << "\n=== DC SOLUTION ===" << endl;
    cout << "Final Gmin: " << scientific << setprecision(2) << finalGmin << " S" << endl;
    cout << fixed << setprecision(4);
    cout << "V(VCC) = " << x[nVCC] << " V" << endl;
    cout << "V(VEE) = " << x[nVEE] << " V" << endl;
    cout << "V(C2)  = " << x[nC2] << " V   *** TARGET NODE ***" << endl;
    cout << "V(C1)  = " << x[nC1] << " V   (mirror reference)" << endl;
    cout << "V(INM) = " << x[nINM] << " V" << endl;
    
    // ======================================================
    // KCL AT nC2 (external node)
    // ======================================================
    // Which elements connect to nC2?
    // From api_2520_builder.h:
    // 1. Q9 collector (via internal node Q9_Ci if RC>0, else direct)
    //    Q9: addBjtExtended(c, nC2, nC1, nE9, q_pnp, true, "Q9")
    //    -> Q9 collector external = nC2
    // 2. Q2 collector (via internal node Q2_Ci if RC>0)
    //    Q2: addBjtExtended(c, nC2, nINM, nE2, q_npn, false, "Q2")
    //    -> Q2 collector external = nC2
    // 3. Q4 base (via internal node Q4_Bi if RB>0)
    //    Q4: addBjtExtended(c, nC4, nB4, nE4...) where nB4 = nC2
    //    -> Q4 base external = nC2
    
    cout << "\n========================================" << endl;
    cout << "  KCL AT nC2 (Q4 Base / Q2 Col / Q9 Col)" << endl;
    cout << "========================================" << endl;
    cout << "\nConnected elements at nC2:" << endl;
    cout << "  1. Q9 collector (PNP mirror slave) - via Q9_RC resistor" << endl;
    cout << "  2. Q2 collector (NPN diff pair) - via Q2_RC resistor" << endl;
    cout << "  3. Q4 base (PNP VAS driver) - via Q4_RB resistor" << endl;
    
    // Get BJT params to know parasitic resistor values
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_npn = spiceToBjtParams(getSpiceBjtModel("BC414C"));
    
    cout << "\nParasitic resistor values:" << endl;
    cout << "  Q9 (PNP) RC = " << q_pnp.RC << " Ohm" << endl;
    cout << "  Q2 (NPN) RC = " << q_npn.RC << " Ohm" << endl;
    cout << "  Q4 (PNP) RB = " << q_pnp.RB << " Ohm" << endl;
    
    // ======================================================
    // CURRENT CALCULATION
    // ======================================================
    cout << "\n=== CURRENTS AT nC2 ===" << endl;
    cout << "(Convention: + = enters nC2, - = leaves nC2)" << endl;
    
    double V_C2 = x[nC2];
    
    // 1. Q9 internal collector current (through Q9_RC)
    // If Q9_Ci exists, current = (V_Q9_Ci - V_C2) / Q9_RC
    double I_Q9_RC = 0;
    if (nQ9_Ci != -1 && q_pnp.RC > 0) {
        I_Q9_RC = (x[nQ9_Ci] - V_C2) / q_pnp.RC;
    } else {
        // Direct connection or no RC - need to compute differently
        // For now, estimate based on voltage
        I_Q9_RC = 0;  // Will show as unknown
    }
    
    // 2. Q2 internal collector current (through Q2_RC)
    double I_Q2_RC = 0;
    if (nQ2_Ci != -1 && q_npn.RC > 0) {
        I_Q2_RC = (x[nQ2_Ci] - V_C2) / q_npn.RC;
    }
    
    // 3. Q4 base current (through Q4_RB)
    double I_Q4_RB = 0;
    if (nQ4_Bi != -1 && q_pnp.RB > 0) {
        I_Q4_RB = (V_C2 - x[nQ4_Bi]) / q_pnp.RB;
    }
    
    // 4. Gmin shunt to ground
    double I_Gmin = finalGmin * V_C2;
    
    cout << fixed << setprecision(6);
    cout << "\nI_Q9_RC (Q9 coll->nC2): " << I_Q9_RC * 1e6 << " uA";
    if (nQ9_Ci != -1) {
        cout << " [V(Q9_Ci)=" << x[nQ9_Ci] << "V]";
    } else {
        cout << " [Q9_Ci node not found]";
    }
    cout << endl;
    
    cout << "I_Q2_RC (Q2 coll->nC2): " << I_Q2_RC * 1e6 << " uA";
    if (nQ2_Ci != -1) {
        cout << " [V(Q2_Ci)=" << x[nQ2_Ci] << "V]";
    } else {
        cout << " [Q2_Ci node not found]";
    }
    cout << endl;
    
    cout << "I_Q4_RB (nC2->Q4 base): " << I_Q4_RB * 1e6 << " uA";
    if (nQ4_Bi != -1) {
        cout << " [V(Q4_Bi)=" << x[nQ4_Bi] << "V]";
    } else {
        cout << " [Q4_Bi node not found]";
    }
    cout << endl;
    
    cout << "I_Gmin (nC2->GND):      " << I_Gmin * 1e6 << " uA" << endl;
    
    // ======================================================
    // IDENTIFY THE "TRACTOR" TO VCC
    // ======================================================
    cout << "\n========================================" << endl;
    cout << "  DIAGNOSIS: Who pulls nC2 to VCC?" << endl;  
    cout << "========================================" << endl;
    
    // If V(C2) is at VCC, the "tractor" is whoever injects current into nC2
    // with no place for it to go, OR whoever prevents current from leaving.
    
    if (V_C2 > 14.0) {
        cout << "\n*** V(C2) = " << V_C2 << "V is CLAMPED TO VCC ***" << endl;
        cout << "\nPotential causes:" << endl;
        
        // Check Q9 (mirror slave)
        if (nQ9_Ci != -1) {
            double V_Q9_Ci = x[nQ9_Ci];
            double V_Q9_Ei = (nQ9_Ei != -1) ? x[nQ9_Ei] : 15.0;
            double VEB_Q9 = V_Q9_Ei - (nQ9_Bi != -1 ? x[nQ9_Bi] : x[nC1]);
            cout << "\n1. Q9 (PNP mirror slave):" << endl;
            cout << "   V(Q9_Ci) = " << V_Q9_Ci << " V" << endl;
            cout << "   V(Q9_Ei) = " << V_Q9_Ei << " V" << endl;
            cout << "   VEB_Q9 = " << VEB_Q9 << " V";
            if (VEB_Q9 < 0.3) {
                cout << " *** CUTOFF! Q9 not conducting ***" << endl;
            } else if (VEB_Q9 > 0.5) {
                cout << " [ACTIVE]" << endl;
            } else {
                cout << " [MARGINAL]" << endl;
            }
        }
        
        // Check Q2 (diff pair NPN)
        if (nQ2_Ci != -1) {
            double V_Q2_Ci = x[nQ2_Ci];
            double V_Q2_Ei = (nQ2_Ei != -1) ? x[nQ2_Ei] : -15.0;
            double V_Q2_Bi = (nQ2_Bi != -1) ? x[nQ2_Bi] : x[nINM];
            double VBE_Q2 = V_Q2_Bi - V_Q2_Ei;
            cout << "\n2. Q2 (NPN diff pair):" << endl;
            cout << "   V(Q2_Ci) = " << V_Q2_Ci << " V" << endl;
            cout << "   V(Q2_Bi) = " << V_Q2_Bi << " V" << endl;
            cout << "   V(Q2_Ei) = " << V_Q2_Ei << " V" << endl;
            cout << "   VBE_Q2 = " << VBE_Q2 << " V";
            if (VBE_Q2 < 0.3) {
                cout << " *** CUTOFF! Q2 not conducting ***" << endl;
            } else if (VBE_Q2 > 0.5) {
                cout << " [ACTIVE]" << endl;
            } else {
                cout << " [MARGINAL]" << endl;
            }
        }
        
        // Check Q4 (VAS driver PNP)
        if (nQ4_Bi != -1) {
            // For PNP: base current flows OUT of base when active
            // If Q4 is cutoff, no base current
            cout << "\n3. Q4 (PNP VAS driver):" << endl;
            cout << "   V(Q4_Bi) = " << x[nQ4_Bi] << " V" << endl;
            cout << "   I_Q4_RB = " << I_Q4_RB * 1e6 << " uA";
            if (abs(I_Q4_RB) < 0.1e-6) {
                cout << " *** NO BASE CURRENT - Q4 is OFF ***" << endl;
            } else {
                cout << endl;
            }
        }
        
        // Summary
        cout << "\n=== KEY FINDING ===" << endl;
        cout << "At nC2:" << endl;
        
        // Sort currents by magnitude
        struct CurrentSource {
            string name;
            double current;
        };
        vector<CurrentSource> sources = {
            {"Q9_RC", I_Q9_RC},
            {"Q2_RC", I_Q2_RC},
            {"Q4_RB", -I_Q4_RB},  // Sign flip: leaving nC2 toward Q4
            {"Gmin", -I_Gmin}     // Sign flip: leaving nC2 toward GND
        };
        
        // Sort by absolute value
        sort(sources.begin(), sources.end(), [](const CurrentSource& a, const CurrentSource& b) {
            return abs(a.current) > abs(b.current);
        });
        
        cout << "\nTop currents (sorted by magnitude):" << endl;
        for (const auto& s : sources) {
            cout << "  " << s.name << ": " << s.current * 1e6 << " uA";
            cout << (s.current > 0 ? " (entering)" : " (leaving)") << endl;
        }
        
    } else {
        cout << "V(C2) = " << V_C2 << "V is NOT at VCC - solution may be correct!" << endl;
    }
    
    // ======================================================
    // MIRROR RATIO CHECK
    // ======================================================
    cout << "\n========================================" << endl;
    cout << "  MIRROR RATIO CHECK (Q3/Q9)" << endl;
    cout << "========================================" << endl;
    cout << "V(C1) = " << x[nC1] << " V (master)" << endl;
    cout << "V(C2) = " << V_C2 << " V (slave)" << endl;
    
    if (x[nC1] > 14.0 && V_C2 > 14.0) {
        cout << "\n*** BOTH C1 and C2 are at VCC! ***" << endl;
        cout << "The current mirror is NOT working." << endl;
        cout << "Check: diff pair is not driving the mirror." << endl;
    }
    
    return (V_C2 > 14.0) ? 1 : 0;
}
