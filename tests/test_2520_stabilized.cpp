#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// STABILIZED 2520 TEST
// Uses precise Nodesets and Pseudo-Transient solver to reliably find the physical DC point.

int main() {
    cout << "=== STABILIZED 2520 DC TEST ===" << endl;
    cout << "Strategy: Corrected Nodesets + Pseudo-Transient Continuation" << endl;
    
    Circuit c;
    Api2520Builder::build2520(c);
    
    const auto& names = c.getNodeNames();
    NodeIndex nC4 = -1, nE5 = -1, nC2 = -1, nE4 = -1, nVAS = -1, nOUT = -1;
    NodeIndex nB_HI = -1, nB_LO = -1;

    for(const auto& [idx, name] : names) {
        if (name == "Q4_Col") nC4 = idx;       // Q5 Base
        if (name == "Q5_Emit") nE5 = idx;
        if (name == "Q2_Col") nC2 = idx;       // Q4 Base
        if (name == "Q4_Emit") nE4 = idx;
        if (name == "VAS_Col") nVAS = idx;
        if (name == "OUT") nOUT = idx;

        if (name == "VAS_High") nB_HI = idx;
        if (name == "VAS_Low") nB_LO = idx;
    }
    
    // === CRITICAL FIX: Physically consistent Nodesets ===
    // Q5 NPN: Needs I_C ~ 1.5mA. R_E5=1.2k to -15V.
    // V(E5) = -15 + 1.5e-3*1200 = -13.2 V.
    // V(B5) = V(C4) = -13.2 + 0.65 = -12.55 V.
    
    // Q4 PNP: V(E4) = VCC - I_Q4*R_E4. Assume I_Q4 ~ 0.5mA.
    // V(E4) = 15 - 0.5e-3*7000 = 11.5 V.
    // V(B4) = V(C2) = 11.5 - 0.65 = 10.85 V.
    
    // VAS Center
    // V(OUT) = 0.
    // V(B_HI) ~ 0.6. V(B_LO) ~ -0.6.
    // V(VAS) ~ 0.7.
    
    cout << "\nApplying Corrected Nodesets:" << endl;
    
    if (nC4 != -1) { c.setNodeset(nC4, -12.55); cout << "  V(Q4_Col/Q5_Base) = -12.55 V" << endl; }
    if (nE5 != -1) { c.setNodeset(nE5, -13.20); cout << "  V(Q5_Emit)       = -13.20 V" << endl; }
    
    if (nC2 != -1) { c.setNodeset(nC2, 11.0);   cout << "  V(Q2_Col/Q4_Base) = 11.00 V" << endl; }
    if (nE4 != -1) { c.setNodeset(nE4, 11.65);  cout << "  V(Q4_Emit)       = 11.65 V" << endl; }
    
    if (nVAS != -1) { c.setNodeset(nVAS, 0.7);   cout << "  V(VAS_Col)       =  0.70 V" << endl; }
    if (nOUT != -1) { c.setNodeset(nOUT, 0.0);   cout << "  V(OUT)           =  0.00 V" << endl; }
    
    // Also help the Output Stage
    if (nB_HI != -1) c.setNodeset(nB_HI, 0.6);
    if (nB_LO != -1) c.setNodeset(nB_LO, -0.6);

    // Run Pseudo-Transient
    // 5ms duration, 1us step -> 5000 steps.
    cout << "\nRunning Pseudo-Transient (OpTran)..." << endl;
    
    vector<double> x;
    bool success = c.solveDcPseudoTransient(x, 5.0e-3, 1.0e-6);
    
    double finalGmin = c.getFinalGmin();
    cout << "OpTran Result: " << (success ? "SUCCESS" : "FAIL") << endl;
    cout << "Final Gmin: " << finalGmin << " S" << endl;
    
    if (nVAS != -1) cout << "V(VAS) = " << x[nVAS] << " V" << endl;
    if (nOUT != -1) cout << "V(OUT) = " << x[nOUT] << " V" << endl;
    if (names.count(3)) cout << "V(INM) = " << x[3] << " V" << endl; // Usually INM is early index
    
    if (success && abs(x[nOUT]) < 1.0) {
        cout << "\n*** VICTORY: PHYSICAL DC POINT FOUND ***" << endl;
        return 0;
    } 
    
    if (abs(x[nOUT]) > 13.0) {
        cout << "\n*** FAIL: Still latched to rail ***" << endl;
    } else {
        cout << "\n*** PARTIAL: Output " << x[nOUT] << "V not yet 0V ***" << endl;
    }
    
    return 1;
}
