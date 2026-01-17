#include "api_2520_builder.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// DIAGNOSIS: Why is V(INM) ~ 6V when INM_Ref is 0V and OUT is -8.6V?
// This implies a massive current injection or connectivity/checking error.

void printKclAtNode(Circuit& c, const vector<double>& x, NodeIndex nodeIdx, const string& name) {
    auto& sys = c.system;
    int N = (int)x.size();
    
    cout << "\n=== KCL AUDIT AT " << name << " (Index " << nodeIdx << ") ===" << endl;
    cout << "V(" << name << ") = " << x[nodeIdx] << " V" << endl;
    
    // Iterate over all elements connected to this node
    // Since we don't have easy back-pointers, we reconstruct from matrix row
    // Row i represents KCL for node i: sum(Gij * Vj) + sum(Aik * Ik) = I_source
    
    // Matrix row audit
    double rowSum = 0;
    for (int j = 0; j < N; ++j) {
        double val = sys.getA(nodeIdx, j);
        if (abs(val) > 1e-12) {
             rowSum += val * x[j];
             string namej = c.getUnknownMeaning(j);
             cout << "  + (" << setw(10) << val << ") * V(" << namej << ") [=" << x[j] << "] -> " << val*x[j] << endl;
        }
    }
    double rhs = sys.getZ(nodeIdx);
    cout << "  - RHS (" << rhs << ")" << endl;
    cout << "  = Residual: " << rowSum - rhs << endl;
    
    // Element-specific audit (manual reconstruction)
    // R_Input: INM (node) to INM_Ref
    // R_Feedback: INM (node) to OUT
}

int main() {
    cout << "=== DIAGNOSE INM 6V ANOMALY ===" << endl;
    
    Circuit c;
    Api2520Builder::build2520(c);
    
    // Get named nodes
    const auto& names = c.getNodeNames();
    NodeIndex nINM = -1, nINM_Ref = -1, nOUT = -1;
    
    // Find precise indices
    for(const auto& [idx, name] : names) {
        if (name == "INM") nINM = idx;
        if (name == "INM_Ref") nINM_Ref = idx;
        if (name == "OUT") nOUT = idx;
    }
    
    // Add NodeHook to capture internal nodes if possible, or just search unknowns
    // Q2 base is connected to INM.
    
    vector<double> x;
    // We want to stop at the failure point to analyze
    // But since we can't hook into the middle easily without modifying circuit,
    // let's run solveDc. If it fails (which it does), we analyze the last state.
    // However, Circuit clears system between steps.
    // Instead, let's setup the condition that produces INM=6V if possible, 
    // OR just rely on the fact that solveDc returns the last "best" (or failed) guess.
    
    ConvergenceStats stats;
    c.solveDc(x, 200, 1e-9, true, 50, &stats); 
    
    // Even if it fails/succeeds with warnings, let's look at x
    
    cout << "\n--- STATE ANALYSIS ---" << endl;
    if (nINM != -1) cout << "V(INM) = " << x[nINM] << " V" << endl;
    if (nINM_Ref != -1) cout << "V(INM_Ref) = " << x[nINM_Ref] << " V" << endl;
    if (nOUT != -1) cout << "V(OUT) = " << x[nOUT] << " V" << endl;
    
    if (nINM != -1 && nINM_Ref != -1 && nOUT != -1) {
        double v_inm = x[nINM];
        double v_ref = x[nINM_Ref];
        double v_out = x[nOUT];
        
        // R_Input check
        double i_input = (v_inm - v_ref) / 10000.0;
        cout << "I(R_Input) [INM->REF] = " << i_input * 1000.0 << " mA" << endl;
        
        // R_Feedback check
        double i_feedback = (v_inm - v_out) / 10000.0;
        cout << "I(R_Feedback) [INM->OUT] = " << i_feedback * 1000.0 << " mA" << endl;
        
        double current_leaving = i_input + i_feedback;
        cout << "Total Resistive Current Leaving INM: " << current_leaving * 1000.0 << " mA" << endl;
        cout << "MUST be injected by Q2 Base or error." << endl;
    }
    
    if (nINM != -1) {
        printKclAtNode(c, x, nINM, "INM");
    }
    
    return 0;
}
