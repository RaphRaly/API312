#include "api_2520_builder.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

// PROPER MNA Constraint Audit
// Phase 1: Verify current sources don't count as DC connections (already true)
// Phase 2: Check "effective floating" - is Jacobian row dominated by Gmin?

class MnaConstraintAudit {
public:
    // Build Jacobian at operating point and analyze row sums
    // A node is "effectively floating" if its Jacobian row sum is << 1/Gmin_threshold
    static void analyzeJacobianRows(Circuit& c, const vector<double>& x, 
                                     double gminApplied, bool verbose = true) {
        if (verbose) {
            cout << "\n=== MNA JACOBIAN CONSTRAINT ANALYSIS ===" << endl;
            cout << "Applied Gmin: " << gminApplied << " S" << endl;
        }
        
        c.finalize();
        int numNodes = c.getNumNodes();
        const auto& nodeNames = c.getNodeNames();
        
        // Rebuild Jacobian at this operating point
        c.system.clear();
        StampContext ctx{c.system, 1.0};
        
        // Stamp all linear elements
        for (auto& e : c.elements) {
            e->stamp(ctx);
        }
        
        // Stamp Newton elements at current x
        for (auto* ne : c.newtonElements) {
            ne->stampNewton(ctx, x);
        }
        
        // DON'T add Gmin here - we want to see the "real" Jacobian
        
        // Analyze each node's row
        vector<pair<int, double>> nodeSums;
        for (int i = 0; i < numNodes; ++i) {
            double rowSum = 0.0;
            for (int j = 0; j < (int)c.system.size(); ++j) {
                rowSum += abs(c.system.getA(i, j));
            }
            nodeSums.push_back({i, rowSum});
        }
        
        // Sort by row sum (ascending - worst first)
        sort(nodeSums.begin(), nodeSums.end(), 
             [](auto& a, auto& b) { return a.second < b.second; });
        
        if (verbose) {
            cout << "\n--- NODE JACOBIAN ROW SUMS (ascending) ---" << endl;
            cout << "Nodes with rowSum < 10*Gmin are EFFECTIVELY FLOATING" << endl;
            cout << setw(15) << "Node" << setw(15) << "RowSum (S)" 
                 << setw(15) << "vs Gmin" << setw(12) << "Status" << endl;
            cout << string(57, '-') << endl;
            
            int floatingCount = 0;
            double threshold = gminApplied * 10.0;  // Considered floating if < 10x Gmin
            
            for (auto& [nodeIdx, rowSum] : nodeSums) {
                auto it = nodeNames.find(nodeIdx);
                string name = (it != nodeNames.end()) ? it->second : "Node" + to_string(nodeIdx);
                
                double ratio = rowSum / gminApplied;
                string status;
                if (rowSum < threshold) {
                    status = "FLOATING!";
                    floatingCount++;
                } else if (ratio < 100) {
                    status = "WEAK";
                } else {
                    status = "OK";
                }
                
                cout << setw(15) << name 
                     << setw(15) << scientific << setprecision(2) << rowSum
                     << setw(15) << fixed << setprecision(1) << ratio << "x"
                     << setw(12) << status << endl;
            }
            
            cout << "\nEFFECTIVELY FLOATING NODES: " << floatingCount << endl;
        }
    }
};

// Build isolated VAS block and analyze
void analyze_vas_block() {
    cout << "\n========================================" << endl;
    cout << "  ISOLATED VAS BLOCK - MNA CONSTRAINT ANALYSIS" << endl;
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
    
    // Q4 PNP
    addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    // Q5 NPN
    addBjtExtended(c, nVAS, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS, 1.5e-3);
    
    // Miller DC path (10MΩ)
    c.addElement<Resistor>("R_Miller_DC", nVAS, nC4, 1.0e7);
    
    // Drive Base of Q4
    c.addElement<VoltageSource>("V_Drive", nB4, GND, 14.0);
    
    // Solve
    vector<double> x;
    bool conv = c.solveDc(x, 250, 1e-6, true);  // verbose
    
    double finalGmin = c.getFinalGmin();
    cout << "\n--- SOLUTION ---" << endl;
    cout << "Converged: " << (conv ? "YES" : "NO") << endl;
    cout << "Final Gmin: " << finalGmin << " S" << endl;
    
    const auto& nodeNames = c.getNodeNames();
    for (const auto& [idx, name] : nodeNames) {
        double v = x[idx];
        cout << "  V(" << setw(10) << left << name << ") = " 
             << setw(10) << right << fixed << setprecision(4) << v << " V";
        if (abs(v) > 15.0) cout << " *** OUTSIDE RAILS! ***";
        cout << endl;
    }
    
    // Analyze Jacobian
    MnaConstraintAudit::analyzeJacobianRows(c, x, finalGmin, true);
}

// Analyze full 2520
void analyze_full_2520() {
    cout << "\n========================================" << endl;
    cout << "  FULL 2520 - MNA CONSTRAINT ANALYSIS" << endl;
    cout << "========================================" << endl;
    
    Circuit c;
    Api2520Builder::build2520(c);
    
    vector<double> x;
    bool conv = c.solveDc(x);
    
    double finalGmin = c.getFinalGmin();
    cout << "Converged: " << (conv ? "YES" : "NO") << endl;
    cout << "Final Gmin: " << finalGmin << " S" << endl;
    
    // Analyze Jacobian
    MnaConstraintAudit::analyzeJacobianRows(c, x, finalGmin, true);
}

int main() {
    cout << "=== MNA CONSTRAINT AUDIT ===" << endl;
    cout << "Checking for 'effectively floating' nodes" << endl;
    cout << "(nodes where Jacobian row sum is dominated by Gmin)" << endl;
    
    analyze_vas_block();
    analyze_full_2520();
    
    return 0;
}
