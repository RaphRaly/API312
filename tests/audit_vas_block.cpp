#include "circuit.h"
#include "bjt_ebers_moll.h"
#include "spice_models_2520.h"
#include "resistor.h"
#include "current_source.h"
#include "voltage_source.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

using namespace std;

// DC Connectivity Audit - reusable version
class DcConnectivityAudit {
public:
    static bool analyze(Circuit& c, bool verbose = true) {
        c.finalize();
        int numNodes = c.getNumNodes();
        const auto& nodeNames = c.getNodeNames();
        
        vector<set<int>> dcAdj(numNodes);
        set<int> referenceNodes;
        referenceNodes.insert(GND);
        
        for (auto& elem : c.elements) {
            vector<pair<int,int>> connections;
            elem->getDcConnections(connections);
            
            for (auto& [a, b] : connections) {
                if (a >= 0 && a < numNodes && b >= 0 && b < numNodes && a != b) {
                    dcAdj[a].insert(b);
                    dcAdj[b].insert(a);
                }
            }
            
            string name = c.getElementName(elem.get());
            if (name.find("Vcc") != string::npos || name.find("Vee") != string::npos ||
                name.find("Vin") != string::npos || name.find("V_") != string::npos) {
                for (auto& [a, b] : connections) {
                    if (a >= 0) referenceNodes.insert(a);
                    if (b >= 0) referenceNodes.insert(b);
                }
            }
        }
        
        vector<bool> reachable(numNodes, false);
        queue<int> bfsQueue;
        
        for (int ref : referenceNodes) {
            if (ref >= 0 && ref < numNodes) {
                reachable[ref] = true;
                bfsQueue.push(ref);
            }
        }
        
        while (!bfsQueue.empty()) {
            int node = bfsQueue.front();
            bfsQueue.pop();
            for (int neighbor : dcAdj[node]) {
                if (!reachable[neighbor]) {
                    reachable[neighbor] = true;
                    bfsQueue.push(neighbor);
                }
            }
        }
        
        vector<int> floatingNodes;
        for (int i = 0; i < numNodes; ++i) {
            if (!reachable[i]) floatingNodes.push_back(i);
        }
        
        if (verbose) {
            if (floatingNodes.empty()) {
                cout << "  DC Audit: All nodes connected - NO FLOATING" << endl;
            } else {
                cout << "  DC Audit: FLOATING NODES:" << endl;
                for (int n : floatingNodes) {
                    auto it = nodeNames.find(n);
                    string name = (it != nodeNames.end()) ? it->second : "Node" + to_string(n);
                    cout << "    - " << name << endl;
                }
            }
        }
        
        return floatingNodes.empty();
    }
};

// Build the isolated VAS test (from test_blocks.cpp test_vas_bias)
void build_vas_block(Circuit& c) {
    cout << "Building isolated VAS block (from test_blocks.cpp)..." << endl;
    
    c.createNode("GND");
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nVEE = c.createNode("VEE");
    
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vee", GND, nVEE, 15.0);
    
    NodeIndex nC4 = c.createNode("C4");
    NodeIndex nE4 = c.createNode("E4");
    NodeIndex nB4 = c.createNode("B4");
    NodeIndex nVAS_Out = c.createNode("VAS_Out");
    NodeIndex nE5 = c.createNode("E5");
    
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    auto q_vas = spiceToBjtParams(getSpiceBjtModel("TIS98"));
    
    // Q4 PNP
    addBjtExtended(c, nC4, nB4, nE4, q_pnp, true, "Q4");
    c.addElement<Resistor>("R_E4", nVCC, nE4, 7000.0);
    c.addElement<Resistor>("R_C4", nC4, nVEE, 10000.0);
    
    // Q5 NPN
    addBjtExtended(c, nVAS_Out, nC4, nE5, q_vas, false, "Q5");
    c.addElement<Resistor>("R_E5", nE5, nVEE, 1200.0);
    c.addElement<CurrentSource>("I_VAS", nVCC, nVAS_Out, 1.5e-3);
    
    // Miller (capacitor open for DC, but DC path via R_Miller_DC)
    c.addElement<Resistor>("R_Miller_DC", nVAS_Out, nC4, 1.0e7);
    
    // Drive Base of Q4
    c.addElement<VoltageSource>("V_Drive", nB4, GND, 14.0);
}

int main() {
    cout << "=== DC CONNECTIVITY AUDIT: Isolated VAS Block ===" << endl;
    
    Circuit c;
    build_vas_block(c);
    
    bool allConnected = DcConnectivityAudit::analyze(c, true);
    
    // Now solve and show voltages
    vector<double> x;
    bool conv = c.solveDc(x);
    
    cout << "\n--- DC SOLUTION ---" << endl;
    if (!conv) {
        cout << "FAILED to converge!" << endl;
    } else {
        const auto& nodeNames = c.getNodeNames();
        for (const auto& [idx, name] : nodeNames) {
            cout << "  V(" << name << ") = " << x[idx] << " V";
            if (abs(x[idx]) > 15.0) {
                cout << "  *** OUTSIDE RAILS! ***";
            }
            cout << endl;
        }
    }
    
    return (allConnected && conv) ? 0 : 1;
}
