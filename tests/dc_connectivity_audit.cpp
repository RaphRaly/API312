#include "circuit.h"
#include "api_2520_builder.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

using namespace std;

// DC Connectivity Audit Tool
// Determines which nodes have DC paths to reference (GND or voltage source terminals)
// Capacitors are treated as OPEN CIRCUITS for DC
// Current sources are treated as OPEN CIRCUITS (infinite impedance)
// Voltage sources create DC connectivity between their terminals (KVL constraint)

class DcConnectivityAudit {
public:
    // Run audit on a finalized circuit
    static void analyze(Circuit& c, bool verbose = true) {
        if (verbose) {
            cout << "\n================================================" << endl;
            cout << "  DC CONNECTIVITY AUDIT" << endl;
            cout << "================================================" << endl;
        }
        
        c.finalize();
        int numNodes = c.getNumNodes();
        const auto& nodeNames = c.getNodeNames();
        
        // Build adjacency list from DC-conductive elements
        vector<set<int>> dcAdj(numNodes);
        set<int> referenceNodes;  // Nodes with known DC reference
        
        referenceNodes.insert(GND);  // GND is always a reference
        
        if (verbose) {
            cout << "\nScanning elements for DC connections..." << endl;
        }
        
        for (auto& elem : c.elements) {
            vector<pair<int,int>> connections;
            elem->getDcConnections(connections);
            
            for (auto& [a, b] : connections) {
                if (a >= 0 && a < numNodes && b >= 0 && b < numNodes && a != b) {
                    dcAdj[a].insert(b);
                    dcAdj[b].insert(a);
                }
            }
            
            // VoltageSource terminals are references via KVL constraint
            // Check by seeing if this element provides DC connections AND is a branch element
            // (VoltageSource provides connections and is a branch element)
            string name = c.getElementName(elem.get());
            if (name.find("Vcc") != string::npos || name.find("Vee") != string::npos ||
                name.find("Vin") != string::npos || name.find("Vsrc") != string::npos ||
                name.find("V_") != string::npos) {
                for (auto& [a, b] : connections) {
                    if (a >= 0) referenceNodes.insert(a);
                    if (b >= 0) referenceNodes.insert(b);
                }
            }
        }
        
        if (verbose) {
            cout << "Reference nodes (directly connected to voltage sources): ";
            for (int n : referenceNodes) {
                if (n == GND) cout << "GND ";
                else {
                    auto it = nodeNames.find(n);
                    if (it != nodeNames.end()) cout << it->second << " ";
                    else cout << "Node" << n << " ";
                }
            }
            cout << endl;
        }
        
        // BFS to find all nodes reachable from reference nodes
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
        
        // Find floating nodes
        vector<int> floatingNodes;
        for (int i = 0; i < numNodes; ++i) {
            if (!reachable[i]) {
                floatingNodes.push_back(i);
            }
        }
        
        cout << "\n--- AUDIT RESULTS ---" << endl;
        if (floatingNodes.empty()) {
            cout << "All nodes have DC paths to reference. NO FLOATING NODES." << endl;
        } else {
            cout << "FLOATING NODES DETECTED (" << floatingNodes.size() << "):" << endl;
            for (int n : floatingNodes) {
                auto it = nodeNames.find(n);
                string name = (it != nodeNames.end()) ? it->second : "Node" + to_string(n);
                cout << "  - " << name << " (index " << n << ")" << endl;
                
                // Suggest fix
                cout << "    Suggested fix: Add high-value resistor (10M-100M) to GND or rail" << endl;
            }
        }
        
        // Print connectivity matrix summary
        if (verbose) {
            cout << "\n--- NODE CONNECTIVITY DEGREE ---" << endl;
            for (int i = 0; i < numNodes; ++i) {
                auto it = nodeNames.find(i);
                string name = (it != nodeNames.end()) ? it->second : "Node" + to_string(i);
                cout << "  " << setw(15) << left << name 
                     << ": " << dcAdj[i].size() << " DC neighbors"
                     << (reachable[i] ? "" : " [FLOATING!]") << endl;
            }
        }
        
        cout << "================================================\n" << endl;
    }
};

int main() {
    cout << "=== DC CONNECTIVITY AUDIT: 2520 Circuit ===" << endl;
    
    Circuit c;
    Api2520Builder::build2520(c);
    
    DcConnectivityAudit::analyze(c, true);
    
    return 0;
}
