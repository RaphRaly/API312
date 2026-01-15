#pragma once
#include "circuit.h"
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

using namespace std;

// Topological Audit tool to detect floating nodes
class ConnectivityAudit {
public:
  static bool run(const Circuit &c) {
    if (!c.finalized) {
      cerr << "[Audit] ERROR: ConnectivityAudit requested before "
                   "Circuit::finalize()."
                << endl;
      return false;
    }

    cout << "[Audit] Running Structural Integrity Check..." << endl;

    int numNodes = c.getNumNodes();
    // Adjacency list for DC conduction paths
    vector<vector<int>> adj(numNodes);

    for (const auto &e : c.elements) {
      vector<pair<int, int>> connections;
      e->getDcConnections(connections);
      for (auto &conn : connections) {
        int u = conn.first;
        int v = conn.second;

        // Treat GND as -1.
        if (u == GND && v == GND)
          continue;

        if (u != GND && v != GND) {
          adj[u].push_back(v);
          adj[v].push_back(u);
        } else if (u != GND) {
          adj[u].push_back(GND); // u connected to ground
        } else if (v != GND) {
          adj[v].push_back(GND); // v connected to ground
        }
      }
    }

    // BFS starting from all nodes directly connected to ground
    vector<bool> reached(numNodes, false);
    queue<int> q;

    for (int i = 0; i < numNodes; ++i) {
      for (int neighbor : adj[i]) {
        if (neighbor == GND) {
          if (!reached[i]) {
            reached[i] = true;
            q.push(i);
          }
        }
      }
    }

    while (!q.empty()) {
      int u = q.front();
      q.pop();

      for (int v : adj[u]) {
        if (v != GND && !reached[v]) {
          reached[v] = true;
          q.push(v);
        }
      }
    }

    // Results reporting
    int floatingCount = 0;
    for (int i = 0; i < numNodes; ++i) {
      if (!reached[i]) {
        string nodeName = c.m_nodeNames.count(i)
                                   ? c.m_nodeNames.at(i)
                                   : "node_" + to_string(i);
        cout
            << "[AUDIT] CRITICAL: Node " << i << " (" << nodeName
            << ") is FLOATING (no DC path to ground via R, V, or junctions)."
            << endl;
        floatingCount++;
      }
    }

    if (floatingCount == 0) {
      cout << "[Audit] SUCCESS: All nodes have a DC path to ground."
                << endl;
      return true;
    } else {
      cout << "[Audit] FAILURE: " << floatingCount
                << " floating nodes detected!" << endl;
      return false;
    }
  }
};
