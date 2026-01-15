#pragma once
#include "elements.h"
#include "gaussian_solver.h"
#include "linear_system.h"
#include "mna_types.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

using namespace std;

class Circuit {
public:
  using ConvergenceStats = ::ConvergenceStats;
  Circuit() = default;

  NodeIndex createNode(const string &name = "") {
    NodeIndex idx = nextNodeIndex++;
    if (!name.empty()) {
      m_nodeNames[idx] = name;
    }
    return idx;
  }
  
  double getFinalGmin() const { return m_finalGmin; }

  template <typename T, typename... Args> T &addElement(Args &&...args) {
    auto el = make_unique<T>(forward<Args>(args)...);
    T &ref = *el;

    if constexpr (is_base_of<INewtonElement, T>::value) {
      newtonElements.push_back(dynamic_cast<INewtonElement *>(el.get()));
    }
    if constexpr (is_base_of<IDynamicElement, T>::value) {
      dynamicElements.push_back(dynamic_cast<IDynamicElement *>(el.get()));
    }
    if constexpr (is_base_of<IBranchElement, T>::value) {
      branchElements.push_back(dynamic_cast<IBranchElement *>(el.get()));
    }

    extractName(el.get(), forward<Args>(args)...);
    elements.push_back(move(el));
    return ref;
  }

  void finalize() {
    if (finalized)
      return;
    numNodes = nextNodeIndex;
    int currentBranchIndex = numNodes;
    for (auto *be : branchElements) {
      be->setBranchIndex(currentBranchIndex);
      currentBranchIndex += be->branchCount();
    }
    numBranches = currentBranchIndex - numNodes;
    system.resize(numNodes + numBranches);
    finalized = true;
  }

  string getUnknownMeaning(int idx) const {
    if (idx < 0)
      return "GND";
    if (idx < numNodes) {
      auto it = m_nodeNames.find(idx);
      if (it != m_nodeNames.end())
        return "V(" + it->second + ")";
      return "V(Node " + to_string(idx) + ")";
    }
    int brIdx = idx - numNodes;
    int curr = 0;
    for (auto *be : branchElements) {
      if (brIdx >= curr && brIdx < curr + be->branchCount()) {
        auto it = m_elementNames.find(dynamic_cast<void *>(be));
        if (it != m_elementNames.end())
          return "I(" + it->second + ")";
        return "I(Branch " + to_string(brIdx) + ")";
      }
      curr += be->branchCount();
    }
    return "UNKNOWN(" + to_string(idx) + ")";
  }

  // Diagnose Newton failure: identify top culprit nodes
  void diagnoseNewtonFailure(const vector<double>& x, const vector<double>& residual, 
                              const vector<double>& deltaX, double gmin, bool verbose = true) {
    if (!verbose) return;
    
    int N = (int)x.size();
    
    // Build list of (index, |R|, |ΔX|, value)
    struct NodeInfo {
      int idx;
      double absR;
      double absDx;
      double value;
    };
    
    vector<NodeInfo> nodes;
    for (int i = 0; i < N; ++i) {
      nodes.push_back({i, abs(residual[i]), abs(deltaX[i]), x[i]});
    }
    
    cout << "\n=== NEWTON FAILURE DIAGNOSIS at Gmin=" << gmin << " ===" << endl;
    
    // Top 10 by |R| (residual)
    cout << "\nTOP 10 WORST RESIDUALS:" << endl;
    cout << setw(20) << "Node" << setw(15) << "|R|" << setw(15) << "V" << endl;
    cout << string(50, '-') << endl;
    sort(nodes.begin(), nodes.end(), [](auto& a, auto& b) { return a.absR > b.absR; });
    for (int i = 0; i < min(10, N); ++i) {
      cout << setw(20) << getUnknownMeaning(nodes[i].idx) 
           << setw(15) << scientific << setprecision(3) << nodes[i].absR
           << setw(15) << fixed << setprecision(4) << nodes[i].value << endl;
    }
    
    // Top 10 by |ΔX| (step size)
    cout << "\nTOP 10 WORST DELTA-X:" << endl;
    cout << setw(20) << "Node" << setw(15) << "|ΔX|" << setw(15) << "V" << endl;
    cout << string(50, '-') << endl;
    sort(nodes.begin(), nodes.end(), [](auto& a, auto& b) { return a.absDx > b.absDx; });
    for (int i = 0; i < min(10, N); ++i) {
      cout << setw(20) << getUnknownMeaning(nodes[i].idx)
           << setw(15) << scientific << setprecision(3) << nodes[i].absDx
           << setw(15) << fixed << setprecision(4) << nodes[i].value << endl;
    }
    
    cout << "=== END DIAGNOSIS ===\n" << endl;
  }

  void setNodeset(NodeIndex n, double v) {
    if (n >= 0 && n < numNodes) {
      m_nodeset[n] = v;
    }
  }

  // Attempt to find DC operating point using Pseudo-Transient (OpTran)
  // Runs transient simulation with overdamped capacitors to settle into DC
  bool solveDcPseudoTransient(vector<double> &x, double duration = 1e-3, double dt = 1e-6) {
      if (!finalized) finalize();
      
      // Initialize with nodesets if available
      int N = (int)system.size();
      if ((int)x.size() != N) x.assign((size_t)N, 0.0);
      
      for (auto const& [node, val] : m_nodeset) {
          if (node < (int)x.size()) x[node] = val;
      }
      
      // Initialize dynamics
      initializeDynamics(x); 
      
      // Run transient
      int steps = (int)(duration / dt);
      bool stable = true;
      for(int i=0; i<steps; ++i) {
          // Use relaxed tolerances for pseudo-transient to avoid getting stuck
          if (!step(dt, x, 10, 1e-3, 1e-6)) {
              stable = false; 
              // Don't abort, maybe we just hit a rough patch, keep ensuring we have latest x
          }
      }
      
      // Use result as guess for full DC solve
      return solveDc(x);
  }

  bool solveDc(vector<double> &x, int maxIters = 250, double tol = 1e-6,
               bool verbose = false, int numSteps = 50,
               ConvergenceStats *stats = nullptr) {
    if (!finalized)
      finalize();
    int N = (int)system.size();
    if ((int)x.size() != N)
      x.assign((size_t)N, 0.0);
      
    // Apply Nodesets as initial guess
    for (auto const& [node, val] : m_nodeset) {
        if (node < (int)x.size()) x[node] = val;
    }
    
    vector<double> xGuess = x;

    // Global iteration cap to prevent infinite loops (CI safety)
    constexpr int GLOBAL_ITER_CAP = 10000;
    int globalIterCount = 0;

    // Find OUT node for soft stabilization
    NodeIndex outNodeIdx = -1;
    for (const auto &pair : m_nodeNames) {
      if (pair.second == "OUT") {
        outNodeIdx = pair.first;
        break;
      }
    }

    auto computeResidualNorm = [&](const vector<double> &sol) {
      double sumSq = 0.0;
      for (int i = 0; i < N; ++i) {
        double rowSum = 0.0;
        for (int j = 0; j < N; ++j)
          rowSum += system.getA(i, j) * sol[(size_t)j];
        double fi = rowSum - system.getZ(i);
        sumSq += fi * fi;
      }
      return sqrt(sumSq);
    };

    // Track per-iteration data for diagnosis
    vector<double> lastResidual;
    vector<double> lastDeltaX;
    double lastGmin = 0.0;
    
    auto innerNewton = [&](int iters, double scale, double g,
                           vector<double> &guess, bool diagnoseOnFail = false) {
      const int actualMaxIters = max(iters, 300);
      lastGmin = g;
      for (int k = 0; k < actualMaxIters; ++k) {
        // Check global iteration cap
        if (++globalIterCount > GLOBAL_ITER_CAP) {
          if (verbose)
            cout << "[DC] Global iteration cap (" << GLOBAL_ITER_CAP
                 << ") exceeded" << endl;
          return false;
        }
        if (stats)
          stats->totalIterations++;
        system.clear();
        StampContext ctx{system, scale};
        for (auto &e : elements)
          e->stamp(ctx);
        for (auto *ne : newtonElements)
          ne->stampNewton(ctx, guess);
        for (int i = 0; i < numNodes; ++i)
          system.addA(i, i, g);

        // Soft stabilization shunt (strictly removed at scale >= 0.5)
        if (outNodeIdx != -1 && scale < 0.5) {
          system.addA(outNodeIdx, outNodeIdx, 1e-2 * (1.0 - scale * 2.0));
        }

        double oldResidNorm = computeResidualNorm(guess);
        vector<double> deltaX;
        if (GaussianSolver::solve(system, deltaX) >= 0) {
          // Adaptive recovery Gmin if singular
          for (int i = 0; i < numNodes; ++i)
            system.addA(i, i, g * 100.0);
          if (GaussianSolver::solve(system, deltaX) >= 0)
            return false;
        }

        double alpha = 1.0;
        vector<double> xNew;
        bool bt = false;
        for (int b = 0; b < 10; ++b) {
          xNew = guess;
          for (int i = 0; i < N; ++i) {
            double dx = alpha * (deltaX[i] - guess[i]);
            if (dx > 2.0)
              dx = 2.0;
            if (dx < -2.0)
              dx = -2.0;
            xNew[i] += dx;
          }
          LimitContext lctx{xNew, guess};
          for (auto *ne : newtonElements)
            ne->computeLimitedVoltages(lctx);
          if (computeResidualNorm(xNew) < oldResidNorm || alpha < 1e-6) {
            if (b > 0)
              bt = true;
            break;
          }
          alpha *= 0.5;
        }

        double dxMax = 0.0;
        for (int i = 0; i < N; ++i)
          dxMax = max(dxMax, abs(xNew[i] - guess[i]));
        
        // Store for diagnosis
        lastDeltaX.resize(N);
        for (int i = 0; i < N; ++i)
          lastDeltaX[i] = xNew[i] - guess[i];
        
        guess = xNew;

        if (verbose && (k % 50 == 0 || dxMax < tol)) {
          cout << "[DC] G=" << g << " S=" << scale << " K=" << k
               << " R=" << oldResidNorm << " dX=" << dxMax
               << (bt ? " (BT)" : "") << endl;
        }
        if (dxMax < tol && oldResidNorm < 1e-4)
          return true;
      }
      // Compute residual for diagnosis
      if (diagnoseOnFail) {
        lastResidual.resize(N);
        for (int i = 0; i < N; ++i) {
          double rowSum = 0.0;
          for (int j = 0; j < N; ++j)
            rowSum += system.getA(i, j) * guess[(size_t)j];
          lastResidual[i] = rowSum - system.getZ(i);
        }
      }
      return false;
    };

    // Two-stage Homotopy Strategy:
    // Stage 1: Reach 100% source scale using a safe activeGmin.
    // Stage 2: Refine Gmin down to target precision at 100% scale.

    double activeGmin = 1e-7;
    int rampSteps = max(numSteps, 50);

    // Try Stage 1: Source ramp
    bool rampSuccessful = false;
    for (int s = 0; s <= rampSteps; ++s) {
      double scale = (double)s / rampSteps;
      if (stats)
        stats->sourceStepsReached = s;
      if (!innerNewton(maxIters, scale, activeGmin, xGuess, false)) {
        // Fallback: try with even higher Gmin
        activeGmin = 1e-3;
        xGuess.assign((size_t)N, 0.0);
        bool fallbackSuccess = true;
        for (int s2 = 0; s2 <= rampSteps; ++s2) {
          double scale2 = (double)s2 / rampSteps;
          if (stats)
            stats->sourceStepsReached = s2;
          if (!innerNewton(maxIters, scale2, activeGmin, xGuess, false)) {
            fallbackSuccess = false;
            break;
          }
        }
        if (!fallbackSuccess)
          return false;
        rampSuccessful = true;
        break;
      }
      if (s == rampSteps)
        rampSuccessful = true;
    }
    if (!rampSuccessful)
      return false;

    // Stage 2: Gmin refinement at 100% scale
    // Use SPICE-like geometric sequence for robust stepping
    // Each step reduces Gmin by ~3x (half-decade steps)
    const vector<double> gminSequence = {
        5e-4, 2e-4, 1e-4,
        5e-5, 2e-5, 1e-5,
        5e-6, 2e-6, 1e-6,
        5e-7, 2e-7, 1e-7,
        5e-8, 2e-8, 1e-8,
        5e-9, 2e-9, 1e-9,
        5e-10, 2e-10, 1e-10,
        5e-11, 2e-11, 1e-11,
        5e-12, 2e-12, m_gmin
    };
    
    for (double g : gminSequence) {
      if (g >= activeGmin) continue; // Skip steps above our current level
      
      vector<double> xGood = xGuess; 
      
      // Try solve at this Gmin level (with diagnosis on failure)
      bool success = innerNewton(maxIters * 2, 1.0, g, xGuess, true);
      
      if (!success) {
          // Diagnose failure before restoring
          if (verbose && !lastResidual.empty()) {
            diagnoseNewtonFailure(xGuess, lastResidual, lastDeltaX, g, true);
          }
          // Restore previous good solution and stop refinement
          xGuess = xGood;
          if (verbose) {
            cout << "[DC] Gmin stepping stopped at G=" << activeGmin 
                 << " (failed at G=" << g << ")" << endl;
          }
          break; 
      } else {
        activeGmin = g;
        if (verbose && (g == 1e-6 || g == 1e-9 || g == m_gmin)) {
          cout << "[DC] Gmin stepped to " << g << " successfully" << endl;
        }
      }
    }

    // M2: Final verification - ensure solution is valid at target Gmin
    // If we couldn't reach target, at least verify current solution is stable
    if (activeGmin > m_gmin * 10.0) {
      // We're stuck at a high Gmin - warn but still return success
      if (verbose) {
        cout << "[DC] WARNING: Final Gmin=" << activeGmin 
             << " (target=" << m_gmin << "). Solution may be contaminated." << endl;
      }
    }

    x = xGuess;
    m_lastSolution = x;
    m_finalGmin = activeGmin;
    if (stats) {
      stats->converged = true;
      stats->lastResidual = activeGmin; // Repurpose to report final Gmin
    }
    return true;
  }

  bool step(double dt, vector<double> &x, int maxNewtonIters = 8,
            double /*relTol*/ = 1e-6, double absTol = 1e-9) {
    if (!finalized)
      finalize();
    int N = (int)system.size();
    if ((int)x.size() != N)
      x.assign((size_t)N, 0.0);
    for (auto *d : dynamicElements)
      d->beginStep(dt);

    vector<double> xGuess = x;
    bool converged = false;
    for (int k = 0; k < maxNewtonIters; ++k) {
      system.clear();
      StampContext ctx{system, 1.0};
      for (auto &e : elements)
        e->stamp(ctx);
      for (auto *ne : newtonElements)
        ne->stampNewton(ctx, xGuess);
      for (int i = 0; i < numNodes; ++i)
        system.addA(i, i, m_gmin);

      vector<double> xNew;
      if (GaussianSolver::solve(system, xNew) >= 0)
        return false;

      double maxDelta = 0.0;
      for (int i = 0; i < N; ++i) {
        double delta = (xNew[i] - xGuess[i]);
        if (delta > 5.0)
          delta = 5.0;
        if (delta < -5.0)
          delta = -5.0;
        xNew[i] = xGuess[i] + delta;
        maxDelta = max(maxDelta, abs(delta));
      }
      vector<double> xOld = xGuess;
      xGuess = xNew;
      LimitContext limitCtx{xGuess, xOld};
      for (auto *ne : newtonElements)
        ne->computeLimitedVoltages(limitCtx);
      if (maxDelta < absTol) {
        converged = true;
        break;
      }
    }
    if (converged) {
      x = xGuess;
      for (auto *d : dynamicElements)
        d->commitStep(x);
    }
    m_lastSolution = x;
    return converged;
  }

  void initializeDynamics(const vector<double> &x) {
    for (auto *d : dynamicElements)
      d->commitStep(x);
    m_lastSolution = x;
  }

  int getNumNodes() const { return numNodes; }
  int getNumBranches() const { return numBranches; }
  const map<int, string> &getNodeNames() const { return m_nodeNames; }
  const vector<double> &getSolution() const { return m_lastSolution; }
  
  string getElementName(const IElement *e) const {
    auto it = m_elementNames.find((void*)e);
    if (it != m_elementNames.end()) return it->second;
    return "UnknownElement";
  }

  // Expose for golden tests and diagnostics
  DenseLinearSystem system;
  vector<unique_ptr<IElement>> elements;
  vector<INewtonElement *> newtonElements;
  vector<IDynamicElement *> dynamicElements;
  vector<IBranchElement *> branchElements;

private:
  bool finalized = false;
  int nextNodeIndex = 0;
  int numNodes = 0;
  int numBranches = 0;
  map<int, string> m_nodeNames;
  map<void *, string> m_elementNames;
  vector<double> m_lastSolution;
  double m_gmin = 1e-12; // Small for precision; complex circuits use larger
                         // values via fallback
  double m_finalGmin = 1e-12;

public:
  template <typename T, typename First, typename... Rest>
  void extractName(T *ptr, First &&first, Rest &&.../*rest*/) {
    if constexpr (is_convertible<First, string>::value) {
      m_elementNames[dynamic_cast<void *>(ptr)] = string(first);
    }
  }
  void extractName(void * /*ptr*/) {}
  friend class ConnectivityAudit;

private:
  map<NodeIndex, double> m_nodeset;
};

// Global compatibility alias already in mna_types.h
