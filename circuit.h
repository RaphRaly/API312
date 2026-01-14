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

struct ConvergenceStats {
  int totalIterations = 0;
  int sourceStepsReached = 0;
  double lastResidual = 0.0;
  bool converged = false;
};

class Circuit {
public:
  Circuit() = default;

  NodeIndex createNode(const std::string &name = "") {
    NodeIndex idx = nextNodeIndex++;
    if (!name.empty()) {
      m_nodeNames[idx] = name;
    }
    return idx;
  }

  template <typename T, typename... Args> T &addElement(Args &&...args) {
    auto el = std::make_unique<T>(std::forward<Args>(args)...);
    T &ref = *el;

    if constexpr (std::is_base_of<INewtonElement, T>::value) {
      newtonElements.push_back(dynamic_cast<INewtonElement *>(el.get()));
    }
    if constexpr (std::is_base_of<IDynamicElement, T>::value) {
      dynamicElements.push_back(dynamic_cast<IDynamicElement *>(el.get()));
    }
    if constexpr (std::is_base_of<IBranchElement, T>::value) {
      branchElements.push_back(dynamic_cast<IBranchElement *>(el.get()));
    }

    extractName(el.get(), std::forward<Args>(args)...);
    elements.push_back(std::move(el));
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

  std::string getUnknownMeaning(int idx) const {
    if (idx < 0)
      return "GND";
    if (idx < numNodes) {
      auto it = m_nodeNames.find(idx);
      if (it != m_nodeNames.end())
        return "V(" + it->second + ")";
      return "V(Node " + std::to_string(idx) + ")";
    }
    int brIdx = idx - numNodes;
    int curr = 0;
    for (auto *be : branchElements) {
      if (brIdx >= curr && brIdx < curr + be->branchCount()) {
        auto it = m_elementNames.find(dynamic_cast<void *>(be));
        if (it != m_elementNames.end())
          return "I(" + it->second + ")";
        return "I(Branch " + std::to_string(brIdx) + ")";
      }
      curr += be->branchCount();
    }
    return "UNKNOWN(" + std::to_string(idx) + ")";
  }

  bool solveDc(std::vector<double> &x, int maxIters = 250, double tol = 1e-6,
               bool verbose = false, int numSteps = 50,
               ConvergenceStats *stats = nullptr) {
    if (!finalized)
      finalize();
    int N = (int)system.size();
    if ((int)x.size() != (size_t)N)
      x.assign((std::size_t)N, 0.0);
    std::vector<double> xGuess = x;

    // Condition #3: Locate OUT node for temporary soft stabilization
    NodeIndex outNodeIdx = -1;
    for (const auto &pair : m_nodeNames) {
      if (pair.second == "OUT") {
        outNodeIdx = pair.first;
        break;
      }
    }

    // Condition #2&3: Methodical Gmin Levels & Soft Stabilization
    const std::vector<double> gminSequence = {1e-5, 1e-6,  1e-7,  1e-8,
                                              1e-9, 1e-10, m_gmin};
    double activeGmin = 1e-5; // Start with safe Gmin for source loop

    auto computeResidualNorm = [&](const std::vector<double> &sol) {
      double sumSq = 0.0;
      for (int i = 0; i < N; ++i) {
        double rowSum = 0.0;
        for (int j = 0; j < N; ++j)
          rowSum += system.getA(i, j) * sol[(std::size_t)j];
        double fi = rowSum - system.getZ(i);
        sumSq += fi * fi;
      }
      return std::sqrt(sumSq);
    };

    auto innerNewton = [&](int iters, double scale, double g,
                           std::vector<double> &guess) {
      for (int k = 0; k < iters; ++k) {
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
        if (outNodeIdx != -1 && scale < 0.5) {
          system.addA(outNodeIdx, outNodeIdx, 1e-2 * (1.0 - scale * 2.0));
        }

        double oldResidNorm = computeResidualNorm(guess);
        std::vector<double> deltaX;
        if (GaussianSolver::solve(system, deltaX) >= 0) {
          for (int i = 0; i < numNodes; ++i)
            system.addA(i, i, g * 100.0);
          if (GaussianSolver::solve(system, deltaX) >= 0)
            return false;
        }

        double alpha = 1.0;
        std::vector<double> xNew;
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
          dxMax = std::max(dxMax, std::abs(xNew[i] - guess[i]));
        guess = xNew;

        if (verbose && (k % 50 == 0 || dxMax < tol)) {
          std::cout << "[DC] G=" << g << " S=" << scale << " K=" << k
                    << " R=" << oldResidNorm << " dX=" << dxMax
                    << (bt ? " (BT)" : "") << std::endl;
        }
        if (dxMax < tol && oldResidNorm < 1e-4)
          return true;
      }
      return false;
    };

    bool stage1Success = false;
    double startScale = 0.02;
    for (int s = 0; s <= numSteps; ++s) {
      double scale = startScale + (1.0 - startScale) * (double)s / numSteps;
      if (scale > 1.0)
        scale = 1.0;
      if (stats)
        stats->sourceStepsReached = s;
      if (!innerNewton(maxIters, scale, activeGmin, xGuess))
        break;
      if (scale >= 1.0) {
        stage1Success = true;
        break;
      }
    }

    if (!stage1Success) {
      activeGmin = 5e-5; // Try recovery with high Gmin
      xGuess.assign((size_t)N, 0.0);
      for (int s = 0; s <= numSteps; ++s) {
        double scale = startScale + (1.0 - startScale) * (double)s / numSteps;
        if (scale >= 1.0)
          scale = 1.0;
        if (!innerNewton(maxIters, scale, activeGmin, xGuess))
          return false;
        if (scale >= 1.0) {
          stage1Success = true;
          break;
        }
      }
    }

    // Stage 2: Gmin refinement at 100% scale
    for (double g : gminSequence) {
      if (g >= activeGmin)
        continue;
      // Use double iterations for critical refinement steps
      if (!innerNewton(maxIters * 2, 1.0, g, xGuess))
        return false;
    }

    x = xGuess;
    m_lastSolution = x;
    if (stats)
      stats->converged = true;
    return true;
  }

  bool step(double dt, std::vector<double> &x, int maxNewtonIters = 8,
            double relTol = 1e-6, double absTol = 1e-9) {
    if (!finalized)
      finalize();
    int N = (int)system.size();
    if ((int)x.size() != N)
      x.assign((std::size_t)N, 0.0);
    for (auto *d : dynamicElements)
      d->beginStep(dt);

    std::vector<double> xGuess = x;
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

      std::vector<double> xNew;
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
        maxDelta = std::max(maxDelta, std::abs(delta));
      }
      std::vector<double> xOld = xGuess;
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

  void initializeDynamics(const std::vector<double> &x) {
    for (auto *d : dynamicElements)
      d->commitStep(x);
    m_lastSolution = x;
  }

  int getNumNodes() const { return numNodes; }
  int getNumBranches() const { return numBranches; }
  const std::map<int, std::string> &getNodeNames() const { return m_nodeNames; }
  const std::vector<double> &getSolution() const { return m_lastSolution; }

  // Expose for golden tests and diagnostics
  DenseLinearSystem system;
  std::vector<std::unique_ptr<IElement>> elements;
  std::vector<INewtonElement *> newtonElements;
  std::vector<IDynamicElement *> dynamicElements;
  std::vector<IBranchElement *> branchElements;

private:
  bool finalized = false;
  int nextNodeIndex = 0;
  int numNodes = 0;
  int numBranches = 0;
  std::map<int, std::string> m_nodeNames;
  std::map<void *, std::string> m_elementNames;
  std::vector<double> m_lastSolution;
  double m_gmin = 1e-12;

  template <typename T, typename First, typename... Rest>
  void extractName(T *ptr, First &&first, Rest &&...rest) {
    if constexpr (std::is_convertible<First, std::string>::value) {
      m_elementNames[dynamic_cast<void *>(ptr)] = std::string(first);
    }
  }
  void extractName(void *ptr) {}
  friend class ConnectivityAudit;
};
