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

  bool solveDc(std::vector<double> &x, int maxIters = 100, double tol = 1e-6,
               bool verbose = false, int numSteps = 10) {
    if (!finalized)
      finalize();
    int N = (int)system.size();
    x.assign((std::size_t)N, 0.0);
    std::vector<double> xGuess = x;

    LimitContext initCtx{xGuess, xGuess};
    for (auto *ne : newtonElements)
      ne->computeLimitedVoltages(initCtx);

    bool useSourceStepping = (numSteps > 1);
    for (int step = 1; step <= numSteps; ++step) {
      double currentScale = (double)step / numSteps;
      if (useSourceStepping && verbose) {
        std::cerr << "[Circuit] Source Step " << step << "/" << numSteps
                  << " (Scale=" << currentScale << ")" << std::endl;
      }

      bool stepConverged = false;
      // Gmin Stepping: start with high conductance to help convergence
      // Final step MUST converge with m_gmin (target).
      std::vector<double> gminLevels;
      if (step < numSteps) {
        gminLevels = {1e-3, 1e-6, 1e-9, m_gmin};
      } else {
        gminLevels = {1e-6, 1e-9, m_gmin}; // Tighter on final step
      }

      for (double activeGmin : gminLevels) {
        for (int k = 0; k < maxIters; ++k) {
          system.clear();
          StampContext ctx{system, currentScale};
          for (auto &e : elements)
            e->stamp(ctx);
          for (auto *ne : newtonElements)
            ne->stampNewton(ctx, xGuess);

          for (int i = 0; i < numNodes; ++i) {
            system.addA(i, i, activeGmin);
          }

          double maxResid = 0.0;
          for (int i = 0; i < N; ++i) {
            double rowSum = 0.0;
            for (int j = 0; j < N; ++j) {
              rowSum += system.getA(i, j) * xGuess[(std::size_t)j];
            }
            double fi = rowSum - system.getZ(i);
            maxResid = std::max(maxResid, std::abs(fi));
          }

          std::vector<double> xNew;
          if (GaussianSolver::solve(system, xNew) >= 0)
            break;

          double maxDelta = 0.0;
          std::vector<double> xOld = xGuess;
          double damping = (k > 50) ? 0.2 : (k > 20 ? 0.5 : 1.0);

          for (int i = 0; i < N; ++i) {
            double delta =
                damping * (xNew[(std::size_t)i] - xOld[(std::size_t)i]);
            // Tighter clamp for stability: 2.0V
            if (delta > 2.0)
              delta = 2.0;
            if (delta < -2.0)
              delta = -2.0;
            xGuess[(std::size_t)i] = xOld[(std::size_t)i] + delta;
            maxDelta = std::max(maxDelta, std::abs(delta));
          }

          if (maxDelta < tol && maxResid < 1e-6) {
            stepConverged = true;
            break;
          }

          LimitContext limitCtx{xGuess, xOld};
          for (auto *ne : newtonElements)
            ne->computeLimitedVoltages(limitCtx);
        }
        if (stepConverged)
          break;
      }

      if (!stepConverged) {
        std::cerr << "[Circuit] Failed to converge source step " << step
                  << std::endl;
        x = xGuess; // Store last guess for debug
        return false;
      }
    }
    x = xGuess;
    m_lastSolution = x;
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
    double damping = 1.0;
    bool converged = false;

    for (int k = 0; k < maxNewtonIters; ++k) {
      system.clear();
      StampContext ctx{system, 1.0};
      for (auto &e : elements)
        e->stamp(ctx);
      for (auto *ne : newtonElements)
        ne->stampNewton(ctx, xGuess);

      double gmin_active = 1e-9;
      for (int i = 0; i < numNodes; ++i) {
        system.addA(i, i, gmin_active);
      }

      std::vector<double> xNew;
      if (GaussianSolver::solve(system, xNew) >= 0)
        return false;

      double maxDelta = 0.0;
      double damping = 1.0;
      for (int i = 0; i < N; ++i) {
        double delta =
            damping * (xNew[(std::size_t)i] - xGuess[(std::size_t)i]);
        if (delta > 5.0)
          delta = 5.0;
        if (delta < -5.0)
          delta = -5.0;
        xNew[(std::size_t)i] = xGuess[(std::size_t)i] + delta;
        maxDelta = std::max(maxDelta, std::abs(delta));
      }

      std::vector<double> xPrevIter = xGuess;
      xGuess = xNew;

      LimitContext limitCtx{xGuess, xPrevIter};
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

private:
  bool finalized = false;
  int nextNodeIndex = 0;
  int numNodes = 0;
  int numBranches = 0;
  DenseLinearSystem system;
  double sourceScale = 1.0;
  std::vector<std::unique_ptr<IElement>> elements;
  std::vector<INewtonElement *> newtonElements;
  std::vector<IDynamicElement *> dynamicElements;
  std::vector<IBranchElement *> branchElements;
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
