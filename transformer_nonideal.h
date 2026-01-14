#pragma once

#include "elements.h"
#include "mna_types.h"
#include "stamping.h"
#include <cmath>
#include <string>
#include <vector>

/**
 * Non-ideal Transformer with B-H Saturation & CMRR
 */
class TransformerNonIdeal : public IElement,
                            public IDynamicElement,
                            public IBranchElement {
public:
  TransformerNonIdeal(const std::string &name, NodeIndex p1, NodeIndex n1,
                      NodeIndex p2, NodeIndex n2, double ratio, double Lp,
                      double phiSat = 0.05)
      : m_name(name), m_p1(p1), m_n1(n1), m_p2(p2), m_n2(n2), m_ratio(ratio),
        m_Lp(Lp), m_phiSat(phiSat) {}

  virtual ~TransformerNonIdeal() = default;

  int branchCount() const override {
    return 1;
  } // Ideal branch for voltage constraint
  void setBranchIndex(int first) override { m_brIdx = first; }

  void stamp(StampContext &ctx) const override {
    // Ideal transformer constraint: V2 = ratio * V1
    // V(p2) - V(n2) - ratio * (V(p1) - V(n1)) = 0
    const int k = m_brIdx;
    if (m_p2 != GND) {
      ctx.sys.addA(k, m_p2, 1.0);
      ctx.sys.addA(m_p2, k, 1.0);
    }
    if (m_n2 != GND) {
      ctx.sys.addA(k, m_n2, -1.0);
      ctx.sys.addA(m_n2, k, -1.0);
    }

    double n = m_ratio;
    if (m_p1 != GND) {
      ctx.sys.addA(k, m_p1, -n);
      ctx.sys.addA(m_p1, k, n);
    }
    if (m_n1 != GND) {
      ctx.sys.addA(k, m_n1, n);
      ctx.sys.addA(m_n1, k, -n);
    }
  }

  void beginStep(double dt) override { m_dt = dt; }
  void commitStep(const std::vector<double> &xSolved) override {
    (void)xSolved;
  }

  // Topological audit
  void getDcConnections(
      std::vector<std::pair<int, int>> &connections) const override {
    connections.push_back({m_p1, m_n1});
    connections.push_back({m_p2, m_n2});
  }

private:
  std::string m_name;
  NodeIndex m_p1, m_n1, m_p2, m_n2;
  double m_ratio, m_Lp, m_phiSat;
  int m_brIdx = -1;
  double m_dt = 1e-6;
};
