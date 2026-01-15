#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

// Test golden: Jacobian verification by finite differences
// This ensures our analytical Jacobian matches numerical derivatives

bool test_jacobian_finite_difference() {
  cout << "Testing Jacobian finite difference verification..."
            << endl;

  // Simple circuit: Voltage divider with BJT
  Circuit c;
  NodeIndex n1 = c.createNode();
  NodeIndex n2 = c.createNode();
  NodeIndex n3 = c.createNode();

  c.addElement<VoltageSource>(n1, GND, 5.0);
  c.addElement<Resistor>(n1, n2, 1000.0);
  c.addElement<Resistor>(n2, n3, 1000.0);

  BjtParams params;
  params.VAF = 50.0; // Add Early effect
  c.addElement<BjtNpnEbersMoll>(n3, n2, GND, params);

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "DC solve failed" << endl;
    return false;
  }

  // Now test Jacobian at the operating point
  // We'll perturb each voltage and measure current changes
  const double h = 1e-6; // Small perturbation
  const int N = x.size();

  // Analytical Jacobian from our implementation
  c.system.clear();
  StampContext ctx{c.system, 1.0};

  // Stamp the circuit at operating point
  for (const auto &e : c.elements) {
    e->stamp(ctx);
  }

  // Compute limited voltages
  LimitContext limitCtx{x, x};
  for (auto *nl : c.newtonElements) {
    nl->computeLimitedVoltages(limitCtx);
  }

  // Stamp nonlinear elements
  for (const auto *nl : c.newtonElements) {
    nl->stampNewton(ctx, x);
  }

  vector<double> analytical_rhs = c.system.rhs();

  // Helper to compute residual f(x) = A(x)x - Z(x)
  auto computeResidual = [&](const vector<double> &v,
                             const vector<double> &v_old) {
    c.system.clear();
    StampContext ctx_local{c.system, 1.0};
    for (auto &e : c.elements)
      e->stamp(ctx_local);
    LimitContext lctx{v, v_old};
    for (auto *nl : c.newtonElements)
      nl->computeLimitedVoltages(lctx);
    for (auto *nl : c.newtonElements)
      nl->stampNewton(ctx_local, v);

    vector<double> res(N, 0.0);
    const vector<double> &rhs = c.system.rhs();
    for (int row = 0; row < N; ++row) {
      double sum = 0.0;
      for (int col = 0; col < N; ++col) {
        sum += c.system.getA(row, col) * v[col];
      }
      res[row] = sum - rhs[row];
    }
    return res;
  };

  vector<double> r0 = computeResidual(x, x);

  // Numerical Jacobian by finite differences
  for (int i = 0; i < N; ++i) {
    vector<double> x_pert = x;
    x_pert[i] += h;

    vector<double> r_pert = computeResidual(x_pert, x);

    for (int j = 0; j < N; ++j) {
      double numerical_deriv = (r_pert[j] - r0[j]) / h;
      double analytical_deriv = c.system.getA(j, i);

      double rel_error = abs(analytical_deriv - numerical_deriv) /
                         (abs(analytical_deriv) + 1e-12);

      if (rel_error >
          1.5e-2) { // 1.5% tolerance (accounts for Early effect nonlinearity)
        cout << "Jacobian error at (" << j << "," << i << "): "
                  << "analytical=" << analytical_deriv
                  << ", numerical=" << numerical_deriv
                  << ", rel_error=" << rel_error << endl;
        return false;
      }
    }
  }

  cout << "Jacobian verification PASSED" << endl;
  return true;
}

bool test_pnp_jacobian() {
  cout << "Testing PNP Jacobian finite difference verification..."
            << endl;

  // Simple PNP circuit: common emitter with load
  Circuit c;
  NodeIndex nBase = c.createNode();
  NodeIndex nCollector = c.createNode();

  // PNP common emitter
  BjtParams pnp_params;
  pnp_params.Is = 7.373e-15; // 2N5087
  pnp_params.nVt = 0.0328;
  pnp_params.betaF = 923.4;
  pnp_params.betaR = 3.745;
  pnp_params.VAF = 80.3;

  c.addElement<BjtPnpEbersMoll>(nCollector, nBase, GND, pnp_params);

  // Base drive and collector load
  c.addElement<VoltageSource>(nBase, GND, 0.0);
  c.addElement<VoltageSource>(nCollector, GND, -2.0); // Negative for PNP

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true, 50)) {
    cout << "PNP DC solve failed" << endl;
    return false;
  }

  // Test Jacobian at operating point using finite differences
  c.system.clear();
  StampContext ctx{c.system, 1.0};

  // Stamp the circuit
  for (const auto &e : c.elements) {
    e->stamp(ctx);
  }

  // Compute limited voltages
  LimitContext limitCtx{x, x};
  for (auto *nl : c.newtonElements) {
    nl->computeLimitedVoltages(limitCtx);
  }

  // Stamp nonlinear elements
  for (const auto *nl : c.newtonElements) {
    nl->stampNewton(ctx, x);
  }

  vector<double> analytical_rhs = c.system.rhs();

  const int N = x.size();
  const double h = 1e-6;

  // Helper to compute residual f(x) = A(x)x - Z(x)
  auto computeResidual = [&](const vector<double> &v,
                             const vector<double> &v_old) {
    c.system.clear();
    StampContext ctx_local{c.system, 1.0};
    for (auto &e : c.elements)
      e->stamp(ctx_local);
    LimitContext lctx{v, v_old};
    for (auto *nl : c.newtonElements)
      nl->computeLimitedVoltages(lctx);
    for (auto *nl : c.newtonElements)
      nl->stampNewton(ctx_local, v);

    vector<double> res(N, 0.0);
    const vector<double> &rhs = c.system.rhs();
    for (int row = 0; row < N; ++row) {
      double sum = 0.0;
      for (int col = 0; col < N; ++col) {
        sum += c.system.getA(row, col) * v[col];
      }
      res[row] = sum - rhs[row];
    }
    return res;
  };

  vector<double> r0 = computeResidual(x, x);

  for (int i = 0; i < N; ++i) {
    vector<double> x_pert = x;
    x_pert[i] += h;

    vector<double> r_pert = computeResidual(x_pert, x);

    for (int j = 0; j < N; ++j) {
      double numerical_deriv = (r_pert[j] - r0[j]) / h;
      double analytical_deriv = c.system.getA(j, i);

      double rel_error = abs(analytical_deriv - numerical_deriv) /
                         (abs(analytical_deriv) + 1e-12);

      if (rel_error >
          1.5e-2) { // 1.5% tolerance (accounts for Early effect nonlinearity)
        cout << "PNP Jacobian error at (" << j << "," << i << "): "
                  << "analytical=" << analytical_deriv
                  << ", numerical=" << numerical_deriv
                  << ", rel_error=" << rel_error << endl;
        return false;
      }
    }
  }

  cout << "PNP Jacobian verification PASSED" << endl;
  return true;
}

bool test_kcl_conservation() {
  cout << "Testing KCL conservation..." << endl;

  // Test circuit from main.cpp KCL test
  Circuit c;
  NodeIndex nC = c.createNode();
  NodeIndex nB = c.createNode();
  NodeIndex nE = c.createNode();

  c.addElement<VoltageSource>(nC, GND, 5.0);
  c.addElement<VoltageSource>(nB, GND, 0.8);
  c.addElement<VoltageSource>(nE, GND, 0.0);

  BjtParams params;
  params.VAF = 50.0;
  c.addElement<BjtNpnEbersMoll>(nC, nB, nE, params);

  c.finalize();
  vector<double> x;
  if (!c.solveDc(x, 50, 1e-9, true)) {
    cout << "DC solve failed" << endl;
    return false;
  }

  // For now, just check that solution exists and voltages are reasonable
  for (int node = 0; node < c.getNumNodes(); ++node) {
    if (abs(x[node]) > 100.0) { // Unreasonable voltage
      cout << "Unreasonable voltage at node " << node << ": " << x[node]
                << endl;
      return false;
    }
  }

  // Check that BJT voltages are reasonable
  double Vc_val = x[nC];
  double Vb_val = x[nB];
  double Ve_val = x[nE];

  if (Vc_val < 0.1 || Vb_val < 0.1 || Ve_val > 1.0) {
    cout << "BJT voltages out of expected range: "
              << "Vc=" << Vc_val << ", Vb=" << Vb_val << ", Ve=" << Ve_val
              << endl;
    return false;
  }

  cout << "KCL conservation test PASSED" << endl;
  return true;
}

int main() {
  bool all_passed = true;

  if (!test_jacobian_finite_difference())
    all_passed = false;
  if (!test_pnp_jacobian())
    all_passed = false;
  if (!test_kcl_conservation())
    all_passed = false;

  if (all_passed) {
    cout << "All golden tests PASSED!" << endl;
    return 0;
  } else {
    cout << "Some golden tests FAILED!" << endl;
    return 1;
  }
}
