#pragma once
#include "linear_system.h"
#include "mna_types.h"

// Standard MNA stamps

inline void stampConductance(DenseLinearSystem &sys, NodeIndex a, NodeIndex b,
                             double g) {
  // Conductance between a and b:
  // +g on (a,a) and (b,b), -g on (a,b) and (b,a)
  if (a != GND)
    sys.addA(a, a, +g);
  if (b != GND)
    sys.addA(b, b, +g);

  if (a != GND && b != GND) {
    sys.addA(a, b, -g);
    sys.addA(b, a, -g);
  }
}

// Current source from a -> b of value I
// KCL convention: leaving node = positive.
// This stamp injects +I leaving a and -I leaving b.
inline void stampCurrentSource(DenseLinearSystem &sys, NodeIndex a, NodeIndex b,
                               double I) {
  if (a != GND)
    sys.addZ(a, -I); // Move to RHS with sign consistent with A*x = z (KCL)
  if (b != GND)
    sys.addZ(b, +I);
}
