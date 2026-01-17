#include "bjt_ebers_moll.h"
#include "circuit.h"
#include "current_source.h"
#include "resistor.h"
#include "spice_models_2520.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// MICRO-TEST UNITAIRE PNP
// Per patron: Must verify PNP model with simple configuration:
// Ve=+15, Vb=+14.3, Vc=0, with reasonable resistors
// Check: currents at 3 terminals, Ic/Ib/Ie signs, Jacobian sensitivity

int main() {
    cout << "========================================" << endl;
    cout << "  MICRO-TEST UNITAIRE PNP" << endl;
    cout << "========================================" << endl;
    cout << "\nThis test validates the PNP BJT model" << endl;
    cout << "before debugging the full 2520 circuit." << endl;
    
    // ======================================================
    // SIMPLE PNP CONFIGURATION
    // ======================================================
    // Emitter at +15V (via small Re)
    // Base at +14.3V (VEB ≈ 0.7V for active)
    // Collector at 0V (via Rc)
    // Expected: Ie > 0 (entering emitter), Ic < 0 (leaving collector)
    
    Circuit c;
    
    NodeIndex nVCC = c.createNode("VCC");
    NodeIndex nE = c.createNode("E_PNP");
    NodeIndex nB = c.createNode("B_PNP");
    NodeIndex nC = c.createNode("C_PNP");
    
    // Supplies
    c.addElement<VoltageSource>("Vcc", nVCC, GND, 15.0);
    c.addElement<VoltageSource>("Vbase", nB, GND, 14.3);  // Fix base at 14.3V
    
    // PNP transistor (using BC416C from 2520)
    auto q_pnp = spiceToBjtParams(getSpiceBjtModel("BC416C"));
    
    // For this test, we use the raw BJT (no extended parasitic resistors)
    // to isolate the core model behavior
    c.addElement<BjtPnpEbersMoll>("Q_test", nC, nB, nE, q_pnp);
    
    // Bias resistors
    c.addElement<Resistor>("Re", nVCC, nE, 1000.0);  // 1kΩ from VCC to emitter
    c.addElement<Resistor>("Rc", nC, GND, 5000.0);   // 5kΩ from collector to GND
    
    // ======================================================
    // SOLVE DC
    // ======================================================
    vector<double> x;
    bool success = c.solveDc(x);
    double finalGmin = c.getFinalGmin();
    
    double V_E = x[nE];
    double V_B = x[nB];
    double V_C = x[nC];
    double V_VCC = x[nVCC];
    
    // Compute currents from terminal voltages
    double I_Re = (V_VCC - V_E) / 1000.0;  // Current in Re (from VCC toward E)
    double I_Rc = V_C / 5000.0;            // Current in Rc (from C toward GND)
    
    // For PNP in active mode:
    // Current flows: Emitter IN (conventional), Collector OUT (conventional)
    // Ie ≈ I_Re (enters emitter)
    // Ic ≈ -I_Rc (leaves collector, goes to Rc then GND)
    // Ib = Ie - Ic (very small)
    
    double VEB = V_E - V_B;  // Should be ~0.7V for active PNP
    double VCB = V_C - V_B;  // Should be strongly negative (reverse biased)
    
    cout << fixed << setprecision(4);
    cout << "\n=== SOLUTION ===" << endl;
    cout << "DC Solve: " << (success ? "SUCCESS" : "FAIL") << endl;
    cout << "Final Gmin: " << scientific << setprecision(2) << finalGmin << " S" << endl;
    
    cout << fixed << setprecision(4);
    cout << "\n=== NODE VOLTAGES ===" << endl;
    cout << "V(VCC) = " << V_VCC << " V (should be 15.0)" << endl;
    cout << "V(E)   = " << V_E << " V" << endl;
    cout << "V(B)   = " << V_B << " V (should be 14.3)" << endl;
    cout << "V(C)   = " << V_C << " V" << endl;
    
    cout << "\n=== JUNCTION VOLTAGES ===" << endl;
    cout << "VEB = V(E) - V(B) = " << VEB << " V";
    if (VEB > 0.5 && VEB < 0.9) {
        cout << " [ACTIVE - CORRECT]" << endl;
    } else if (VEB < 0.3) {
        cout << " [CUTOFF - WRONG!]" << endl;
    } else if (VEB > 0.9) {
        cout << " [SATURATED?]" << endl;
    } else {
        cout << " [MARGINAL]" << endl;
    }
    
    cout << "VCB = V(C) - V(B) = " << VCB << " V";
    if (VCB < -0.3) {
        cout << " [REVERSE BIASED - CORRECT]" << endl;
    } else if (VCB > 0.3) {
        cout << " [FORWARD BIASED - SATURATION]" << endl;
    } else {
        cout << " [EDGE OF SATURATION]" << endl;
    }
    
    cout << "\n=== CURRENTS (computed from terminal resistors) ===" << endl;
    cout << "I(Re) = (VCC-Ve)/Re = " << I_Re * 1e3 << " mA";
    cout << " = I_EMITTER entering" << endl;
    
    cout << "I(Rc) = Vc/Rc = " << I_Rc * 1e3 << " mA";
    cout << " = I_COLLECTOR leaving" << endl;
    
    double I_base_computed = I_Re - I_Rc;  // KCL at BJT
    cout << "I_BASE = Ie - Ic = " << I_base_computed * 1e6 << " uA" << endl;
    
    double beta_measured = I_Rc / I_base_computed;
    cout << "\nMeasured beta = Ic/Ib = " << beta_measured;
    cout << " (expected: " << q_pnp.betaF << ")" << endl;
    
    // ======================================================
    // JACOBIAN SENSITIVITY TEST
    // ======================================================
    cout << "\n=== JACOBIAN SENSITIVITY TEST ===" << endl;
    cout << "Testing: small ΔVb should change Ib in correct direction" << endl;
    
    // Create a second circuit with slightly different Vbase
    double delta_Vb = 0.01;  // +10mV
    
    Circuit c2;
    NodeIndex nVCC2 = c2.createNode("VCC");
    NodeIndex nE2 = c2.createNode("E_PNP");
    NodeIndex nB2 = c2.createNode("B_PNP");
    NodeIndex nC2 = c2.createNode("C_PNP");
    
    c2.addElement<VoltageSource>("Vcc", nVCC2, GND, 15.0);
    c2.addElement<VoltageSource>("Vbase", nB2, GND, 14.3 + delta_Vb);  // Slightly higher
    c2.addElement<BjtPnpEbersMoll>("Q_test", nC2, nB2, nE2, q_pnp);
    c2.addElement<Resistor>("Re", nVCC2, nE2, 1000.0);
    c2.addElement<Resistor>("Rc", nC2, GND, 5000.0);
    
    vector<double> x2;
    c2.solveDc(x2);
    
    double V_E2 = x2[nE2];
    double V_C2 = x2[nC2];
    double I_Re2 = (15.0 - V_E2) / 1000.0;
    double I_Rc2 = V_C2 / 5000.0;
    double I_base2 = I_Re2 - I_Rc2;
    
    double dIb_dVb = (I_base2 - I_base_computed) / delta_Vb;
    
    cout << "Vb = 14.3V + " << delta_Vb*1000 << "mV" << endl;
    cout << "Ib(new) = " << I_base2 * 1e6 << " uA" << endl;
    cout << "ΔIb = " << (I_base2 - I_base_computed) * 1e6 << " uA" << endl;
    cout << "dIb/dVb = " << dIb_dVb * 1e3 << " mA/V" << endl;
    
    // For PNP: raising Vb (making VEB smaller) should REDUCE Ib
    // So dIb/dVb should be NEGATIVE
    cout << "\nExpected: dIb/dVb < 0 (raising Vb reduces VEB, reduces currents)" << endl;
    if (dIb_dVb < 0) {
        cout << "Result: dIb/dVb = " << dIb_dVb * 1e3 << " mA/V [CORRECT SIGN]" << endl;
    } else {
        cout << "Result: dIb/dVb = " << dIb_dVb * 1e3 << " mA/V [*** WRONG SIGN! ***]" << endl;
    }
    
    // ======================================================
    // VERDICT
    // ======================================================
    cout << "\n========================================" << endl;
    cout << "  VERDICT" << endl;
    cout << "========================================" << endl;
    
    bool veb_ok = (VEB > 0.5 && VEB < 0.9);
    bool vcb_ok = (VCB < 0);
    bool currents_ok = (I_Re > 0.05e-3 && I_Rc > 0.05e-3);  // At least 0.05mA
    bool beta_ok = (beta_measured > 30 && beta_measured < 600);
    bool jacobian_ok = (dIb_dVb < 0);
    bool gmin_ok = (finalGmin < 1e-8);
    
    bool all_pass = success && veb_ok && vcb_ok && currents_ok && beta_ok && jacobian_ok && gmin_ok;
    
    if (all_pass) {
        cout << "*** PNP MODEL VALIDATED ***" << endl;
        cout << "All checks passed. Model stamps and signs are correct." << endl;
        return 0;
    } else {
        cout << "*** PNP MODEL HAS ISSUES ***" << endl;
        if (!success) cout << "- DC solve failed" << endl;
        if (!veb_ok) cout << "- VEB not in active range (0.5-0.9V)" << endl;
        if (!vcb_ok) cout << "- VCB not reverse biased" << endl;
        if (!currents_ok) cout << "- Currents too low" << endl;
        if (!beta_ok) cout << "- Beta out of range (30-500)" << endl;
        if (!jacobian_ok) cout << "- Jacobian sign wrong (dIb/dVb should be < 0)" << endl;
        if (!gmin_ok) cout << "- Gmin contamination (> 1e-8 S)" << endl;
        return 1;
    }
}
