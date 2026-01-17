#include "api_2520_continuation.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

// API 2520 CONTINUATION TEST
// Implements patron's 4-phase strategy:
//   Phase 1: Supply ramp (0→±15V) with weak feedback
//   Phase 2: Feedback ramp (alpha: 0.001→1) at full supply
//   Phase 3: Instrumentation (verify physical solution)
//   Phase 4: Non-regression checks

struct ContinuationResult {
    bool success;
    double V_OUT, V_INM, V_INP, V_C2, VBE_Q2;
    double finalGmin;
    double gminFractionC2, gminFractionINM, gminFractionOUT;
};

// Run continuation and return final result
ContinuationResult runContinuation(bool verbose = true) {
    ContinuationResult result = {};
    
    if (verbose) {
        cout << "========================================" << endl;
        cout << "  API 2520 CONTINUATION SOLVE" << endl;
        cout << "========================================" << endl;
    }
    
    // Continuation parameters
    // Supply: 10 equal steps
    const vector<double> supplySteps = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // Alpha: MUCH finer steps to avoid latch-up transitions
    // Critical region is between 0.01 and 0.1 where latch can occur
    const vector<double> alphaSteps = {
        0.001, 0.003, 0.005, 0.007, 0.01,
        0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1,
        0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0
    };
    
    vector<double> x;  // Warm-start solution
    Api2520Continuation::NodeMap nodes;
    
    // ======================================================
    // PHASE 1: Supply Ramp with weak feedback (alpha=0.001)
    // ======================================================
    if (verbose) {
        cout << "\n--- PHASE 1: Supply Ramp (alpha=0.001) ---" << endl;
    }
    
    double alpha_weak = 0.001;
    
    for (double s : supplySteps) {
        Circuit c;
        nodes = Api2520Continuation::build(c, alpha_weak, s);
        
        // Warm-start from previous solution (scale voltages)
        if (!x.empty()) {
            // The circuit size may differ, so just use what we can
            vector<double> xInit(c.getNumNodes() + 10, 0.0);  // Rough estimate
            for (size_t i = 0; i < min(x.size(), xInit.size()); ++i) {
                xInit[i] = x[i];
            }
            x = xInit;
        }
        
        bool ok = c.solveDc(x, 200, 1e-6, false, 30);
        result.finalGmin = c.getFinalGmin();
        
        if (verbose) {
            cout << "  s=" << fixed << setprecision(2) << s 
                 << " | V(OUT)=" << setprecision(4) << x[nodes.OUT] 
                 << " | Gmin=" << scientific << setprecision(1) << result.finalGmin
                 << " | " << (ok ? "OK" : "FAIL") << endl;
        }
        
        if (!ok) {
            if (verbose) cout << "Supply ramp failed at s=" << s << endl;
            result.success = false;
            return result;
        }
    }
    
    // ======================================================
    // PHASE 2: Feedback Ramp at full supply WITH BACKTRACKING
    // ======================================================
    if (verbose) {
        cout << "\n--- PHASE 2: Feedback Ramp (supplyScale=1.0) ---" << endl;
        cout << "Strategy: Bisection with backtracking when latch detected" << endl;
    }
    
    // Good solution checkpoint
    vector<double> xGood = x;
    double alphaGood = 0.001;
    double alphaTarget = 1.0;
    
    // Use adaptive stepping with bisection
    double alphaCurrent = 0.001;
    double alphaStep = 0.01;  // Initial step size
    const double alphaStepMin = 0.001;  // Minimum step
    int maxBisections = 50;
    int bisectionCount = 0;
    
    while (alphaCurrent < alphaTarget && bisectionCount < maxBisections) {
        bisectionCount++;
        double alphaNext = min(alphaCurrent + alphaStep, alphaTarget);
        
        Circuit c;
        nodes = Api2520Continuation::build(c, alphaNext, 1.0);
        
        // Warm-start from good solution
        vector<double> xTry = xGood;
        if ((int)xTry.size() < c.getNumNodes() + 10) {
            xTry.resize(c.getNumNodes() + 10, 0.0);
        }
        
        bool ok = c.solveDc(xTry, 200, 1e-6, false, 30);
        double gmin = c.getFinalGmin();
        double vout = xTry[nodes.OUT];
        
        double rfb = Api2520Continuation::R_FB_NOMINAL / max(alphaNext, Api2520Continuation::ALPHA_MIN);
        
        // STRICT acceptance: solution is REJECTED if Gmin > target or OUT rails
        // A solution with Gmin=1e-5 is NOT OK - it's artificially stabilized!
        const double GMIN_TARGET = 1e-12;
        bool solutionBad = (!ok) || (gmin > GMIN_TARGET) || (abs(vout) > 5.0);
        
        if (verbose) {
            cout << "  alpha=" << fixed << setprecision(4) << alphaNext 
                 << " (R_fb=" << scientific << setprecision(1) << rfb << " Ohm)"
                 << " | V(OUT)=" << fixed << setprecision(4) << vout
                 << " | Gmin=" << scientific << setprecision(1) << gmin;
            if (solutionBad) {
                cout << " [REJECT]";
            } else {
                cout << " [ACCEPT]";
            }
            cout << endl;
        }
        
        if (solutionBad) {
            // Solution rejected - reduce step and try again from last good
            alphaStep = max(alphaStep * 0.5, alphaStepMin);
            if (verbose) {
                cout << "    -> REJECTED (Gmin=" << scientific << gmin << " or OUT=" << fixed << vout << ")";
                cout << ", reducing step to " << alphaStep << endl;
            }
            
            // If step is at minimum and still failing, we're stuck
            if (alphaStep <= alphaStepMin) {
                if (verbose) cout << "    -> Minimum step reached at alpha=" << alphaCurrent << endl;
                // DON'T accept the bad solution! Stay at last good.
                break;
            }
            continue;
        }
        
        // Good step! Accept and possibly increase step size
        x = xTry;
        alphaCurrent = alphaNext;
        xGood = x;
        alphaGood = alphaCurrent;
        
        // Increase step if we've been succeeding
        alphaStep = min(alphaStep * 1.5, 0.2);
        
        result.finalGmin = gmin;
        
        if (verbose && alphaCurrent >= alphaTarget - 0.001) {
            cout << "    -> Reached target alpha=" << alphaTarget << endl;
        }
    }
    
    if (verbose) {
        cout << "\nFinal alpha achieved: " << alphaGood << " (target: " << alphaTarget << ")" << endl;
    }
    
    // If we didn't reach alpha=1, rebuild final circuit at alpha=1 with best guess
    if (alphaGood < 0.999) {
        if (verbose) cout << "Attempting final solve at alpha=1.0..." << endl;
        Circuit c;
        nodes = Api2520Continuation::build(c, 1.0, 1.0);
        
        if ((int)x.size() < c.getNumNodes() + 10) {
            x.resize(c.getNumNodes() + 10, 0.0);
        }
        
        bool ok = c.solveDc(x, 300, 1e-6, false, 50);
        result.finalGmin = c.getFinalGmin();
        
        if (verbose) {
            cout << "  alpha=1.000 | V(OUT)=" << fixed << setprecision(4) << x[nodes.OUT]
                 << " | Gmin=" << scientific << setprecision(1) << result.finalGmin
                 << " | " << (ok ? "OK" : "FAIL") << endl;
        }
    }

    if (verbose) {
        cout << "\n========================================" << endl;
        cout << "  PHASE 3: INSTRUMENTATION" << endl;
        cout << "========================================" << endl;
    }
    
    // Extract key voltages
    result.V_OUT = x[nodes.OUT];
    result.V_INM = x[nodes.INM];
    result.V_INP = x[nodes.INP];
    result.V_C2 = x[nodes.C2];
    
    // VBE_Q2 = V(Q2_Bi) - V(Q2_Ei)
    result.VBE_Q2 = x[nodes.Q2_Bi] - x[nodes.Q2_Ei];
    
    if (verbose) {
        cout << fixed << setprecision(4);
        cout << "\n=== KEY VOLTAGES ===" << endl;
        cout << "V(INP)     = " << result.V_INP << " V" << endl;
        cout << "V(INM)     = " << result.V_INM << " V" << endl;
        cout << "V(OUT)     = " << result.V_OUT << " V" << endl;
        cout << "V(C2)      = " << result.V_C2 << " V (Q4 base)" << endl;
        cout << "V(Q2_Bi)   = " << x[nodes.Q2_Bi] << " V" << endl;
        cout << "V(Q2_Ei)   = " << x[nodes.Q2_Ei] << " V" << endl;
        cout << "VBE_Q2     = " << result.VBE_Q2 << " V";
        if (result.VBE_Q2 > 0.55 && result.VBE_Q2 < 0.75) {
            cout << " [CONDUCTING]" << endl;
        } else if (result.VBE_Q2 < 0.2) {
            cout << " *** CUTOFF! ***" << endl;
        } else {
            cout << " [MARGINAL]" << endl;
        }
        cout << "\nFinal Gmin = " << scientific << setprecision(2) << result.finalGmin << " S" << endl;
    }
    
    // ======================================================
    // Gmin Current Fraction Analysis
    // ======================================================
    if (verbose) {
        cout << "\n=== GMIN CURRENT ANALYSIS ===" << endl;
    }
    
    // To compute Gmin fraction, we need KCL at each node
    // I_gmin = Gmin * V_node
    // Total current estimated from resistor loads
    
    // At nC2: main currents are from Q9 collector, Q2 collector, Q4 base
    // We approximate total as |I_gmin| / |V_node / typical_R|
    double i_gmin_c2 = result.finalGmin * result.V_C2;
    double i_gmin_inm = result.finalGmin * result.V_INM;
    double i_gmin_out = result.finalGmin * result.V_OUT;
    
    // Approximate expected currents
    // At C2: mirror current ~100µA
    double i_expected_c2 = 100e-6;
    // At INM: feedback current ~V_OUT/R_fb
    double i_expected_inm = max(1e-9, abs(result.V_OUT) / 10000.0);
    // At OUT: load current ~V_OUT/R_load
    double i_expected_out = max(1e-9, abs(result.V_OUT) / 10000.0 + 1e-6);
    
    result.gminFractionC2 = abs(i_gmin_c2) / i_expected_c2 * 100.0;
    result.gminFractionINM = abs(i_gmin_inm) / max(i_expected_inm, 1e-9) * 100.0;
    result.gminFractionOUT = abs(i_gmin_out) / max(i_expected_out, 1e-9) * 100.0;
    
    if (verbose) {
        cout << fixed << setprecision(4);
        cout << "I_gmin(C2)  = " << i_gmin_c2 * 1e6 << " uA";
        cout << " (" << setprecision(2) << result.gminFractionC2 << "% of expected)" << endl;
        cout << "I_gmin(INM) = " << i_gmin_inm * 1e6 << " uA";
        cout << " (" << setprecision(2) << result.gminFractionINM << "% of expected)" << endl;
        cout << "I_gmin(OUT) = " << i_gmin_out * 1e6 << " uA";
        cout << " (" << setprecision(2) << result.gminFractionOUT << "% of expected)" << endl;
    }
    
    // ======================================================
    // PHASE 4: Non-Regression Checks
    // ======================================================
    if (verbose) {
        cout << "\n========================================" << endl;
        cout << "  PHASE 4: ACCEPTANCE CRITERIA" << endl;
        cout << "========================================" << endl;
    }
    
    bool out_ok = abs(result.V_OUT) < 0.5;
    bool vbe_ok = (result.VBE_Q2 > 0.55 && result.VBE_Q2 < 0.75);
    bool gmin_ok = (result.finalGmin <= 1e-12);
    bool gmin_fraction_ok = (result.gminFractionC2 < 1.0) && (result.gminFractionINM < 10.0);
    bool c2_not_vcc = (result.V_C2 < 14.0);
    
    if (verbose) {
        cout << "|V(OUT)| < 0.5V:           " << (out_ok ? "PASS" : "FAIL") 
             << " (" << result.V_OUT << "V)" << endl;
        cout << "0.55 < VBE_Q2 < 0.75V:     " << (vbe_ok ? "PASS" : "FAIL")
             << " (" << result.VBE_Q2 << "V)" << endl;
        cout << "finalGmin <= 1e-12:        " << (gmin_ok ? "PASS" : "FAIL")
             << " (" << scientific << result.finalGmin << ")" << endl;
        cout << fixed;
        cout << "Gmin fraction < 1%:        " << (gmin_fraction_ok ? "PASS" : "FAIL")
             << " (C2=" << result.gminFractionC2 << "%)" << endl;
        cout << "nC2 not at VCC:            " << (c2_not_vcc ? "PASS" : "FAIL")
             << " (" << result.V_C2 << "V)" << endl;
    }
    
    result.success = out_ok && vbe_ok && gmin_ok && gmin_fraction_ok && c2_not_vcc;
    
    if (verbose) {
        cout << "\n========================================" << endl;
        if (result.success) {
            cout << "  *** VICTORY: PHYSICAL DC POINT FOUND ***" << endl;
        } else {
            cout << "  *** FAIL: Criteria not met ***" << endl;
        }
        cout << "========================================" << endl;
    }
    
    return result;
}

int main() {
    ContinuationResult result = runContinuation(true);
    return result.success ? 0 : 1;
}
