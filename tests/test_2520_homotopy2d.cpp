#include "api_2520_homotopy.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

// =============================================================================
// 2D HOMOTOPY TEST FOR API 2520
// =============================================================================
// Strategy:
//   Phase A: Ramp sourceScale 0→1, alpha=0 (feedback open), bleeder ON
//            ALL sources scale (VCC, VEE, I_Tail, I_VAS, I_Bias)
//   Phase B: Ramp alpha 0→1, sourceScale=1, bleeder strength → 0
//   Phase C: Final solve with bleeder OFF (verify true circuit)
//
// WHY this works:
// 1. At sourceScale=0, circuit is at trivial 0V solution
// 2. As sources ramp, transistors bias from balanced state (not from guess)
// 3. Bleeder at nC2 prevents mirror node from floating to VCC when Q2 weak
// 4. Feedback open during source ramp prevents latch instability
// 5. Once biased, alpha ramp engages feedback while bleeder fades
// 6. Final state: true circuit with no helper elements
// =============================================================================

struct HomotopyResult {
    bool success;
    double V_OUT, V_INM, V_C2, VBE_Q2;
    double finalGmin;
    double maxAlphaReached;
};

HomotopyResult run2DHomotopy(bool verbose = true) {
    HomotopyResult result = {};
    
    if (verbose) {
        cout << "========================================" << endl;
        cout << "  API 2520 2D HOMOTOPY" << endl;
        cout << "========================================" << endl;
        cout << "Source scaling + bleeder anti-latch strategy" << endl;
    }
    
    // Build circuit ONCE
    Circuit c;
    Api2520Homotopy::Sources src;
    Api2520Homotopy::Nodes nodes;
    Api2520Homotopy::build(c, src, nodes);
    
    // Initialize solution vector
    vector<double> x;
    
    // Homotopy parameters
    const double ALPHA_OPEN = 0.001;       // Feedback effectively open
    const double BLEEDER_G_START = 1e-4;   // ~10kΩ bleeder - stronger to prevent latch
    const double BLEEDER_G_END = 0.0;      // Bleeder off at end
    
    // =========================================================================
    // PHASE A: Source Ramp (alpha=0, bleeder ON)
    // =========================================================================
    if (verbose) {
        cout << "\n--- PHASE A: Source Ramp (alpha=0, bleeder ON) ---" << endl;
    }
    
    // Finer source steps to find exact failure point
    const vector<double> sourceSteps = {
        0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5,
        0.55, 0.6, 0.62, 0.64, 0.66, 0.68, 0.7, 0.72, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0
    };
    
    for (double s : sourceSteps) {
        // KEY INSIGHT: Engage feedback proportionally to sources to stabilize OUT!
        // At s=0, alpha=0 (open). As sources ramp, feedback gradually engages.
        // This prevents the open-loop drift that causes OUT to rail.
        double alpha = max(0.001, s * 0.1);  // At s=1, alpha=0.1 (R_fb=100kΩ)
        
        Api2520Homotopy::updateSources(src, s, alpha, BLEEDER_G_START);
        
        bool ok = c.solveDc(x, 200, 1e-6, false, 30);
        double gmin = c.getFinalGmin();
        
        auto diag = Api2520Homotopy::getDiagnostics(x, nodes, gmin);
        
        if (verbose) {
            cout << "s=" << s << " OUT=" << diag.V_OUT << " VBE=" << diag.VBE_Q2 
                 << " C2=" << diag.V_C2 << " Gmin=" << diag.finalGmin
                 << (diag.isLatched ? " LATCH" : " OK") << endl;
        }
        
        if (!ok || diag.isLatched) {
            if (verbose) {
                cout << "  -> Source ramp issue at s=" << s << endl;
                cout << "     solveDc returned: " << (ok ? "OK" : "FAIL") << endl;
                cout << "     VBE_Q2=" << diag.VBE_Q2 << " (latch if <0.1)" << endl;
                cout << "     V(C2)=" << diag.V_C2 << " (latch if >14)" << endl;
                cout << "     V(OUT)=" << diag.V_OUT << " (latch if |OUT|>8)" << endl;
            }
            // Try pseudo-transient recovery
            if (verbose) cout << "  -> Attempting pseudo-transient recovery..." << endl;
            ok = c.solveDcPseudoTransient(x, 1e-3, 1e-6);
            gmin = c.getFinalGmin();
            diag = Api2520Homotopy::getDiagnostics(x, nodes, gmin);
            if (verbose) {
                cout << "  -> After PT: ";
                diag.print();
            }
            if (!ok || diag.isLatched) {
                result.success = false;
                result.maxAlphaReached = 0.0;
                return result;
            }
        }
    }
    
    // Save good solution before alpha ramp
    vector<double> xAfterSourceRamp = x;
    
    // =========================================================================
    // PHASE B: Alpha Ramp (sourceScale=1, bleeder → 0)
    // =========================================================================
    if (verbose) {
        cout << "\n--- PHASE B: Alpha Ramp (sources=1, bleeder fading) ---" << endl;
    }
    
    // Adaptive stepping for alpha
    double alpha = ALPHA_OPEN;
    double alphaStep = 0.01;
    const double ALPHA_STEP_MIN = 0.001;
    const double ALPHA_TARGET = 1.0;
    int maxSteps = 100;
    int stepCount = 0;
    
    vector<double> xGood = x;
    double alphaGood = alpha;
    
    while (alpha < ALPHA_TARGET && stepCount < maxSteps) {
        stepCount++;
        double alphaNext = min(alpha + alphaStep, ALPHA_TARGET);
        
        // Bleeder fades as alpha increases: at alpha=1, bleeder is off
        // Linear interpolation from BLEEDER_G_START to 0
        double bleederG = BLEEDER_G_START * (1.0 - alphaNext);
        
        // Update sources
        Api2520Homotopy::updateSources(src, 1.0, alphaNext, bleederG);
        
        bool ok = c.solveDc(x, 200, 1e-6, false, 30);
        double gmin = c.getFinalGmin();
        auto diag = Api2520Homotopy::getDiagnostics(x, nodes, gmin);
        
        if (verbose) {
            double rfb = Api2520Homotopy::R_FB_NOMINAL / max(alphaNext, Api2520Homotopy::ALPHA_MIN);
            cout << "  alpha=" << fixed << setprecision(4) << alphaNext
                 << " R_fb=" << scientific << setprecision(1) << rfb
                 << " bleed=" << bleederG;
            diag.print();
        }
        
        bool acceptable = Api2520Homotopy::isAcceptable(diag);
        
        if (!ok || !acceptable) {
            // REJECT: reduce step, restore from last good
            alphaStep = max(alphaStep * 0.5, ALPHA_STEP_MIN);
            x = xGood;
            
            if (verbose) {
                cout << "    -> REJECTED, reducing step to " << alphaStep << endl;
            }
            
            // Try pseudo-transient from good solution
            if (alphaStep <= ALPHA_STEP_MIN) {
                if (verbose) cout << "    -> Trying pseudo-transient at alpha=" << alphaNext << "..." << endl;
                x = xGood;
                Api2520Homotopy::updateSources(src, 1.0, alphaNext, bleederG);
                ok = c.solveDcPseudoTransient(x, 1e-3, 1e-6);
                gmin = c.getFinalGmin();
                diag = Api2520Homotopy::getDiagnostics(x, nodes, gmin);
                
                if (verbose) {
                    cout << "    -> After PT: ";
                    diag.print();
                }
                
                if (ok && Api2520Homotopy::isAcceptable(diag)) {
                    // PT worked! Accept and continue
                    alpha = alphaNext;
                    xGood = x;
                    alphaGood = alpha;
                    alphaStep = ALPHA_STEP_MIN * 2;
                } else {
                    // Still failing at minimum step - we're stuck
                    if (verbose) cout << "    -> Stuck at alpha=" << alphaGood << endl;
                    break;
                }
            }
            continue;
        }
        
        // ACCEPT
        alpha = alphaNext;
        xGood = x;
        alphaGood = alpha;
        
        // Increase step on success
        alphaStep = min(alphaStep * 1.3, 0.1);
    }
    
    result.maxAlphaReached = alphaGood;
    
    // =========================================================================
    // PHASE C: Final verification (bleeder fully OFF)
    // =========================================================================
    if (verbose) {
        cout << "\n--- PHASE C: Final Verification (bleeder OFF) ---" << endl;
    }
    
    // Set final circuit: alpha=1, bleeder=0
    Api2520Homotopy::updateSources(src, 1.0, 1.0, 0.0);
    
    // Use best solution as starting point
    x = xGood;
    
    bool finalOk = c.solveDc(x, 300, 1e-6, false, 50);
    double finalGmin = c.getFinalGmin();
    auto finalDiag = Api2520Homotopy::getDiagnostics(x, nodes, finalGmin);
    
    if (verbose) {
        cout << "  Final (alpha=1, bleeder=0):";
        finalDiag.print();
    }
    
    if (!finalOk || finalDiag.isLatched) {
        if (verbose) cout << "  -> Final solve latched, trying PT..." << endl;
        x = xGood;
        finalOk = c.solveDcPseudoTransient(x, 1e-3, 1e-6);
        finalGmin = c.getFinalGmin();
        finalDiag = Api2520Homotopy::getDiagnostics(x, nodes, finalGmin);
        if (verbose) {
            cout << "  -> After final PT:";
            finalDiag.print();
        }
    }
    
    // =========================================================================
    // RESULTS
    // =========================================================================
    result.V_OUT = finalDiag.V_OUT;
    result.V_INM = finalDiag.V_INM;
    result.V_C2 = finalDiag.V_C2;
    result.VBE_Q2 = finalDiag.VBE_Q2;
    result.finalGmin = finalGmin;
    
    if (verbose) {
        cout << "\n========================================" << endl;
        cout << "  FINAL RESULTS" << endl;
        cout << "========================================" << endl;
        cout << fixed << setprecision(4);
        cout << "Max alpha reached: " << result.maxAlphaReached << " (target: 1.0)" << endl;
        cout << "V(OUT) = " << result.V_OUT << " V (want |OUT| < 0.5V)" << endl;
        cout << "V(INM) = " << result.V_INM << " V (want ~0V)" << endl;
        cout << "V(C2)  = " << result.V_C2 << " V (want NOT at VCC)" << endl;
        cout << "VBE_Q2 = " << result.VBE_Q2 << " V (want 0.55-0.75V)" << endl;
        cout << "Gmin   = " << scientific << result.finalGmin << " S (want <=1e-12)" << endl;
    }
    
    // Check acceptance criteria
    bool out_ok = abs(result.V_OUT) < 0.5;
    bool vbe_ok = (result.VBE_Q2 > 0.55 && result.VBE_Q2 < 0.75);
    bool gmin_ok = (result.finalGmin <= 1e-12);
    bool c2_ok = (result.V_C2 < 13.0);  // Not pinned to VCC
    bool alpha_ok = (result.maxAlphaReached > 0.99);
    
    result.success = out_ok && vbe_ok && gmin_ok && c2_ok && alpha_ok;
    
    if (verbose) {
        cout << "\n=== ACCEPTANCE CRITERIA ===" << endl;
        cout << "|V(OUT)| < 0.5V:        " << (out_ok ? "PASS" : "FAIL") << endl;
        cout << "0.55 < VBE_Q2 < 0.75V:  " << (vbe_ok ? "PASS" : "FAIL") << endl;
        cout << "finalGmin <= 1e-12:     " << (gmin_ok ? "PASS" : "FAIL") << endl;
        cout << "nC2 not at VCC:         " << (c2_ok ? "PASS" : "FAIL") << endl;
        cout << "alpha reached 1.0:      " << (alpha_ok ? "PASS" : "FAIL") << endl;
        
        cout << "\n========================================" << endl;
        if (result.success) {
            cout << "  *** SUCCESS: 2D HOMOTOPY CONVERGED ***" << endl;
        } else {
            cout << "  *** FAIL: Criteria not met ***" << endl;
        }
        cout << "========================================" << endl;
    }
    
    return result;
}

int main() {
    HomotopyResult result = run2DHomotopy(true);
    return result.success ? 0 : 1;
}
