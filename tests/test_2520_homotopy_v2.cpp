#include "api_2520_homotopy_v2.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

// =============================================================================
// 2D HOMOTOPY v2 - PATRON'S STRATEGY
// =============================================================================
// Key changes from v1:
// 1. REMOVE bleeder on nC2 (it was pulling C2 negative, causing OUT saturation)
// 2. ADD INM clamp (high G at start, fades to 0) to keep Q2 conducting
// 3. NEVER fully open feedback (alpha >= 0.01 always)
// 4. Use PT (pseudo-transient) THEN DC at each step for robustness
// 5. Adaptive stepping with backtracking to last good state
//
// Homotopy path:
//   Phase A: Ramp sources 0→1 with strong INM clamp + moderate feedback
//   Phase B: Ramp alpha 0.01→1 while fading INM clamp
//   Phase C: Verify final solution with clamp OFF
// =============================================================================

struct HomotopyResult {
    bool success;
    double V_OUT, V_INM, V_C2, VBE_Q2;
    double finalGmin;
    double maxAlphaReached;
};

HomotopyResult run2DHomotopyV2(bool verbose = true) {
    HomotopyResult result = {};
    
    if (verbose) {
        cout << "========================================" << endl;
        cout << "  API 2520 2D HOMOTOPY v2" << endl;
        cout << "  (INM clamp + alpha never open)" << endl;
        cout << "========================================" << endl;
    }
    
    Circuit c;
    Api2520HomotopyV2::Sources src;
    Api2520HomotopyV2::Nodes nodes;
    Api2520HomotopyV2::build(c, src, nodes);
    
    vector<double> x;
    
    // Homotopy parameters
    const double ALPHA_START = 0.01;  // Never fully open
    const double CLAMP_G_START = 0.01;  // Strong clamp at start (100Ω)
    
    // =========================================================================
    // PHASE A: Source Ramp with moderate feedback + strong INM clamp
    // =========================================================================
    if (verbose) {
        cout << "\n--- PHASE A: Source Ramp (clamp ON, fb weak) ---" << endl;
    }
    
    const vector<double> sourceSteps = {
        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0
    };
    
    // During source ramp: alpha grows from 0.01 to 0.1, clamp stays strong
    for (double s : sourceSteps) {
        double alpha = ALPHA_START + (0.1 - ALPHA_START) * s;  // 0.01 → 0.1
        double clampG = CLAMP_G_START;  // Keep strong during source ramp
        
        Api2520HomotopyV2::updateSources(src, s, alpha, clampG);
        
        // PT first to find good initial guess, then DC to polish
        // Phase A: KEEP internal source stepping for stability during initial ramp
        // ("rampe dans rampe" - stabilizes early steps where nodesets may conflict)
        bool okPT = c.solveDcPseudoTransient(x, 1e-3, 1e-6);
        bool okDC = c.solveDc(x, 200, 1e-6, false, 30);
        double gmin = c.getFinalGmin();
        
        auto diag = Api2520HomotopyV2::getDiagnostics(x, nodes, gmin);
        
        if (verbose) {
            cout << "s=" << fixed << setprecision(2) << s 
                 << " a=" << setprecision(3) << alpha << " ";
            diag.print();
        }
        
        if (!okDC || diag.isLatched) {
            if (verbose) {
                cout << "  -> Source ramp issue at s=" << s << endl;
                cout << "     PT: " << (okPT ? "OK" : "FAIL") << ", DC: " << (okDC ? "OK" : "FAIL") << endl;
            }
            result.success = false;
            result.maxAlphaReached = alpha;
            return result;
        }
    }
    
    vector<double> xGood = x;
    double alphaGood = 0.1;
    
    // =========================================================================
    // PHASE B: Alpha Ramp with fading INM clamp
    // =========================================================================
    if (verbose) {
        cout << "\n--- PHASE B: Alpha Ramp (clamp fading) ---" << endl;
    }
    
    double alpha = 0.1;
    double alphaStep = 0.05;
    const double ALPHA_STEP_MIN = 0.01;
    const double ALPHA_TARGET = 1.0;
    int maxSteps = 100;
    
    for (int step = 0; step < maxSteps && alpha < ALPHA_TARGET; step++) {
        double alphaNext = min(alpha + alphaStep, ALPHA_TARGET);
        
        // Clamp fades SLOWLY as alpha increases: at alpha=1, clamp is off
        // Use (1-alpha)^4 for MUCH slower fade - keeps INM stabilized longer
        double clampG = CLAMP_G_START * pow(1.0 - alphaNext, 4);
        
        Api2520HomotopyV2::updateSources(src, 1.0, alphaNext, clampG);
        
        // Try PT then DC (disableSourceStepping=true for external homotopy)
        x = xGood;  // Start from last good
        bool okPT = c.solveDcPseudoTransient(x, 1e-3, 1e-6, true);
        bool okDC = c.solveDc(x, 200, 1e-6, false, 30, nullptr, true);
        double gmin = c.getFinalGmin();
        
        auto diag = Api2520HomotopyV2::getDiagnostics(x, nodes, gmin);
        
        if (verbose) {
            cout << "a=" << fixed << setprecision(3) << alphaNext 
                 << " clamp=" << scientific << setprecision(1) << clampG << " ";
            cout << fixed;
            diag.print();
        }
        
        bool acceptable = Api2520HomotopyV2::isAcceptable(diag) || 
                         (okDC && !diag.isLatched && gmin <= 1e-10);
        
        if (!okDC || !acceptable) {
            // REJECT: backtrack and reduce step
            alphaStep = max(alphaStep * 0.5, ALPHA_STEP_MIN);
            x = xGood;
            
            if (verbose) {
                cout << "  -> REJECTED, step=" << alphaStep << endl;
            }
            
            if (alphaStep <= ALPHA_STEP_MIN) {
                // Try one more time with PT only
                Api2520HomotopyV2::updateSources(src, 1.0, alpha + ALPHA_STEP_MIN, clampG);
                x = xGood;
                okPT = c.solveDcPseudoTransient(x, 2e-3, 1e-6, true);  // Longer PT
                gmin = c.getFinalGmin();
                diag = Api2520HomotopyV2::getDiagnostics(x, nodes, gmin);
                
                if (okPT && !diag.isLatched) {
                    alpha = alpha + ALPHA_STEP_MIN;
                    xGood = x;
                    alphaGood = alpha;
                    alphaStep = ALPHA_STEP_MIN * 2;
                } else {
                    if (verbose) cout << "  -> Stuck at alpha=" << alphaGood << endl;
                    break;
                }
            }
            continue;
        }
        
        // ACCEPT
        alpha = alphaNext;
        xGood = x;
        alphaGood = alpha;
        alphaStep = min(alphaStep * 1.2, 0.1);
        
        // TRY DIRECT JUMP to alpha=1 periodically (every 5 steps or at checkpoints)
        if (step % 5 == 0 && alpha > 0.15) {
            vector<double> xTest = x;
            Api2520HomotopyV2::updateSources(src, 1.0, 1.0, 0.0);  // Full circuit
            bool okJump = c.solveDcPseudoTransient(xTest, 2e-3, 1e-6, true);
            okJump = okJump && c.solveDc(xTest, 200, 1e-6, false, 30, nullptr, true);
            double gminJump = c.getFinalGmin();
            auto diagJump = Api2520HomotopyV2::getDiagnostics(xTest, nodes, gminJump);
            
            if (okJump && !diagJump.isLatched && gminJump <= 1e-12) {
                if (verbose) {
                    cout << "  -> JUMP to alpha=1 SUCCEEDED! ";
                    diagJump.print();
                }
                alpha = 1.0;
                xGood = xTest;
                alphaGood = 1.0;
                break;  // Done with Phase B!
            }
            // Restore circuit state
            Api2520HomotopyV2::updateSources(src, 1.0, alpha, clampG);
        }
    }
    
    result.maxAlphaReached = alphaGood;
    
    // =========================================================================
    // PHASE C: Final verification (clamp fully OFF)
    // =========================================================================
    if (verbose) {
        cout << "\n--- PHASE C: Final (clamp OFF) ---" << endl;
    }
    
    Api2520HomotopyV2::updateSources(src, 1.0, 1.0, 0.0);
    
    // If JUMP already succeeded at Gmin<=1e-12, use that solution directly
    // (re-solving would just re-run Gmin stepping which might get stuck)
    double finalGmin = c.getFinalGmin();
    if (alphaGood >= 1.0 && finalGmin <= 1e-12) {
        // JUMP already achieved target - use xGood directly
        x = xGood;
        if (verbose) {
            cout << "Using JUMP result (Gmin already at " << finalGmin << ")" << endl;
        }
    } else {
        // Need to solve to achieve final state
        x = xGood;
        bool finalOkPT = c.solveDcPseudoTransient(x, 2e-3, 1e-6, true);
        bool finalOkDC = c.solveDc(x, 300, 1e-6, false, 50, nullptr, true);
        finalGmin = c.getFinalGmin();
        (void)finalOkPT; (void)finalOkDC;  // Used implicitly via getFinalGmin
    }
    
    auto finalDiag = Api2520HomotopyV2::getDiagnostics(x, nodes, finalGmin);
    
    if (verbose) {
        cout << "Final: ";
        finalDiag.print();
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
        cout << "  RESULTS" << endl;
        cout << "========================================" << endl;
        cout << fixed << setprecision(4);
        cout << "Max alpha: " << result.maxAlphaReached << " / 1.0" << endl;
        cout << "V(OUT): " << result.V_OUT << " V (want |OUT| < 0.5V)" << endl;
        cout << "VBE_Q2: " << result.VBE_Q2 << " V (want 0.55-0.75V)" << endl;
        cout << "Gmin: " << scientific << result.finalGmin << " S (want <=1e-12)" << endl;
    }
    
    bool out_ok = abs(result.V_OUT) < 0.5;
    bool vbe_ok = (result.VBE_Q2 > 0.55 && result.VBE_Q2 < 0.75);
    bool gmin_ok = (result.finalGmin <= 1e-12);
    bool alpha_ok = (result.maxAlphaReached > 0.99);
    
    result.success = out_ok && vbe_ok && gmin_ok && alpha_ok;
    
    if (verbose) {
        cout << "\n=== CRITERIA ===" << endl;
        cout << "|OUT|<0.5V: " << (out_ok ? "PASS" : "FAIL") << endl;
        cout << "VBE 0.55-0.75V: " << (vbe_ok ? "PASS" : "FAIL") << endl;
        cout << "Gmin<=1e-12: " << (gmin_ok ? "PASS" : "FAIL") << endl;
        cout << "alpha=1: " << (alpha_ok ? "PASS" : "FAIL") << endl;
        
        cout << "\n========================================" << endl;
        cout << (result.success ? "  *** SUCCESS ***" : "  *** FAIL ***") << endl;
        cout << "========================================" << endl;
    }
    
    return result;
}

int main() {
    HomotopyResult result = run2DHomotopyV2(true);
    return result.success ? 0 : 1;
}
