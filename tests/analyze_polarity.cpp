#include "circuit.h"
#include "current_source.h"
#include "resistor.h"
#include "voltage_source.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// What the 2520 builder INTENDS vs what ACTUALLY happens:

int main() {
    cout << "=== ANALYSIS: 2520 Current Source Polarities ===" << endl;
    cout << "\n** Actual convention discovered: CurrentSource(a,b,I) means I flows FROM b TO a **\n" << endl;
    
    cout << "1. I_Tail(nTAIL, nVEE, 200uA)" << endl;
    cout << "   INTENT: Sink 200uA from diff pair tail to VEE (correct for tail current)" << endl;
    cout << "   ACTUAL: Current flows FROM nVEE TO nTAIL = current INJECTED into tail" << endl;
    cout << "   VERDICT: WRONG! This sources current into tail instead of sinking." << endl;
    cout << "   FIX: Should be CurrentSource(nVEE, nTAIL, 200uA)" << endl;
    
    cout << "\n2. I_VAS(nVCC, nVAS, 1.5mA)" << endl;
    cout << "   INTENT: Source 1.5mA from VCC into VAS collector (active load)" << endl;
    cout << "   ACTUAL: Current flows FROM nVAS TO nVCC = current EXTRACTED from VAS" << endl;
    cout << "   VERDICT: WRONG! This sinks current from VAS instead of sourcing." << endl;
    cout << "   FIX: Should be CurrentSource(nVAS, nVCC, 1.5mA)" << endl;
    
    cout << "\n3. I_BiasChain_Sink(nB_LO, nVEE, 0.5mA)" << endl;
    cout << "   INTENT: Sink 0.5mA from bias chain low point to VEE" << endl;
    cout << "   ACTUAL: Current flows FROM nVEE TO nB_LO = current INJECTED into B_LO" << endl;
    cout << "   VERDICT: WRONG! This sources current into bias chain." << endl;
    cout << "   FIX: Should be CurrentSource(nVEE, nB_LO, 0.5mA)" << endl;
    
    cout << "\n=== SUMMARY ===" << endl;
    cout << "ALL THREE current sources in the 2520 have INVERTED polarities." << endl;
    cout << "This explains the bizarre DC operating point!" << endl;
    
    return 0;
}
