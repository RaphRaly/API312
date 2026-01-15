#include "api_2520_builder.h"
#include "circuit.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

ofstream out;

double getV(const Circuit &c, const vector<double> &x, const string &name) {
    for (const auto &pair : c.getNodeNames()) {
        if (pair.second == name) return x[pair.first];
    }
    return 999.999;
}

int main() {
    out.open("topology_analysis.txt");
    
    Circuit c;
    Api2520Builder::build2520(c, 15.0);
    c.finalize();

    vector<double> x;
    if (!c.solveDc(x, 400, 1e-6, false, 50)) {
        out << "DC solve failed!" << endl;
        return 1;
    }
    
    out << fixed << setprecision(3);
    out << "=== TOPOLOGY ANALYSIS - Finding why output saturates ===" << endl;
    
    out << "\n--- POWER RAILS ---" << endl;
    out << "VCC = " << getV(c,x,"VCC") << "V (expected +15V)" << endl;
    out << "VEE (should be at node) = -15V (by construction)" << endl;
    
    out << "\n--- INPUT STAGE ---" << endl;
    out << "INP (Q1 base external) = " << getV(c,x,"INP") << "V" << endl;
    out << "INM (Q2 base external) = " << getV(c,x,"INM") << "V" << endl;
    out << "INM_Ref (VinM source) = " << getV(c,x,"INM_Ref") << "V" << endl;
    out << "Diff_Tail = " << getV(c,x,"Diff_Tail") << "V" << endl;
    out << "Q1_Emit = " << getV(c,x,"Q1_Emit") << "V" << endl;
    out << "Q2_Emit = " << getV(c,x,"Q2_Emit") << "V" << endl;
    out << "Q1_Col = " << getV(c,x,"Q1_Col") << "V" << endl;
    out << "Q2_Col = " << getV(c,x,"Q2_Col") << "V" << endl;
    
    out << "\n--- CURRENT MIRROR (Q3, Q9) ---" << endl;
    out << "Q3 (PNP) connected: C=Q1_Col, B=Q1_Col (diode), E=Q3_Emit" << endl;
    out << "Q9 (PNP) connected: C=Q2_Col, B=Q1_Col (mirror), E=Q9_Emit" << endl;
    out << "Q3_Emit = " << getV(c,x,"Q3_Emit") << "V" << endl;
    out << "Q9_Emit = " << getV(c,x,"Q9_Emit") << "V" << endl;
    
    out << "\n--- VAS STAGE (Q4, Q5) ---" << endl;
    out << "Q4 (PNP) connected: C=Q4_Col, B=Q2_Col, E=Q4_Emit" << endl;
    out << "Q4_Col = " << getV(c,x,"Q4_Col") << "V (goes to Q5 base)" << endl;
    out << "Q4_Emit = " << getV(c,x,"Q4_Emit") << "V" << endl;
    out << "Q5 (NPN) connected: C=VAS_Col, B=Q4_Col, E=Q5_Emit" << endl;
    out << "VAS_Col = " << getV(c,x,"VAS_Col") << "V" << endl;
    out << "Q5_Emit = " << getV(c,x,"Q5_Emit") << "V" << endl;
    
    out << "\n--- BIAS MULTIPLIER ---" << endl;
    out << "VAS_High = " << getV(c,x,"VAS_High") << "V (Q7 base)" << endl;
    out << "D_Bias_mid = " << getV(c,x,"D_Bias_mid") << "V" << endl;
    out << "VAS_Low = " << getV(c,x,"VAS_Low") << "V (Q8 base)" << endl;
    
    out << "\n--- OUTPUT STAGE (Q7, Q8) ---" << endl;
    out << "Q7 (NPN) connected: C=VCC, B=VAS_High, E=Q7_Emit" << endl;
    out << "Q7_Emit = " << getV(c,x,"Q7_Emit") << "V" << endl;
    out << "Q8 (PNP) connected: C=VEE, B=VAS_Low, E=Q8_Emit" << endl;
    out << "Q8_Emit = " << getV(c,x,"Q8_Emit") << "V" << endl;
    out << "OUT = " << getV(c,x,"OUT") << "V" << endl;
    
    out << "\n--- FEEDBACK ANALYSIS ---" << endl;
    double vOut = getV(c,x,"OUT");
    double vInm = getV(c,x,"INM");
    double vInmRef = getV(c,x,"INM_Ref");
    out << "Expected INM (with feedback): VinM + (OUT-VinM)*Rg/(Rf+Rg) = " 
        << vInmRef + (vOut-vInmRef)*0.5 << "V" << endl;
    out << "Actual INM = " << vInm << "V" << endl;
    out << "Match: " << (abs(vInm - (vInmRef + (vOut-vInmRef)*0.5)) < 0.5 ? "YES" : "NO") << endl;
    
    out << "\n=== SIGNAL PATH GAIN ANALYSIS ===" << endl;
    out << "\nFor a balanced input (INP=INM=0V), we expect OUT≈0V." << endl;
    out << "But we see OUT = " << vOut << "V." << endl;
    out << "\nThis means the open-loop amp has an INTRINSIC offset pushing output negative." << endl;
    out << "\nLet's check Q1 vs Q2 balance:" << endl;
    double vQ1b = getV(c,x,"Q1_Bi"), vQ1e = getV(c,x,"Q1_Ei");
    double vQ2b = getV(c,x,"Q2_Bi"), vQ2e = getV(c,x,"Q2_Ei");
    out << "Q1: Vbe = " << (vQ1b - vQ1e) << "V (should be ~0.6V)" << endl;
    out << "Q2: Vbe = " << (vQ2b - vQ2e) << "V (should be ~0.6V)" << endl;
    
    out << "\n=== KEY DIAGNOSTIC ===" << endl;
    out << "Q2 has Vbe = " << (vQ2b - vQ2e) << "V which is " 
        << ((vQ2b - vQ2e) < 0.4 ? "TOO LOW - Q2 is cutoff!" : "OK") << endl;
    out << "This causes C2 to be pulled to rail by mirror (Q9)." << endl;
    
    // Check if there's a topology issue with the VAS
    out << "\n=== VAS TOPOLOGY CHECK ===" << endl;
    double vQ4b = getV(c,x,"Q4_Bi"), vQ4e = getV(c,x,"Q4_Ei");
    double vQ4c = getV(c,x,"Q4_Ci");
    out << "Q4 (PNP): B=" << vQ4b << ", E=" << vQ4e << ", C=" << vQ4c << endl;
    out << "Veb (should be ~0.6V for ON) = " << (vQ4e - vQ4b) << "V" << endl;
    out << "Q4 status: " << ((vQ4e - vQ4b) > 0.4 ? "ON" : "OFF or WEAK") << endl;
    
    out.close();
    cout << "Analysis written to topology_analysis.txt" << endl;
    return 0;
}
