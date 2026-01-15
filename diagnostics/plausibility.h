#pragma once

#include "circuit.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

struct BjtPlausibilityResult {
  string name;
  double Vc, Vb, Ve;
  double Vje, Vjc; // Vje is Vbe for NPN, Veb for PNP
  bool isPnp;
  bool passVje;
  bool passSaturation;
};

struct PlausibilityReport {
  bool overallPass = true;
  vector<string> railViolations;
  vector<BjtPlausibilityResult> bjts;
  double iVcc, iVee;
  double vOut;
};

class PlausibilityChecker {
public:
  static double getNodeVoltage(const Circuit &c, const vector<double> &x,
                               const string &name) {
    for (const auto &pair : c.getNodeNames()) {
      if (pair.second == name)
        return x[pair.first];
    }
    return 0.0;
  }

  static PlausibilityReport
  check(const Circuit &c, const vector<double> &x, double supplyV) {
    PlausibilityReport report;
    double vcc = abs(supplyV);
    double vee = -vcc;

    // 1. Rail Checks
    for (const auto &pair : c.getNodeNames()) {
      double v = x[pair.first];
      if (v > vcc + 0.201 || v < vee - 0.201) {
        report.railViolations.push_back(pair.second + ": " + to_string(v) +
                                        "V");
        report.overallPass = false;
      }
    }

    // 2. BJT Checks - We look for pattern-based node names from builder
    // This is a bit manual currently, but we can iterate common patterns
    vector<pair<string, bool>> bjtsToCheck = {
        {"Q1", false}, {"Q2", false}, {"Q3", true},
        {"Q4", true},  {"Q5", false}, {"Q7", false},
        {"Q8", true},  {"Q9", true},  {"Q_Bias", false}};

    for (const auto &[name, isPnp] : bjtsToCheck) {
      BjtPlausibilityResult res;
      res.name = name;
      res.isPnp = isPnp;

      // Use internal BJT nodes (_Ci, _Bi, _Ei) for junction voltage measurement
      // These are created by addBjtExtended when RB/RC/RE > 0 (all SPICE models have these)
      // External nodes include voltage drops across parasitic resistances, which would
      // give incorrect Vje/Vjc readings
      string c_name = name + "_Ci";
      string b_name = name + "_Bi";
      string e_name = name + "_Ei";

      res.Vc = getNodeVoltage(c, x, c_name);
      res.Vb = getNodeVoltage(c, x, b_name);
      res.Ve = getNodeVoltage(c, x, e_name);

      if (isPnp) {
        res.Vje = res.Ve - res.Vb;
        res.Vjc = res.Vc - res.Vb;
      } else {
        res.Vje = res.Vb - res.Ve;
        res.Vjc = res.Vb - res.Vc;
      }

      res.passVje = (res.Vje <= 1.0) && (res.Vje >= -0.2);
      res.passSaturation = (res.Vjc <= 0.5); // Warn if deep saturation

      if (!res.passVje)
        report.overallPass = false;
      report.bjts.push_back(res);
    }

    report.vOut = getNodeVoltage(c, x, "OUT");
    return report;
  }

  static void printReport(const PlausibilityReport &report) {
    cout << "\n--- PHYSICAL PLAUSIBILITY REPORT ---\n";
    cout << "STATUS: " << (report.overallPass ? "[PASS]" : "[FAIL]")
              << "\n";

    if (!report.railViolations.empty()) {
      cout << "RAIL VIOLATIONS:\n";
      for (const auto &v : report.railViolations)
        cout << "  - " << v << "\n";
    }

    cout << left << setw(10) << "BJT" << " | " << setw(8)
              << "Vje" << " | " << setw(8) << "Vjc" << " | "
              << "Status\n";
    cout << "-------------------------------------------\n";
    for (const auto &b : report.bjts) {
      string status = b.passVje ? "OK" : "VJE_BAD";
      if (b.passSaturation == false)
        status += "/SAT";
      cout << left << setw(10) << b.name << " | " << fixed
                << setprecision(3) << setw(8) << b.Vje << " | "
                << setw(8) << b.Vjc << " | " << status << "\n";
    }
    cout << "OUT Offset: " << report.vOut << " V\n";
    cout << "-------------------------------------------\n";
  }
};
