#pragma once

#include "circuit.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct BjtPlausibilityResult {
  std::string name;
  double Vc, Vb, Ve;
  double Vje, Vjc; // Vje is Vbe for NPN, Veb for PNP
  bool isPnp;
  bool passVje;
  bool passSaturation;
};

struct PlausibilityReport {
  bool overallPass = true;
  std::vector<std::string> railViolations;
  std::vector<BjtPlausibilityResult> bjts;
  double iVcc, iVee;
  double vOut;
};

class PlausibilityChecker {
public:
  static double getNodeVoltage(const Circuit &c, const std::vector<double> &x,
                               const std::string &name) {
    for (const auto &pair : c.getNodeNames()) {
      if (pair.second == name)
        return x[pair.first];
    }
    return 0.0;
  }

  static PlausibilityReport
  check(const Circuit &c, const std::vector<double> &x, double supplyV) {
    PlausibilityReport report;
    double vcc = std::abs(supplyV);
    double vee = -vcc;

    // 1. Rail Checks
    for (const auto &pair : c.getNodeNames()) {
      double v = x[pair.first];
      if (v > vcc + 0.201 || v < vee - 0.201) {
        report.railViolations.push_back(pair.second + ": " + std::to_string(v) +
                                        "V");
        report.overallPass = false;
      }
    }

    // 2. BJT Checks - We look for pattern-based node names from builder
    // This is a bit manual currently, but we can iterate common patterns
    std::vector<std::pair<std::string, bool>> bjtsToCheck = {
        {"Q1", false}, {"Q2", false}, {"Q3", true},
        {"Q4", true},  {"Q5", false}, {"Q7", false},
        {"Q8", true},  {"Q9", true},  {"Q_Bias", false}};

    for (const auto &[name, isPnp] : bjtsToCheck) {
      BjtPlausibilityResult res;
      res.name = name;
      res.isPnp = isPnp;

      // Reconstruct internal nodes if extended, otherwise use external
      // For now, use the builder node naming convention
      std::string c_name, b_name, e_name;
      if (name == "Q1" || name == "Q2") {
        c_name = name + "_Col_soft";
        b_name = (name == "Q1") ? "INP" : "INM";
        e_name = name + "_Emit";
      } else if (name == "Q7" || name == "Q8") {
        c_name = isPnp ? "VEE" : "VCC";
        b_name = isPnp ? "VAS_Low" : "VAS_High";
        e_name = name + "_Emit";
      } else if (name == "Q3" || name == "Q9") {
        c_name = (name == "Q3") ? "Q1_Col" : "Q2_Col";
        b_name = "Q1_Col"; // Diode master or mirror base
        e_name = name + "_Emit";
      } else if (name == "Q4") {
        c_name = "Q4_Col";
        b_name = "Q2_Col";
        e_name = "Q4_Emit";
      } else if (name == "Q5") {
        c_name = "VAS_Col";
        b_name = "Q4_Col";
        e_name = "Q5_Emit";
      } else if (name == "Q_Bias") {
        c_name = "VAS_High";
        b_name = "D_Bias_mid";
        e_name = "VAS_Low";
      }

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
    std::cout << "\n--- PHYSICAL PLAUSIBILITY REPORT ---\n";
    std::cout << "STATUS: " << (report.overallPass ? "[PASS]" : "[FAIL]")
              << "\n";

    if (!report.railViolations.empty()) {
      std::cout << "RAIL VIOLATIONS:\n";
      for (const auto &v : report.railViolations)
        std::cout << "  - " << v << "\n";
    }

    std::cout << std::left << std::setw(10) << "BJT" << " | " << std::setw(8)
              << "Vje" << " | " << std::setw(8) << "Vjc" << " | "
              << "Status\n";
    std::cout << "-------------------------------------------\n";
    for (const auto &b : report.bjts) {
      std::string status = b.passVje ? "OK" : "VJE_BAD";
      if (b.passSaturation == false)
        status += "/SAT";
      std::cout << std::left << std::setw(10) << b.name << " | " << std::fixed
                << std::setprecision(3) << std::setw(8) << b.Vje << " | "
                << std::setw(8) << b.Vjc << " | " << status << "\n";
    }
    std::cout << "OUT Offset: " << report.vOut << " V\n";
    std::cout << "-------------------------------------------\n";
  }
};
