#include "circuit.h"
#include "resistor.h"
#include "voltage_source.h"
#include "current_source.h"
#include "capacitor_trap.h"
#include "diode_shockley_nr.h"
#include <cassert>
#include <iostream>
#include <cmath>

// Test utilities
#define TEST_ASSERT(cond, msg) \
    if (!(cond)) { \
        std::cerr << "TEST FAILED: " << msg << std::endl; \
        return false; \
    }

bool test_resistor_voltage_divider() {
    Circuit c;

    NodeIndex n1 = c.createNode();
    NodeIndex n2 = c.createNode();

    c.addElement<VoltageSource>(n1, GND, 10.0);
    c.addElement<Resistor>(n1, n2, 1000.0);
    c.addElement<Resistor>(n2, GND, 1000.0);

    c.finalize();
    std::vector<double> x;
    bool ok = c.step(1.0, x, 10, 1e-9, 1.0);

    TEST_ASSERT(ok, "Convergence failed");
    TEST_ASSERT(std::abs(x[n2] - 5.0) < 1e-6, "Wrong voltage divider result");

    return true;
}

bool test_capacitor_dc() {
    Circuit c;

    NodeIndex n1 = c.createNode();
    c.addElement<VoltageSource>(n1, GND, 5.0);
    c.addElement<CapacitorTrap>(n1, GND, 1e-6); // 1uF

    c.finalize();
    std::vector<double> x;
    bool ok = c.step(1e-3, x, 10, 1e-9, 1.0); // 1ms step

    TEST_ASSERT(ok, "DC convergence failed");
    TEST_ASSERT(std::abs(x[n1] - 5.0) < 1e-6, "Capacitor should charge to source voltage in DC");

    return true;
}

bool test_diode_forward_bias() {
    Circuit c;

    NodeIndex n1 = c.createNode();
    NodeIndex n2 = c.createNode();

    c.addElement<VoltageSource>(n1, GND, 1.0); // Forward bias
    c.addElement<Resistor>(n1, n2, 1000.0);
    c.addElement<DiodeShockleyNR>(n2, GND, 1e-15, 1.0, 0.02585);

    c.finalize();
    std::vector<double> x;
    bool ok = c.step(1.0, x, 20, 1e-9, 1.0, true);

    TEST_ASSERT(ok, "Diode convergence failed");
    TEST_ASSERT(x[n2] > 0.6 && x[n2] < 0.8, "Diode voltage should be around 0.7V");

    return true;
}

int main() {
    std::cout << "Running MNA Circuit Tests..." << std::endl;

    bool all_passed = true;

    if (!test_resistor_voltage_divider()) all_passed = false;
    if (!test_capacitor_dc()) all_passed = false;
    if (!test_diode_forward_bias()) all_passed = false;

    if (all_passed) {
        std::cout << "All tests PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "Some tests FAILED!" << std::endl;
        return 1;
    }
}
