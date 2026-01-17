cmake_minimum_required(VERSION 3.16)
project(MNASimulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable warnings
if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create interface library (header-only)
# Create interface library (header-only)
add_library(mna_lib INTERFACE)
target_include_directories(mna_lib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Main executable
add_executable(mna_simulator main.cpp)
target_link_libraries(mna_simulator PRIVATE mna_lib)

# Enable testing
enable_testing()

# Test executables
# Update paths to point to 'tests/' directory
add_executable(mna_tests tests/test_circuit.cpp)
target_link_libraries(mna_tests PRIVATE mna_lib)

add_executable(mna_tests_golden tests/test_golden.cpp)
target_link_libraries(mna_tests_golden PRIVATE mna_lib)

add_executable(mna_tests_2520 tests/test_2520.cpp)
target_link_libraries(mna_tests_2520 PRIVATE mna_lib)

add_executable(mna_tests_pnp_diode tests/test_pnp_diode.cpp)
target_link_libraries(mna_tests_pnp_diode PRIVATE mna_lib)

add_executable(mna_tests_bjt_extended tests/test_bjt_extended.cpp)
target_link_libraries(mna_tests_bjt_extended PRIVATE mna_lib)

add_executable(sweep_supplies tests/sweep_supplies.cpp)
target_link_libraries(sweep_supplies PRIVATE mna_lib)

add_executable(benchmark tests/benchmark.cpp)
target_link_libraries(benchmark PRIVATE mna_lib)

add_executable(test_blocks tests/test_blocks.cpp)
target_link_libraries(test_blocks PRIVATE mna_lib)

add_executable(test_conventions tests/test_conventions.cpp)
target_link_libraries(test_conventions PRIVATE mna_lib)

add_executable(dc_connectivity_audit tests/dc_connectivity_audit.cpp)
target_link_libraries(dc_connectivity_audit PRIVATE mna_lib)

add_executable(audit_vas_block tests/audit_vas_block.cpp)
target_link_libraries(audit_vas_block PRIVATE mna_lib)

add_executable(mna_constraint_audit tests/mna_constraint_audit.cpp)
target_link_libraries(mna_constraint_audit PRIVATE mna_lib)

add_executable(test_2520_hierarchical tests/test_2520_hierarchical.cpp)
target_link_libraries(test_2520_hierarchical PRIVATE mna_lib)

add_executable(diagnose_vas_54v tests/diagnose_vas_54v.cpp)
target_link_libraries(diagnose_vas_54v PRIVATE mna_lib)

add_executable(test_vas_with_bias tests/test_vas_with_bias.cpp)
target_link_libraries(test_vas_with_bias PRIVATE mna_lib)

add_executable(kcl_audit_vas tests/kcl_audit_vas.cpp)
target_link_libraries(kcl_audit_vas PRIVATE mna_lib)

add_executable(diagnose_inm_6v tests/diagnose_inm_6v.cpp)
target_link_libraries(diagnose_inm_6v PRIVATE mna_lib)

add_executable(test_components_sanity tests/test_components_sanity.cpp)
target_link_libraries(test_components_sanity PRIVATE mna_lib)

add_executable(debug_feedback_bias tests/debug_feedback_bias.cpp)
target_link_libraries(debug_feedback_bias PRIVATE mna_lib)

add_executable(test_2520_stabilized tests/test_2520_stabilized.cpp)
target_link_libraries(test_2520_stabilized PRIVATE mna_lib)

add_executable(test_2520_patron_fix tests/test_2520_patron_fix.cpp)
target_link_libraries(test_2520_patron_fix PRIVATE mna_lib)

add_executable(test_parasitic_scale tests/test_parasitic_scale.cpp)
target_link_libraries(test_parasitic_scale PRIVATE mna_lib)

add_executable(test_pnp_unit tests/test_pnp_unit.cpp)
target_link_libraries(test_pnp_unit PRIVATE mna_lib)

add_executable(kcl_audit_nC2 tests/kcl_audit_nC2.cpp)
target_link_libraries(kcl_audit_nC2 PRIVATE mna_lib)

add_executable(test_vas_bias_chain tests/test_vas_bias_chain.cpp)
target_link_libraries(test_vas_bias_chain PRIVATE mna_lib)

add_executable(test_2520_continuation tests/test_2520_continuation.cpp)
target_link_libraries(test_2520_continuation PRIVATE mna_lib)

add_executable(test_2520_homotopy2d tests/test_2520_homotopy2d.cpp)
target_link_libraries(test_2520_homotopy2d PRIVATE mna_lib)

add_executable(test_2520_homotopy_v2 tests/test_2520_homotopy_v2.cpp)
target_link_libraries(test_2520_homotopy_v2 PRIVATE mna_lib)

add_executable(diagnose_polarity diagnostics/diagnose_polarity.cpp)
target_link_libraries(diagnose_polarity PRIVATE mna_lib)

add_executable(dump_nodes diagnostics/dump_nodes.cpp)
target_link_libraries(dump_nodes PRIVATE mna_lib)

# Add tests
add_test(NAME circuit_tests COMMAND mna_tests)
add_test(NAME golden_tests COMMAND mna_tests_golden)
add_test(NAME pnp_diode_tests COMMAND mna_tests_pnp_diode)
add_test(NAME bjt_extended_tests COMMAND mna_tests_bjt_extended)
add_test(NAME 2520_tests COMMAND mna_tests_2520)
add_test(NAME supply_sweep COMMAND sweep_supplies)
add_test(NAME performance_benchmark COMMAND benchmark)
add_test(NAME block_tests COMMAND test_blocks)
add_test(NAME convention_tests COMMAND test_conventions)

