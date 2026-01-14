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
add_library(mna_lib INTERFACE)
target_include_directories(mna_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
# Explicitly list headers for IDE visibility if needed, but not as part of add_library source list for INTERFACE

# Main executable
add_executable(mna_simulator main.cpp)
target_link_libraries(mna_simulator PRIVATE mna_lib)

# Enable testing
enable_testing()

# Test executables
add_executable(mna_tests test_circuit.cpp)
target_link_libraries(mna_tests PRIVATE mna_lib)

add_executable(mna_tests_golden test_golden.cpp)
target_link_libraries(mna_tests_golden PRIVATE mna_lib)

add_executable(mna_tests_2520 test_2520.cpp)
target_link_libraries(mna_tests_2520 PRIVATE mna_lib)

add_executable(mna_tests_pnp_diode test_pnp_diode.cpp)
target_link_libraries(mna_tests_pnp_diode PRIVATE mna_lib)

add_executable(mna_tests_bjt_extended test_bjt_extended.cpp)
target_link_libraries(mna_tests_bjt_extended PRIVATE mna_lib)

add_executable(sweep_supplies sweep_supplies.cpp)
target_link_libraries(sweep_supplies PRIVATE mna_lib)

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE mna_lib)

# Add tests
add_test(NAME circuit_tests COMMAND mna_tests)
add_test(NAME golden_tests COMMAND mna_tests_golden)
add_test(NAME pnp_diode_tests COMMAND mna_tests_pnp_diode)
add_test(NAME bjt_extended_tests COMMAND mna_tests_bjt_extended)
add_test(NAME 2520_tests COMMAND mna_tests_2520)
add_test(NAME supply_sweep COMMAND sweep_supplies)
add_test(NAME performance_benchmark COMMAND benchmark)
