cmake_minimum_required(VERSION 3.16)
project(truck_drone_dynamic_routing CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- Stage-1 core library ---------------------------------------------
# Only components that are actually implemented so far are listed here;
# see README.md / DECISIONS.md for what remains.
add_library(tdr_core
    src/tsp/held_karp_solver.cpp
    src/tsp/heuristic_tsp_solver.cpp
    src/tsp/tsp_solver_factory.cpp
)
target_include_directories(tdr_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

# --- Tests ---------------------------------------------------------------
enable_testing()

add_executable(test_tsp tests/test_tsp.cpp)
target_link_libraries(test_tsp PRIVATE tdr_core)
add_test(NAME test_tsp COMMAND test_tsp)

# model/ (Phase B) is header-only and independent of tsp/, so its test
# only needs the include path, not tdr_core.
add_executable(test_model tests/test_model.cpp)
target_include_directories(test_model PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_model COMMAND test_model)

# graph/ (Phase C) is header-only and builds on model/ only (not tsp/),
# so its test also just needs the include path. It additionally needs
# tests/ on the include path for the shared small_graph_fixture.hpp.
add_executable(test_graph tests/test_graph.cpp)
target_include_directories(test_graph PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_graph COMMAND test_graph)

# flp/ (Phase D) is header-only and builds on model/ only (not tsp/ or
# graph/), so its test also just needs the include path plus tests/ for
# the shared small_graph_fixture.hpp.
add_executable(test_flp tests/test_flp.cpp)
target_include_directories(test_flp PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_flp COMMAND test_flp)

# timing/ (Phase E) is header-only and builds on model/ and graph/ (not
# tsp/ or flp/); its test constructs its own small fixtures inline rather
# than using small_graph_fixture.hpp, so it only needs the include path.
add_executable(test_timing tests/test_timing.cpp)
target_include_directories(test_timing PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_timing COMMAND test_timing)

# init/ (Phase F) builds on model/, graph/, flp/, timing/, AND tsp/ (it
# calls TSPSolverFactory for the truck-only route), so its test links
# tdr_core the same way test_tsp does.
add_executable(test_init tests/test_init.cpp)
target_link_libraries(test_init PRIVATE tdr_core)
add_test(NAME test_init COMMAND test_init)

# search infrastructure (M1, 9 Nov 2026): RNG, config, instance generator,
# FLP arc index, shared drone-id assignment, route operations, candidate
# window. Header-only except that the tests build real initial solutions via
# init/, hence tdr_core is linked (same as test_init).
add_executable(test_search_infra tests/test_search_infra.cpp)
target_link_libraries(test_search_infra PRIVATE tdr_core)
add_test(NAME test_search_infra COMMAND test_search_infra)

add_executable(test_fuzz_search_infra tests/test_fuzz_search_infra.cpp)
target_link_libraries(test_fuzz_search_infra PRIVATE tdr_core)
add_test(NAME test_fuzz_search_infra COMMAND test_fuzz_search_infra)

# M1 performance follow-up (P1/P2): production evaluator vs the frozen reference
# copy of the original evaluator (tests/reference/). Default mode checks the
# accepted P1+P2 contract; the "_strict" variant demands bitwise identity.
add_executable(test_evaluator_differential tests/test_evaluator_differential.cpp)
target_link_libraries(test_evaluator_differential PRIVATE tdr_core)
target_include_directories(test_evaluator_differential PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_evaluator_differential COMMAND test_evaluator_differential)

# Evaluator micro-benchmark (M1 performance follow-up). Not built by default and
# not part of CTest.  Optional 4th variant: pass -DTDR_BENCH_P1_HEADER=<path> with a
# saved copy of the P1-only evaluator (namespace tdr::timing_p1).
option(TDR_BUILD_BENCH "Build bench/ micro-benchmarks" OFF)
if(TDR_BUILD_BENCH)
    add_executable(bench_evaluator bench/bench_evaluator.cpp)
    target_link_libraries(bench_evaluator PRIVATE tdr_core)
    target_include_directories(bench_evaluator PRIVATE ${CMAKE_SOURCE_DIR}/tests)
    add_executable(bench_operators bench/bench_operators.cpp)
    target_link_libraries(bench_operators PRIVATE tdr_core)
    add_executable(bench_n7_n8 bench/bench_n7_n8.cpp)
    target_link_libraries(bench_n7_n8 PRIVATE tdr_core)
    add_executable(bench_n3_n6 bench/bench_n3_n6.cpp)
    target_link_libraries(bench_n3_n6 PRIVATE tdr_core)
    add_executable(bench_n1_n2 bench/bench_n1_n2.cpp)
    target_link_libraries(bench_n1_n2 PRIVATE tdr_core)
    add_executable(m9_feasibility_probe bench/m9_feasibility_probe.cpp)
    target_link_libraries(m9_feasibility_probe PRIVATE tdr_core)
    add_executable(bench_shakes bench/bench_shakes.cpp)
    target_link_libraries(bench_shakes PRIVATE tdr_core)
    add_executable(bench_vns bench/bench_vns.cpp)
    target_link_libraries(bench_vns PRIVATE tdr_core)
    set(TDR_BENCH_P1_HEADER "" CACHE FILEPATH "Saved P1-only evaluator header for the bench's variant C")
    if(TDR_BENCH_P1_HEADER)
        target_compile_definitions(bench_evaluator PRIVATE TDR_BENCH_P1_HEADER="${TDR_BENCH_P1_HEADER}")
    endif()
endif()

# M2: neighbourhoods N4 (truck service node relocate) and N5 (T-D service node swap).
add_executable(test_n4_n5 tests/test_n4_n5.cpp)
target_link_libraries(test_n4_n5 PRIVATE tdr_core)
target_include_directories(test_n4_n5 PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n4_n5 COMMAND test_n4_n5)

add_executable(test_n4_n5_oracle tests/test_n4_n5_oracle.cpp)
target_link_libraries(test_n4_n5_oracle PRIVATE tdr_core)
target_include_directories(test_n4_n5_oracle PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n4_n5_oracle COMMAND test_n4_n5_oracle)

# M3: neighbourhoods N7 (truck node delete / drone node insert) and N8 (drone node delete / truck node insert).
add_executable(test_n7_n8 tests/test_n7_n8.cpp)
target_link_libraries(test_n7_n8 PRIVATE tdr_core)
target_include_directories(test_n7_n8 PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n7_n8 COMMAND test_n7_n8)

add_executable(test_n7_n8_oracle tests/test_n7_n8_oracle.cpp)
target_link_libraries(test_n7_n8_oracle PRIVATE tdr_core)
target_include_directories(test_n7_n8_oracle PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n7_n8_oracle COMMAND test_n7_n8_oracle)

# M4: neighbourhoods N3 (drone service node relocate) and N6 (D-D service node swap).
add_executable(test_n3_n6 tests/test_n3_n6.cpp)
target_link_libraries(test_n3_n6 PRIVATE tdr_core)
target_include_directories(test_n3_n6 PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n3_n6 COMMAND test_n3_n6)

add_executable(test_n3_n6_oracle tests/test_n3_n6_oracle.cpp)
target_link_libraries(test_n3_n6_oracle PRIVATE tdr_core)
target_include_directories(test_n3_n6_oracle PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n3_n6_oracle COMMAND test_n3_n6_oracle)

add_executable(test_n1_n2 tests/test_n1_n2.cpp)
target_link_libraries(test_n1_n2 PRIVATE tdr_core)
target_include_directories(test_n1_n2 PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n1_n2 COMMAND test_n1_n2)

add_executable(test_n1_n2_oracle tests/test_n1_n2_oracle.cpp)
target_link_libraries(test_n1_n2_oracle PRIVATE tdr_core)
target_include_directories(test_n1_n2_oracle PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_n1_n2_oracle COMMAND test_n1_n2_oracle)

add_executable(test_shakes tests/test_shakes.cpp)
target_link_libraries(test_shakes PRIVATE tdr_core)
target_include_directories(test_shakes PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_shakes COMMAND test_shakes)

add_executable(test_shakes_oracle tests/test_shakes_oracle.cpp)
target_link_libraries(test_shakes_oracle PRIVATE tdr_core)
target_include_directories(test_shakes_oracle PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_shakes_oracle COMMAND test_shakes_oracle)

add_executable(test_vns tests/test_vns.cpp)
target_link_libraries(test_vns PRIVATE tdr_core)
target_include_directories(test_vns PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_vns COMMAND test_vns)

add_executable(test_vns_oracle tests/test_vns_oracle.cpp)
target_link_libraries(test_vns_oracle PRIVATE tdr_core)
target_include_directories(test_vns_oracle PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_vns_oracle COMMAND test_vns_oracle)

# --- M8: experiment runner / result writer / configuration / CLI (9 Nov 2026) ---
add_executable(tdr_static apps/tdr_static.cpp)
target_link_libraries(tdr_static PRIVATE tdr_core)

add_executable(test_experiment_config tests/test_experiment_config.cpp)
target_link_libraries(test_experiment_config PRIVATE tdr_core)
add_test(NAME test_experiment_config COMMAND test_experiment_config)

add_executable(test_runner_cli tests/test_runner_cli.cpp)
target_link_libraries(test_runner_cli PRIVATE tdr_core)
target_include_directories(test_runner_cli PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_runner_cli COMMAND test_runner_cli)

# M9: the recorded validation protocol still reproduces (scripts/m9_check.cmake)
add_test(NAME m9_protocol_reproduces COMMAND ${CMAKE_COMMAND} -DBIN=$<TARGET_FILE:tdr_static> -DSRC=${CMAKE_SOURCE_DIR} -DOUT=${CMAKE_BINARY_DIR}/m9_check_out -P ${CMAKE_SOURCE_DIR}/scripts/m9_check.cmake)
