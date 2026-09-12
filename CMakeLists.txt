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
