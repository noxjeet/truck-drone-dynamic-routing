# M9 regression guard (run by CTest as m9_protocol_reproduces; also usable by hand):
#   cmake -DBIN=<tdr_static> -DSRC=<repo root> -DOUT=<scratch dir> -P scripts/m9_check.cmake
# 1. every M9 protocol configuration must pass `tdr_static check`;
# 2. re-running the recorded 10-customer experiment (config/m9/n10_seed1.cfg, 4 datasets x 10 runs) must reproduce the committed-to-disk
#    results/m9/n10_seed1/{results.json,runs.csv,config.resolved.cfg} byte for byte (the deterministic outputs; timing.json is excluded).
foreach(n 10 20 50)
  foreach(s 1 2 3)
    execute_process(COMMAND "${BIN}" check --config "${SRC}/config/m9/n${n}_seed${s}.cfg" RESULT_VARIABLE rc OUTPUT_QUIET ERROR_VARIABLE err)
    if(NOT rc EQUAL 0)
      message(FATAL_ERROR "config n${n}_seed${s}.cfg is invalid (exit ${rc}): ${err}")
    endif()
  endforeach()
endforeach()
file(REMOVE_RECURSE "${OUT}")
execute_process(COMMAND "${BIN}" run --config "${SRC}/config/m9/n10_seed1.cfg" --out "${OUT}" --quiet RESULT_VARIABLE rc ERROR_VARIABLE err)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "tdr_static run failed (exit ${rc}): ${err}")
endif()
foreach(f results.json runs.csv config.resolved.cfg)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUT}/${f}" "${SRC}/results/m9/n10_seed1/${f}" RESULT_VARIABLE diff)
  if(NOT diff EQUAL 0)
    message(FATAL_ERROR "${f} differs from the recorded M9 result")
  endif()
endforeach()
message(STATUS "M9 protocol configurations valid; n10_seed1 reproduced byte-identically")
