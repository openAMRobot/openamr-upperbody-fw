# SPDX-License-Identifier: MIT
# SocketCAN harness of the CTL-017 application layer (Linux only) and the
# vcan pytest. No lift core or plant model: the status is fixed.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(ctl017_sim ctl017_sim.c)
  target_compile_definitions(ctl017_sim PRIVATE _DEFAULT_SOURCE)
  target_link_libraries(ctl017_sim PRIVATE ctl017_app)

  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_FOUND AND NOT CTL017_PERMIT_OD)
    # Reported as SKIPPED (exit 77) when no vcan interface, python-can or
    # pytest is available (see tools/ctl017_sim/setup_vcan.sh).
    add_test(NAME vcan_pytest
      COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/ctl017_vcan/run_vcan_tests.py)
    set_tests_properties(vcan_pytest PROPERTIES
      SKIP_RETURN_CODE 77
      ENVIRONMENT "CTL017_SIM=$<TARGET_FILE:ctl017_sim>")
  endif()
endif()
