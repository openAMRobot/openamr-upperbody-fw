# SPDX-License-Identifier: MIT
# SocketCAN simulator of CTL-017 (Linux only) and the vcan pytest.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(ctl017_sim ctl017_sim.c)
  target_link_libraries(ctl017_sim PRIVATE ctl017_app ctl017_plant)

  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_FOUND)
    # Skips itself when no vcan interface is up (see tools/ctl017_sim/setup_vcan.sh).
    add_test(NAME vcan_pytest
      COMMAND ${CMAKE_COMMAND} -E env CTL017_SIM=$<TARGET_FILE:ctl017_sim>
              ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${PROJECT_SOURCE_DIR}/tests/ctl017_vcan)
  endif()
endif()
