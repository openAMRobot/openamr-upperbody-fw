# SPDX-License-Identifier: MIT
# Host unit tests for CTL-017 (ctest).

add_library(ctl017_plant STATIC ${PROJECT_SOURCE_DIR}/firmware/ctl017/sim/lift_plant.c)
target_include_directories(ctl017_plant PUBLIC ${PROJECT_SOURCE_DIR}/firmware/ctl017/sim)
target_link_libraries(ctl017_plant PUBLIC ctl017_core)

function(ctl017_test name)
  add_executable(${name} ${name}.c)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${name} PRIVATE ${ARGN})
  add_test(NAME ${name} COMMAND ${name})
endfunction()

ctl017_test(test_lift ctl017_core ctl017_plant)
ctl017_test(test_canopen ctl017_app ctl017_plant)
ctl017_test(test_drive_gate ctl017_core ctl017_plant)
# The board drive configuration is HAL-free so the skeleton gate is tested here.
target_include_directories(test_drive_gate PRIVATE ${PROJECT_SOURCE_DIR}/boards/nucleo_g474re)
