# SPDX-License-Identifier: MIT
# Host unit tests for the CTL-017 CANopen and application layers (ctest).
# The lift is replaced by the test double in lift_double.c (never moves).

add_library(ctl017_lift_double STATIC lift_double.c)
target_include_directories(ctl017_lift_double PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ctl017_lift_double PUBLIC ctl017_iface)

function(ctl017_test name)
  add_executable(${name} ${name}.c)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${name} PRIVATE ctl017_app ctl017_lift_double)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

ctl017_test(test_canopen)
ctl017_test(test_co_robust)
if(CTL017_PERMIT_OD)
  ctl017_test(test_permit)
endif()
