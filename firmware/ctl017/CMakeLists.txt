# SPDX-License-Identifier: MIT
# Portable CTL-017 libraries. No HAL: these build for the host and for the
# NUCLEO-G474RE alike.

add_library(ctl017_core STATIC
  core/src/debounce.c
  core/src/lift_config.c
  core/src/lift.c
  core/src/aux_io.c
)
target_include_directories(ctl017_core PUBLIC core/include)

add_library(ctl017_canopen STATIC
  canopen/src/co_node.c
)
target_include_directories(ctl017_canopen PUBLIC canopen/include)

add_library(ctl017_app STATIC
  app/src/ctl017_app.c
  app/src/ctl017_od.c
  app/src/lift_pdo.c
)
target_include_directories(ctl017_app PUBLIC app/include)
target_link_libraries(ctl017_app PUBLIC ctl017_core ctl017_canopen)
