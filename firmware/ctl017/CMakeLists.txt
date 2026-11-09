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
