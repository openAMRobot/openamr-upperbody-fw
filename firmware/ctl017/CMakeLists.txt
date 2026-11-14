# SPDX-License-Identifier: MIT
# Portable CTL-017 layers: no HAL, no registers, no allocation. They build
# for the host and, as object libraries, for the Cortex-M4 of the G474RE.

set(CTL017_FW_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")
set(CTL017_IFACE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/iface/src/lift_iface.c CACHE INTERNAL "")
set(CTL017_CANOPEN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/canopen/src/co_node.c CACHE INTERNAL "")
set(CTL017_APP_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/app/src/ctl017_app.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/src/ctl017_od.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/src/lift_pdo.c
  CACHE INTERNAL "")

if(CMAKE_CROSSCOMPILING)
  set(_kind OBJECT)
else()
  set(_kind STATIC)
endif()

add_library(ctl017_iface ${_kind} ${CTL017_IFACE_SRC})
target_include_directories(ctl017_iface PUBLIC iface/include)

add_library(ctl017_canopen ${_kind} ${CTL017_CANOPEN_SRC})
target_include_directories(ctl017_canopen PUBLIC canopen/include)

add_library(ctl017_app ${_kind} ${CTL017_APP_SRC})
target_include_directories(ctl017_app PUBLIC app/include)
target_link_libraries(ctl017_app PUBLIC ctl017_canopen ctl017_iface)

if(CTL017_PERMIT_OD)
  if(CTL017_PERMIT_DEFS STREQUAL "")
    message(FATAL_ERROR "CTL017_PERMIT_OD=ON needs CTL017_PERMIT_DEFS (TODO(Evans), see docs/ctl017_permit_contract.md)")
  endif()
  target_compile_definitions(ctl017_app PUBLIC CTL017_PERMIT_OD ${CTL017_PERMIT_DEFS})
  target_compile_definitions(ctl017_iface PUBLIC CTL017_PERMIT_OD)
endif()
