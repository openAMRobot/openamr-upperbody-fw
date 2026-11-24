# SPDX-License-Identifier: MIT
# CTL-017 CANopen and application layers, host tests and tools.
# Host build:   cmake -S . -B build && cmake --build build && ctest --test-dir build
# Cortex-M4:    cmake -S . -B build-m4 -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#               (object libraries only, no tests)
# The lift core, the plant model and the board port are not part of this
# build; they stay on the draft branch of upperbody-fw #16 for the owner.
cmake_minimum_required(VERSION 3.16)
project(openamr_upperbody_fw C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(CTL017_SANITIZE "Build host targets with ASan and UBSan" ON)
# Permit object extension (docs/ctl017_permit_contract.md). Off by default.
# When on, CTL017_PERMIT_DEFS must name every parameter of the contract,
# for example "CTL017_PERMIT_OD_INDEX=...;CTL017_PERMIT_COB_ID=...". No
# value is proposed here (TODO(Evans)).
option(CTL017_PERMIT_OD "Build the firmware libraries with the permit object" OFF)
set(CTL017_PERMIT_DEFS "" CACHE STRING "Permit contract parameters (TODO(Evans))")

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wno-sign-conversion)
  if(CTL017_SANITIZE AND NOT CMAKE_CROSSCOMPILING)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

add_subdirectory(firmware/ctl017)

if(NOT CMAKE_CROSSCOMPILING)
  enable_testing()
  add_subdirectory(tests/ctl017)
  add_subdirectory(tools/ctl017_od)
  add_subdirectory(tools/ctl017_sim)
endif()
