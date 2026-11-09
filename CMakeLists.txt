# SPDX-License-Identifier: MIT
# Host build of the CTL-017 firmware core, CANopen slave and tests.
# The NUCLEO-G474RE build lives in boards/nucleo_g474re (cross toolchain).
cmake_minimum_required(VERSION 3.16)
project(openamr_upperbody_fw C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(CTL017_SANITIZE "Build host targets with ASan and UBSan" ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wno-sign-conversion)
  if(CTL017_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

add_subdirectory(firmware/ctl017)

enable_testing()
add_subdirectory(tests/ctl017)
add_subdirectory(tools/ctl017_od)
