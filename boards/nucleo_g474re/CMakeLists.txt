# SPDX-License-Identifier: MIT
# CTL-017 firmware image for the NUCLEO-G474RE.
#   boards/nucleo_g474re/fetch_hal.sh
#   cmake -S boards/nucleo_g474re -B build-g474 -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build-g474
cmake_minimum_required(VERSION 3.16)
project(ctl017_nucleo_g474re C ASM)

set(CMAKE_C_STANDARD 11)
set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(VENDOR ${CMAKE_CURRENT_SOURCE_DIR}/vendor CACHE PATH "Output of fetch_hal.sh")
set(HAL ${VENDOR}/stm32g4xx_hal_driver)
set(DEV ${VENDOR}/cmsis_device_g4)
set(CORE ${VENDOR}/cmsis_core)

if(NOT EXISTS ${HAL}/Inc/stm32g4xx_hal.h)
  message(FATAL_ERROR "ST HAL not found in ${VENDOR}: run boards/nucleo_g474re/fetch_hal.sh")
endif()

# HAL configuration: the ST template unchanged (all modules declared; only
# the drivers listed below are compiled).
configure_file(${HAL}/Inc/stm32g4xx_hal_conf_template.h ${CMAKE_BINARY_DIR}/conf/stm32g4xx_hal_conf.h COPYONLY)

set(HAL_SRC)
foreach(m hal hal_cortex hal_rcc hal_rcc_ex hal_pwr hal_pwr_ex hal_gpio hal_fdcan hal_tim hal_tim_ex
          hal_adc hal_adc_ex hal_iwdg hal_flash hal_flash_ex hal_dma hal_exti ll_adc)
  list(APPEND HAL_SRC ${HAL}/Src/stm32g4xx_${m}.c)
endforeach()

add_library(st_hal STATIC ${HAL_SRC} ${DEV}/Source/Templates/system_stm32g4xx.c)
target_include_directories(st_hal PUBLIC ${CMAKE_BINARY_DIR}/conf ${HAL}/Inc ${DEV}/Include ${CORE}/Include)
target_compile_definitions(st_hal PUBLIC STM32G474xx USE_HAL_DRIVER)
target_compile_options(st_hal PRIVATE -O2 -w)

add_library(ctl017_fw STATIC
  ${REPO}/firmware/ctl017/core/src/debounce.c
  ${REPO}/firmware/ctl017/core/src/lift_config.c
  ${REPO}/firmware/ctl017/core/src/lift.c
  ${REPO}/firmware/ctl017/core/src/aux_io.c
  ${REPO}/firmware/ctl017/canopen/src/co_node.c
  ${REPO}/firmware/ctl017/app/src/ctl017_app.c
  ${REPO}/firmware/ctl017/app/src/ctl017_od.c
  ${REPO}/firmware/ctl017/app/src/lift_pdo.c
)
target_include_directories(ctl017_fw PUBLIC
  ${REPO}/firmware/ctl017/core/include ${REPO}/firmware/ctl017/canopen/include ${REPO}/firmware/ctl017/app/include)
target_compile_options(ctl017_fw PRIVATE -Os -Wall -Wextra -Werror)

add_executable(ctl017.elf board_main.c ${DEV}/Source/Templates/gcc/startup_stm32g474xx.s)
target_include_directories(ctl017.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(ctl017.elf PRIVATE $<$<COMPILE_LANGUAGE:C>:-Os -Wall -Wextra>)
target_link_libraries(ctl017.elf PRIVATE ctl017_fw st_hal)
target_link_options(ctl017.elf PRIVATE -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g474re_flash.ld -Wl,-Map=ctl017.map)
add_custom_command(TARGET ctl017.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary ctl017.elf ctl017.bin
  COMMAND ${CMAKE_SIZE} ctl017.elf)
