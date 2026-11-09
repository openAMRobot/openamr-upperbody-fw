# CTL-017 board port: NUCLEO-G474RE

Status: skeleton, build-verified only. It has never run on a board. With the H-bridge pins unassigned (the default), this image cannot move the lift.

Board per P-03 item 30 A7 (decision D-11): NUCLEO-G474RE, Infineon BLDC-SHIELD_IFX007T, SN65HVD230 CAN transceiver.

## Pins

Fixed by P-03 item 30 A7, checked against the ST pin data for STM32G474RETx (STMicroelectronics/STM32_open_pin_data, commit 7d1f151):

| Function | Pin | Peripheral | AF |
|---|---|---|---|
| CAN3 RX (to SN65HVD230 R) | PA11 | FDCAN1_RX | AF9 |
| CAN3 TX (to SN65HVD230 D) | PA12 | FDCAN1_TX | AF9 |
| Hall A | PB2 | TIM5_CH1, encoder mode | AF2 |
| Hall B | PC12 | TIM5_CH2, encoder mode | AF1 |
| Shared current sense | PC1 | ADC1_IN7 | analog |
| Board LED LD2 (status blink) | PA5 | GPIO | |

Open, `BOARD_PIN_NONE` in `board_pins.h` until the owner assigns them (TODO(Evans), CTL-017 pin budget):

| Function | Notes |
|---|---|
| IFX007T IN and INH of the two half-bridges used for the DOLD motor, PWM timer channel | direction by polarity on DOLD pins 1 and 8 |
| Stop sensors L1000, L1175, L1350 | NPN NO, 4.7 kOhm pull-up to 24 V, 27k/3.3k divider, clamp and TVS: pin LOW = target present (`BOARD_STOP_ACTIVE_LOW`) |
| DOLD end switches | only if wired out; DOLD data open |
| Motor-supply sense | divider from the switched lift feed, threshold about 18 V (Anushka 27 Oct) |
| Hall-supply monitor | power-good or ADC (CAN3 v1, platform-fw #12) |
| Second E-stop spare NO | diagnostics only |
| Spare inputs 1 to 3 | functional only |
| Head light low-side PWM | Falcon FLDL-i150x15-W, 24 V, 0.32 A; switch LED-, output rated at least 1 A |
| Getac ZX80 power or charging enable | |

Other open board values: FDCAN bit timing must match the base controller (here 170 MHz / 10, 1 + 29 + 4 tq, sample point 88 %), TIM5 input filter and Hall pull-ups (DOLD Hall electrical data), current-sense scale `BOARD_ISENSE_UA_PER_COUNT` (0 = current limit inactive), IWDG timeout (about 100 ms here).

## What the port does

- 170 MHz from HSI and PLL, SysTick 1 kHz.
- FDCAN1 classic CAN 500 kbit/s, all standard data frames into RX FIFO 0, polled; the CANopen layer filters by identifier.
- TIM5 32-bit encoder counter on the two Hall channels; the signed count goes to the core every millisecond.
- ADC1 IN7 polled once per millisecond.
- 1 kHz loop: read inputs, `ctl017_app_step()`, apply H-bridge, head light and Getac outputs, feed the IWDG. NMT reset node ends in `NVIC_SystemReset()`.
- Reset cause in 0x2300:02 (bit 0 pin, 1 brown-out, 2 software, 3 IWDG, 4 WWDG, 5 low power, 6 option byte; proposal), serial number in 0x1018:04 from the unique ID.
- Outputs are driven low before their pins become outputs.
- Until a PWM timer channel is assigned, the H-bridge is driven only for a 1000 permille duty; a reduced-speed request is not applied rather than run at full speed.

## Build

```sh
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi
boards/nucleo_g474re/fetch_hal.sh          # ST HAL, CMSIS device and core at pinned commits into vendor/ (not committed)
cmake -S boards/nucleo_g474re -B build-g474 -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-g474                    # ctl017.elf, ctl017.bin, ctl017.map
```

Fetched, not vendored: STMicroelectronics stm32g4xx_hal_driver v1.2.7 (BSD-3-Clause), cmsis_device_g4 v1.2.6 (Apache-2.0), cmsis_core v5.9.0 (Apache-2.0). The HAL configuration is ST's `stm32g4xx_hal_conf_template.h` copied unchanged at configure time. The linker script is our own (`stm32g474re_flash.ld`, 512 KB flash, 96 KB SRAM1 plus SRAM2).

A CubeMX project was not generated: CubeMX does not run headless here. When Evans fixes the pin budget, a CubeMX `.ioc` can replace `board_pins.h` and the init functions; the application interface stays `ctl017_app_init()`, `ctl017_app_rx()` and `ctl017_app_step()`.
