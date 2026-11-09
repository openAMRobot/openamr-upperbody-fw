/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 on NUCLEO-G474RE with BLDC-SHIELD_IFX007T and SN65HVD230.
 *
 * Fixed by P-03 item 30 A7 (decision D-11) and checked against the ST pin
 * data for STM32G474RETx (STM32_open_pin_data):
 *   FDCAN1 RX PA11 (AF9), FDCAN1 TX PA12 (AF9)
 *   Hall A TIM5_CH1 PB2 (AF2), Hall B TIM5_CH2 PC12 (AF1), encoder mode
 *   Shared current sense PC1 = ADC1_IN7
 *
 * Every other pin is open: TODO(Evans) in the CTL-017 pin budget. An open
 * pin is BOARD_PIN_NONE; the port then reads that input as inactive and
 * drives nothing. With the H-bridge pins open the lift cannot move.
 */
#ifndef CTL017_BOARD_PINS_H
#define CTL017_BOARD_PINS_H

#include "stm32g4xx_hal.h"

typedef struct {
    GPIO_TypeDef *port;   /* NULL = not assigned */
    uint16_t pin;
} board_pin_t;

#define BOARD_PIN_NONE {NULL, 0u}

/* Fixed pins (P-03 item 30 A7) */
#define BOARD_FDCAN1_RX   {GPIOA, GPIO_PIN_11}
#define BOARD_FDCAN1_TX   {GPIOA, GPIO_PIN_12}
#define BOARD_FDCAN1_AF   GPIO_AF9_FDCAN1
#define BOARD_HALL_A      {GPIOB, GPIO_PIN_2}
#define BOARD_HALL_A_AF   GPIO_AF2_TIM5
#define BOARD_HALL_B      {GPIOC, GPIO_PIN_12}
#define BOARD_HALL_B_AF   GPIO_AF1_TIM5
#define BOARD_ISENSE      {GPIOC, GPIO_PIN_1}
#define BOARD_ISENSE_ADC_CHANNEL ADC_CHANNEL_7   /* ADC1_IN7 */

/* IFX007T half-bridge inputs. TODO(Evans): IN and INH pins of the two
 * half-bridges used for the DOLD motor (pins 1 and 8 of the column) on the
 * Arduino header, and the PWM timer and channels for IN. */
#define BOARD_HB_IN_A     BOARD_PIN_NONE
#define BOARD_HB_IN_B     BOARD_PIN_NONE
#define BOARD_HB_INH_A    BOARD_PIN_NONE
#define BOARD_HB_INH_B    BOARD_PIN_NONE

/* Stop sensors L1000, L1175, L1350 (SICK IME12-04BNSZC0S, NPN NO, 4.7 kOhm
 * pull-up to 24 V, 27k/3.3k divider, clamp and TVS: the pin reads LOW when
 * the target is present). TODO(Evans): pins. */
#define BOARD_STOP0       BOARD_PIN_NONE
#define BOARD_STOP1       BOARD_PIN_NONE
#define BOARD_STOP2       BOARD_PIN_NONE
#define BOARD_STOP_ACTIVE_LOW 1

/* DOLD end switches, only if wired out (TODO(Evans): DOLD data). */
#define BOARD_LOWER_LIMIT BOARD_PIN_NONE
#define BOARD_UPPER_LIMIT BOARD_PIN_NONE
#define BOARD_LIMIT_ACTIVE_LOW 1   /* TODO(Evans) */

/* Functional status inputs (Anushka 27 Oct): motor-supply sense (divider,
 * threshold about 18 V) and Hall-supply monitor. TODO(Evans): pins and
 * polarity; digital comparator input assumed. */
#define BOARD_MOTOR_SENSE BOARD_PIN_NONE
#define BOARD_HALL_SUPPLY_OK BOARD_PIN_NONE

/* Second E-stop spare NO contact (diagnostics only): pull-up, contact to
 * ground, LOW = pressed. TODO(Evans): pin and input circuit. */
#define BOARD_ESTOP2_NO   BOARD_PIN_NONE

/* Spare functional inputs (P-03 item 29). TODO(Evans). */
#define BOARD_SPARE1      BOARD_PIN_NONE
#define BOARD_SPARE2      BOARD_PIN_NONE
#define BOARD_SPARE3      BOARD_PIN_NONE

/* Head light low-side PWM output (Falcon FLDL-i150x15-W, 24 V, 0.32 A, switch
 * LED-, output rated at least 1 A) and Getac power or charging enable.
 * TODO(Evans): pins, timer channel for the light PWM, driver part. */
#define BOARD_HEAD_LIGHT  BOARD_PIN_NONE
#define BOARD_GETAC_EN    BOARD_PIN_NONE

/* Board LED LD2 (NUCLEO-G474RE): heartbeat blink. */
#define BOARD_LED         {GPIOA, GPIO_PIN_5}

/* Current sense scale and the drive gate: board_drive_config.h. */

#endif /* CTL017_BOARD_PINS_H */
