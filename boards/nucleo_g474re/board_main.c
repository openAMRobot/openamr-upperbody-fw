/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 board port for the NUCLEO-G474RE: clock, FDCAN1 at 500 kbit/s,
 * TIM5 Hall counter, ADC1 current sense, GPIO, IWDG, and the 1 kHz loop that
 * feeds the board-independent application (firmware/ctl017/app).
 *
 * Skeleton: build-verified only, never run on hardware. Pins not fixed by
 * P-03 item 30 A7 are BOARD_PIN_NONE (board_pins.h, TODO(Evans)); with the
 * H-bridge pins unassigned this firmware cannot move the lift.
 */
#include <string.h>

#include "board_pins.h"
#include "ctl017/ctl017_app.h"
#include "stm32g4xx_hal.h"

static FDCAN_HandleTypeDef hfdcan1;
static TIM_HandleTypeDef htim5;
static ADC_HandleTypeDef hadc1;
static IWDG_HandleTypeDef hiwdg;
static ctl017_app_t app;
static uint32_t can_tx_dropped;

static const board_pin_t pin_stop[LIFT_NUM_STOPS] = {BOARD_STOP0, BOARD_STOP1, BOARD_STOP2};
static const board_pin_t pin_lower = BOARD_LOWER_LIMIT;
static const board_pin_t pin_upper = BOARD_UPPER_LIMIT;
static const board_pin_t pin_motor_sense = BOARD_MOTOR_SENSE;
static const board_pin_t pin_hall_supply = BOARD_HALL_SUPPLY_OK;
static const board_pin_t pin_estop2 = BOARD_ESTOP2_NO;
static const board_pin_t pin_spare[AUX_NUM_SPARE_IN] = {BOARD_SPARE1, BOARD_SPARE2, BOARD_SPARE3};
static const board_pin_t pin_light = BOARD_HEAD_LIGHT;
static const board_pin_t pin_getac = BOARD_GETAC_EN;
static const board_pin_t pin_in_a = BOARD_HB_IN_A;
static const board_pin_t pin_in_b = BOARD_HB_IN_B;
static const board_pin_t pin_inh_a = BOARD_HB_INH_A;
static const board_pin_t pin_inh_b = BOARD_HB_INH_B;
static const board_pin_t pin_led = BOARD_LED;

void Error_Handler(void);

/* ---- Clock: HSI 16 MHz, PLL to 170 MHz, range 1 boost ------------------- */
static void clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef per = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSIState = RCC_LSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = RCC_PLLM_DIV4;
    osc.PLL.PLLN = 85;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
    per.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_ADC12;
    per.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    per.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&per) != HAL_OK) {
        Error_Handler();
    }
}

/* ---- GPIO helpers -------------------------------------------------------- */
static void enable_port_clock(GPIO_TypeDef *p)
{
    if (p == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (p == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (p == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (p == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
}

static void pin_input(const board_pin_t *p, uint32_t pull)
{
    if (p->port == NULL) {
        return;
    }
    GPIO_InitTypeDef g = {0};
    enable_port_clock(p->port);
    g.Pin = p->pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = pull;
    HAL_GPIO_Init(p->port, &g);
}

static void pin_output(const board_pin_t *p)
{
    if (p->port == NULL) {
        return;
    }
    GPIO_InitTypeDef g = {0};
    enable_port_clock(p->port);
    HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_RESET); /* off before enabling the driver */
    g.Pin = p->pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(p->port, &g);
}

static bool pin_read(const board_pin_t *p, bool active_low)
{
    if (p->port == NULL) {
        return false;
    }
    bool high = HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET;
    return active_low ? !high : high;
}

static void pin_write(const board_pin_t *p, bool on)
{
    if (p->port != NULL) {
        HAL_GPIO_WritePin(p->port, p->pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void gpio_init(void)
{
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        pin_input(&pin_stop[k], GPIO_NOPULL); /* external divider */
    }
    pin_input(&pin_lower, GPIO_NOPULL);
    pin_input(&pin_upper, GPIO_NOPULL);
    pin_input(&pin_motor_sense, GPIO_NOPULL);
    pin_input(&pin_hall_supply, GPIO_NOPULL);
    pin_input(&pin_estop2, GPIO_PULLUP);
    for (unsigned i = 0; i < AUX_NUM_SPARE_IN; i++) {
        pin_input(&pin_spare[i], GPIO_PULLUP);
    }
    pin_output(&pin_inh_a);
    pin_output(&pin_inh_b);
    pin_output(&pin_in_a);
    pin_output(&pin_in_b);
    pin_output(&pin_light);
    pin_output(&pin_getac);
    pin_output(&pin_led);
}

/* ---- FDCAN1, classic CAN 500 kbit/s ---------------------------------------- */
/* 170 MHz / 10 = 17 MHz, 1 + 29 + 4 = 34 tq, sample point 88 %.
 * TODO(Evans): match the base controller FDCAN3 bit timing. */
static void fdcan_init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1 = 29;
    hfdcan1.Init.NominalTimeSeg2 = 4;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    /* All standard data frames into FIFO 0; the CANopen layer filters by ID. */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *h)
{
    if (h->Instance != FDCAN1) {
        return;
    }
    const board_pin_t rx = BOARD_FDCAN1_RX;
    const board_pin_t tx = BOARD_FDCAN1_TX;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_FDCAN_CLK_ENABLE();
    enable_port_clock(rx.port);
    g.Pin = rx.pin | tx.pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = BOARD_FDCAN1_AF;
    HAL_GPIO_Init(rx.port, &g);
}

static const uint32_t dlc_code[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
};

static bool can_send(void *ctx, const co_frame_t *f)
{
    (void)ctx;
    FDCAN_TxHeaderTypeDef th = {0};
    th.Identifier = f->id;
    th.IdType = FDCAN_STANDARD_ID;
    th.TxFrameType = FDCAN_DATA_FRAME;
    th.DataLength = dlc_code[f->dlc > 8u ? 8u : f->dlc];
    th.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    th.BitRateSwitch = FDCAN_BRS_OFF;
    th.FDFormat = FDCAN_CLASSIC_CAN;
    th.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    th.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &th, (uint8_t *)f->data) != HAL_OK) {
        can_tx_dropped++;
        return false;
    }
    return true;
}

static void can_poll(uint32_t now)
{
    FDCAN_RxHeaderTypeDef rh;
    uint8_t data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rh, data) != HAL_OK) {
            break;
        }
        if (rh.IdType != FDCAN_STANDARD_ID || rh.RxFrameType != FDCAN_DATA_FRAME) {
            continue;
        }
        co_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id = (uint16_t)rh.Identifier;
        for (uint8_t n = 0; n <= 8u; n++) {
            if (dlc_code[n] == rh.DataLength) {
                f.dlc = n;
            }
        }
        memcpy(f.data, data, f.dlc);
        ctl017_app_rx(&app, &f, now);
    }
}

/* ---- TIM5 encoder on the two Hall channels --------------------------------- */
static void tim5_init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFFu;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 0x8; /* TODO(Evans): input filter for the DOLD Hall signals */
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 0x8;
    if (HAL_TIM_Encoder_Init(&htim5, &enc) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM5) {
        return;
    }
    const board_pin_t a = BOARD_HALL_A;
    const board_pin_t b = BOARD_HALL_B;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM5_CLK_ENABLE();
    enable_port_clock(a.port);
    enable_port_clock(b.port);
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL; /* TODO(Evans): DOLD Hall output type and pull-ups */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = a.pin;
    g.Alternate = BOARD_HALL_A_AF;
    HAL_GPIO_Init(a.port, &g);
    g.Pin = b.pin;
    g.Alternate = BOARD_HALL_B_AF;
    HAL_GPIO_Init(b.port, &g);
}

/* ---- ADC1 IN7 (PC1) shared current sense ----------------------------------- */
static void adc_init(void)
{
    ADC_ChannelConfTypeDef ch = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    ch.Channel = BOARD_ISENSE_ADC_CHANNEL;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) {
        Error_Handler();
    }
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *h)
{
    if (h->Instance != ADC1) {
        return;
    }
    const board_pin_t is = BOARD_ISENSE;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_ADC12_CLK_ENABLE();
    enable_port_clock(is.port);
    g.Pin = is.pin;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(is.port, &g);
}

static uint16_t current_ma(void)
{
    uint32_t raw = 0;
    if (HAL_ADC_Start(&hadc1) == HAL_OK && HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc1);
    }
    uint32_t ma = raw * BOARD_ISENSE_UA_PER_COUNT / 1000u;
    return (uint16_t)(ma > 0xFFFFu ? 0xFFFFu : ma);
}

/* ---- IWDG: about 100 ms, fed only by the control loop ---------------------- */
static void iwdg_init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32; /* LSI 32 kHz / 32 = 1 kHz */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload = 100;                  /* TODO(Evans) */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

/* Reset cause bits for 0x2300:02 (proposal, TODO(Evans) encoding):
 * bit 0 pin, 1 brown-out, 2 software, 3 IWDG, 4 WWDG, 5 low power, 6 option byte. */
static uint8_t read_reset_cause(void)
{
    uint8_t r = 0;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) r |= 1u << 0;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) r |= 1u << 1;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) r |= 1u << 2;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) r |= 1u << 3;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) r |= 1u << 4;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) r |= 1u << 5;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST)) r |= 1u << 6;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return r;
}

/* ---- H-bridge ------------------------------------------------------------- */
/* IFX007T: INH high enables a half-bridge, IN selects high or low side.
 * Direction by polarity on DOLD pins 1 and 8 (DOLD interface data).
 * TODO(Evans): PWM on IN through a timer channel. Until a PWM channel is
 * assigned, a duty below 1000 permille is not applied (no drive), so a
 * reduced-speed command never runs at full speed by mistake. */
static void hbridge_apply(const lift_hbridge_cmd_t *hb)
{
    bool full = hb->duty_permille >= 1000u;
    if (!hb->enable || (!hb->brake && !full)) {
        pin_write(&pin_inh_a, false);
        pin_write(&pin_inh_b, false);
        pin_write(&pin_in_a, false);
        pin_write(&pin_in_b, false);
        return;
    }
    if (hb->brake) {
        pin_write(&pin_in_a, false); /* both low sides on */
        pin_write(&pin_in_b, false);
    } else {
        pin_write(&pin_in_a, hb->dir == LIFT_DIR_UP);
        pin_write(&pin_in_b, hb->dir == LIFT_DIR_DOWN);
    }
    pin_write(&pin_inh_a, true);
    pin_write(&pin_inh_b, true);
}

/* ---- Main loop --------------------------------------------------------------- */
int main(void)
{
    HAL_Init();
    clock_config();
    gpio_init();
    uint8_t reset_cause = read_reset_cause();
    fdcan_init();
    tim5_init();
    adc_init();

    lift_config_t lc;
    aux_config_t ac;
    lift_config_default(&lc);
    aux_config_default(&ac);
    if (!ctl017_app_init(&app, &lc, &ac, can_send, NULL, HAL_GetTick())) {
        Error_Handler();
    }
    app.reset_cause = reset_cause;
    app.serial = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
    iwdg_init();

    uint32_t last = HAL_GetTick();
    for (;;) {
        uint32_t now = HAL_GetTick();
        can_poll(now);
        if (now == last) {
            continue;
        }
        last = now;

        lift_inputs_t li;
        aux_inputs_t ai;
        memset(&li, 0, sizeof(li));
        memset(&ai, 0, sizeof(ai));
        li.now_ms = now;
        li.hall_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        li.hall_supply_ok = pin_read(&pin_hall_supply, false);
        li.motor_power_present = pin_read(&pin_motor_sense, false);
        for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
            li.stop_sensor[k] = pin_read(&pin_stop[k], BOARD_STOP_ACTIVE_LOW != 0);
        }
        li.lower_limit = pin_read(&pin_lower, BOARD_LIMIT_ACTIVE_LOW != 0);
        li.upper_limit = pin_read(&pin_upper, BOARD_LIMIT_ACTIVE_LOW != 0);
        li.motor_current_ma = current_ma();
        ai.now_ms = now;
        ai.estop2_no_closed = pin_read(&pin_estop2, true);
        for (unsigned i = 0; i < AUX_NUM_SPARE_IN; i++) {
            ai.spare_in[i] = pin_read(&pin_spare[i], true);
        }

        const ctl017_outputs_t *out = ctl017_app_step(&app, &li, &ai);
        hbridge_apply(&out->hbridge);
        pin_write(&pin_light, out->aux.head_light_on); /* TODO(Evans): PWM dim level */
        pin_write(&pin_getac, out->aux.getac_enable);
        pin_write(&pin_led, (now / 500u) % 2u == 0u);
        if (out->reset_requested) {
            NVIC_SystemReset();
        }
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* ---- Handlers ------------------------------------------------------------------ */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
        /* IWDG (if started) resets the board; outputs stay at reset level. */
    }
}

void HardFault_Handler(void)
{
    Error_Handler();
}
