#include "motion_executor_z.h"
#include <math.h>

/* Local TIM handle for Z axis (TIM3) */
static TIM_HandleTypeDef htim3;

/* Timer tick frequency after prescale (Hz) */
static float g_tim3_tick_hz = 0.0f;

/* Z axis motion state */
static Z_MotionState zState = {0};

/* -------- Internal prototypes -------- */
static uint32_t TIM3_Get_Clock_Hz(void);
static void     TIM3_Configure_Base(uint16_t psc, uint16_t arr);
static void     Configure_Z_PWM_Channel(uint16_t arr);
static void     Set_Z_Direction(void);
static void     Enable_Z_Driver(bool enable);
static uint16_t Z_SPS_to_ARR(float steps_per_sec);
static void     Z_Precompute_Trapezoid(uint32_t total_steps, uint32_t steps_to_accel, float vmax_mm_s);
static void     Z_Apply_ARR(uint16_t arr);
static void     Z_StopInternal(void);

/* -------- Public API -------- */
void Motion_InitZ(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* DIR / ENA pins */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = Z_DIR_GPIO_PIN; HAL_GPIO_Init(Z_DIR_GPIO_PORT, &gpio);
    gpio.Pin = Z_ENA_GPIO_PIN; HAL_GPIO_Init(Z_ENA_GPIO_PORT, &gpio);

    /* STEP pin PB0 as AF PP (TIM3_CH3) */
    GPIO_InitTypeDef gpio_af = {0};
    gpio_af.Pin   = Z_STEP_GPIO_PIN;
    gpio_af.Mode  = GPIO_MODE_AF_PP;
    gpio_af.Pull  = GPIO_NOPULL;
    gpio_af.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Z_STEP_GPIO_PORT, &gpio_af);

    TIM3_Configure_Base(TIM3_FIXED_PSC, 999);

    /* Derive tick frequency */
    uint32_t tim_clk = TIM3_Get_Clock_Hz();  /* ~48 MHz */
    g_tim3_tick_hz = (float)tim_clk / (float)(TIM3_FIXED_PSC + 1U);

    /* NVIC */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    Enable_Z_Driver(false);
    zState = (Z_MotionState){0};
}

bool Move_Z(int32_t z_mm, uint32_t speed_mm_min)
{
    if (speed_mm_min == 0) return false;
    if (zState.busy)       return false;

    zState.dir_positive = (z_mm >= 0);
    uint32_t abs_mm = (uint32_t)((z_mm >= 0) ? z_mm : -z_mm);
    uint32_t steps = abs_mm * Z_STEPS_PER_MM;
    if (steps == 0) return true;

    zState.total_steps = steps;

    /* Cap speed */
    uint32_t vmax_mm_min = (speed_mm_min > Z_MAX_VELOCITY_MM_MIN) ? Z_MAX_VELOCITY_MM_MIN : speed_mm_min;
    float vmax_mm_s = (float)vmax_mm_min / 60.0f;

    /* Accel distance and steps */
    float d_accel_mm = (vmax_mm_s * vmax_mm_s) / (2.0f * Z_ACCELERATION_MM_S2); /* A=500 */
    uint32_t steps_to_accel = (uint32_t)(d_accel_mm * (float)Z_STEPS_PER_MM + 0.5f);

    if ((2U * steps_to_accel) > steps) {
        steps_to_accel = steps / 2U; /* triangular */
    }

    zState.steps_to_accel = steps_to_accel;
    zState.steps_to_decel = steps - steps_to_accel;
    zState.steps_done     = 0;
    zState.ramp_state     = (steps_to_accel == 0U) ? Z_RAMP_CONST : Z_RAMP_UP;

    Z_Precompute_Trapezoid(steps, steps_to_accel, vmax_mm_s);

    /* Direction & driver */
    Set_Z_Direction();
    Enable_Z_Driver(true);

    uint16_t first_arr = (zState.ramp_len > 0) ? zState.ramp_buffer[0] : zState.vmax_arr;
    Z_Apply_ARR(first_arr);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);

    zState.busy = true;
    return true;
}

bool Motion_Z_IsBusy(void)
{
    return zState.busy;
}

void Motion_Z_Stop(void)
{
    Z_StopInternal();
}

const Z_MotionState* Motion_Z_GetState(void)
{
    return &zState;
}

/* -------- IRQ Handler Trampoline -------- */
void Motion_TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) == RESET) return;
    if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

    if (!zState.busy) return;

    zState.steps_done++;

    if (zState.steps_done >= zState.total_steps) {
        Z_StopInternal();
        return;
    }

    uint16_t next_arr;
    if (zState.steps_done < zState.steps_to_accel) {
        uint32_t idx = zState.steps_done;
        if (idx >= zState.ramp_len) idx = zState.ramp_len - 1;
        next_arr = zState.ramp_buffer[idx];
        zState.ramp_state = Z_RAMP_UP;
    } else if (zState.steps_done < zState.steps_to_decel) {
        next_arr = zState.vmax_arr;
        zState.ramp_state = Z_RAMP_CONST;
    } else {
        uint32_t steps_into_decel = zState.steps_done - zState.steps_to_decel;
        uint32_t idx_from_end = (steps_into_decel >= zState.ramp_len) ? zState.ramp_len - 1 : steps_into_decel;
        uint32_t idx = (zState.ramp_len - 1U) - idx_from_end;
        next_arr = zState.ramp_buffer[idx];
        zState.ramp_state = Z_RAMP_DOWN;
    }

    Z_Apply_ARR(next_arr);
}

/* -------- Internal functions -------- */
static uint32_t TIM3_Get_Clock_Hz(void)
{
    /* APB1 prescaler divides, timer clock doubles if prescaler >1 */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t cfgr  = RCC->CFGR;
    uint32_t ppre1 = (cfgr & RCC_CFGR_PPRE1);
    bool ppre1_div1 = (ppre1 == RCC_CFGR_PPRE1_DIV1);
    return ppre1_div1 ? pclk1 : (pclk1 * 2U);
}

static void TIM3_Configure_Base(uint16_t psc, uint16_t arr)
{
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = psc;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = arr;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_DeInit(&htim3);
    HAL_TIM_PWM_Init(&htim3);
    Configure_Z_PWM_Channel(arr);
}

static void Configure_Z_PWM_Channel(uint16_t arr)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = (uint32_t)((arr + 1U) / 2U);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_3);
}

static uint16_t Z_SPS_to_ARR(float steps_per_sec)
{
    if (steps_per_sec < 1.0f) steps_per_sec = 1.0f;
    float period_ticks_f = g_tim3_tick_hz / steps_per_sec;
    if (period_ticks_f < 2.0f) period_ticks_f = 2.0f;
    uint32_t period_ticks = (uint32_t)(period_ticks_f + 0.5f);
    if (period_ticks > 65536U) period_ticks = 65536U;
    return (uint16_t)(period_ticks - 1U);
}

static void Z_Precompute_Trapezoid(uint32_t total_steps, uint32_t steps_to_accel, float vmax_mm_s)
{
    float vmax_sps = vmax_mm_s * (float)Z_STEPS_PER_MM;
    uint16_t vmax_arr = Z_SPS_to_ARR(vmax_sps);
    zState.vmax_arr = vmax_arr;

    uint32_t len = steps_to_accel;
    if (len > Z_RAMP_BUFFER_SIZE) len = Z_RAMP_BUFFER_SIZE;
    zState.ramp_len = (len == 0) ? 0 : len;

    for (uint32_t i = 0; i < len; ++i) {
        float s_mm = (float)(i + 1U) / (float)Z_STEPS_PER_MM;
        float v_mm_s = sqrtf(2.0f * Z_ACCELERATION_MM_S2 * s_mm);
        if (v_mm_s > vmax_mm_s) v_mm_s = vmax_mm_s;
        float sps = v_mm_s * (float)Z_STEPS_PER_MM;
        uint16_t arr = Z_SPS_to_ARR(sps);
        if (arr < vmax_arr) arr = vmax_arr;
        zState.ramp_buffer[i] = arr;
    }
}

static void Z_Apply_ARR(uint16_t arr)
{
    zState.current_arr = arr;
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    uint32_t pulse = (arr + 1U) / 2U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
}

static void Set_Z_Direction(void)
{
    HAL_GPIO_WritePin(Z_DIR_GPIO_PORT, Z_DIR_GPIO_PIN,
                      zState.dir_positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Enable_Z_Driver(bool enable)
{
    /* Active LOW assumed */
    HAL_GPIO_WritePin(Z_ENA_GPIO_PORT, Z_ENA_GPIO_PIN,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Z_StopInternal(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim3);

    Enable_Z_Driver(false);

    zState.busy        = false;
    zState.ramp_state  = Z_RAMP_IDLE;
    zState.steps_done  = 0;
    zState.current_arr = 0;
    zState.ramp_len    = 0;
}
