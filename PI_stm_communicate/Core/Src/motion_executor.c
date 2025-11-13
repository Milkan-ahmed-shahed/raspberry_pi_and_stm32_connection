#include "motion_executor.h"
#include <math.h>       /* sqrtf only used in precompute (not ISR) */
#include <stdbool.h>

/* Local TIM handle */
static TIM_HandleTypeDef htim2;

/* Timer tick frequency after prescale (Hz) - float only used outside ISR */
static float g_tim2_tick_hz = 0.0f;

/* Global motion state */
static GantryState gState = {0};

/* -------- Internal prototypes -------- */
static uint32_t TIM2_Get_Clock_Hz(void);
static void     TIM2_Configure_Base(uint16_t psc, uint16_t arr);
static void     Configure_PWM_Channels(uint16_t arr);
static void     Set_Direction_Pins(void);
static void     Enable_Drivers(bool enable);
static uint16_t SPS_to_ARR(float steps_per_sec);
static void     Precompute_Trapezoid(uint32_t master_steps, uint32_t steps_to_accel, float vmax_mm_s);
static void     Apply_ARR(uint16_t arr, bool slave_pulse_enabled);  /* UPDATED */
static void     StopMotion(void);
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

/* -------- Initialization -------- */
void Motion_InitXY(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* DIR / ENA pins */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = X_DIR_GPIO_PIN; HAL_GPIO_Init(X_DIR_GPIO_PORT, &gpio);
    gpio.Pin = X_ENA_GPIO_PIN; HAL_GPIO_Init(X_ENA_GPIO_PORT, &gpio);
    gpio.Pin = Y_DIR_GPIO_PIN; HAL_GPIO_Init(Y_DIR_GPIO_PORT, &gpio);
    gpio.Pin = Y_ENA_GPIO_PIN; HAL_GPIO_Init(Y_ENA_GPIO_PORT, &gpio);

    /* STEP pins (PWM) */
    GPIO_InitTypeDef gpio_af = {0};
    gpio_af.Mode  = GPIO_MODE_AF_PP;
    gpio_af.Pull  = GPIO_NOPULL;
    gpio_af.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_af.Pin = X_STEP_GPIO_PIN; HAL_GPIO_Init(X_STEP_GPIO_PORT, &gpio_af);
    gpio_af.Pin = Y_STEP_GPIO_PIN; HAL_GPIO_Init(Y_STEP_GPIO_PORT, &gpio_af);

    TIM2_Configure_Base(TIM2_FIXED_PSC, 999);

    uint32_t tim_clk = TIM2_Get_Clock_Hz();
    g_tim2_tick_hz = (float)tim_clk / (float)(TIM2_FIXED_PSC + 1U);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    Enable_Drivers(false);
    gState = (GantryState){0};
}

/* -------- Public API -------- */
bool Move_Gantry_XY(int32_t x_mm, int32_t y_mm, uint32_t speed_mm_min)
{
    if (speed_mm_min == 0) return false;
    if (gState.busy)       return false;

    gState.x_dir_positive = (x_mm >= 0);
    gState.y_dir_positive = (y_mm >= 0);

    uint32_t x_steps = (uint32_t)((x_mm >= 0) ? x_mm : -x_mm) * XY_STEPS_PER_MM;
    uint32_t y_steps = (uint32_t)((y_mm >= 0) ? y_mm : -y_mm) * XY_STEPS_PER_MM;

    if (x_steps == 0 && y_steps == 0) return true;

    gState.x_total_steps = x_steps;
    gState.y_total_steps = y_steps;

    if (x_steps >= y_steps) {
        gState.master_is_x        = true;
        gState.master_total_steps = x_steps;
        gState.slave_total_steps  = y_steps;
    } else {
        gState.master_is_x        = false;
        gState.master_total_steps = y_steps;
        gState.slave_total_steps  = x_steps;
    }

    uint32_t vmax_mm_min = (speed_mm_min > MAX_VELOCITY_MM_MIN) ? MAX_VELOCITY_MM_MIN : speed_mm_min;
    float vmax_mm_s = (float)vmax_mm_min / 60.0f;

    /* Accel distance and steps */
    float d_accel_mm = (vmax_mm_s * vmax_mm_s) / (2.0f * 500.0f); /* A=500 mm/s^2 */
    uint32_t steps_to_accel = (uint32_t)(d_accel_mm * (float)XY_STEPS_PER_MM + 0.5f);

    if ((2U * steps_to_accel) > gState.master_total_steps) {
        steps_to_accel = gState.master_total_steps / 2U; /* triangular */
    }

    gState.steps_to_accel    = steps_to_accel;
    gState.steps_to_decel    = gState.master_total_steps - steps_to_accel;
    gState.steps_done_master = 0;
    gState.steps_done_slave  = 0;
    gState.ramp_state        = (steps_to_accel == 0U) ? RAMP_CONST : RAMP_UP;

    Precompute_Trapezoid(gState.master_total_steps, steps_to_accel, vmax_mm_s);

    /* Centered Bresenham accumulator */
    gState.y_accumulator = -(int32_t)(gState.master_total_steps / 2U);

    Set_Direction_Pins();
    Enable_Drivers(true);

    /* Load first ARR and ensure slave is gated OFF for the first period */
    uint16_t first_arr = (gState.ramp_len > 0) ? gState.ramp_buffer[0] : gState.vmax_arr;
    Apply_ARR(first_arr, /*slave_pulse_enabled*/ false);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim2);

    gState.busy = true;
    return true;
}

bool Move_X(int32_t x_mm, uint32_t speed_mm_min)
{
    return Move_Gantry_XY(x_mm, 0, speed_mm_min);
}

bool Motion_XY_IsBusy(void) { return gState.busy; }

void Motion_XY_Stop(void) { StopMotion(); }

const GantryState* Motion_XY_GetState(void) { return &gState; }

/* -------- IRQ Handler --------
   IMPORTANT: No CCER bit twiddling here.
   We gate the slave axis by writing CCR=0 for the slave channel when no step is needed.
   ARR and both CCRs are updated together (preload) and take effect the next period. */
void Motion_TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET) return;
    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

    if (!gState.busy) return;

    gState.steps_done_master++;

    if (gState.steps_done_master >= gState.master_total_steps) {
        StopMotion();
        return;
    }

    /* Determine next ARR based on ramp phase */
    uint16_t next_arr;
    if (gState.steps_done_master < gState.steps_to_accel) {
        uint32_t idx = gState.steps_done_master;
        if (idx >= gState.ramp_len) idx = gState.ramp_len - 1;
        next_arr = gState.ramp_buffer[idx];
        gState.ramp_state = RAMP_UP;
    } else if (gState.steps_done_master < gState.steps_to_decel) {
        next_arr = gState.vmax_arr;
        gState.ramp_state = RAMP_CONST;
    } else {
        uint32_t steps_into_decel = gState.steps_done_master - gState.steps_to_decel;
        uint32_t idx_from_end = (steps_into_decel >= gState.ramp_len) ? gState.ramp_len - 1 : steps_into_decel;
        uint32_t idx = (gState.ramp_len - 1U) - idx_from_end;
        next_arr = gState.ramp_buffer[idx];
        gState.ramp_state = RAMP_DOWN;
    }

    /* Bresenham: decide if slave needs a step in the NEXT period */
    bool enable_slave = false;
    if (gState.slave_total_steps > 0 && gState.steps_done_slave < gState.slave_total_steps) {
        gState.y_accumulator += (int32_t)gState.slave_total_steps;
        if (gState.y_accumulator >= 0) {
            enable_slave = true;
            gState.y_accumulator -= (int32_t)gState.master_total_steps;
            gState.steps_done_slave++;
        }
    }

    /* Apply ARR and CCRs atomically via preload (slave gated by CCR=0) */
    Apply_ARR(next_arr, enable_slave);
}

/* -------- Internal functions -------- */
static uint32_t TIM2_Get_Clock_Hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t cfgr  = RCC->CFGR;
    uint32_t ppre1 = (cfgr & RCC_CFGR_PPRE1);
    bool ppre1_div1 = (ppre1 == RCC_CFGR_PPRE1_DIV1);
    return ppre1_div1 ? pclk1 : (pclk1 * 2U);
}

static void TIM2_Configure_Base(uint16_t psc, uint16_t arr)
{
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = psc;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = arr;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_DeInit(&htim2);
    HAL_TIM_PWM_Init(&htim2);
    Configure_PWM_Channels(arr);
}

static void Configure_PWM_Channels(uint16_t arr)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = (uint32_t)((arr + 1U) / 2U);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_4);
}

static uint16_t SPS_to_ARR(float steps_per_sec)
{
    if (steps_per_sec < 1.0f) steps_per_sec = 1.0f;
    float period_ticks_f = g_tim2_tick_hz / steps_per_sec;
    if (period_ticks_f < 2.0f) period_ticks_f = 2.0f;
    uint32_t period_ticks = (uint32_t)(period_ticks_f + 0.5f);
    if (period_ticks > 65536U) period_ticks = 65536U;
    return (uint16_t)(period_ticks - 1U);
}

static void Precompute_Trapezoid(uint32_t master_steps, uint32_t steps_to_accel, float vmax_mm_s)
{
    float vmax_sps = vmax_mm_s * (float)XY_STEPS_PER_MM;
    uint16_t vmax_arr = SPS_to_ARR(vmax_sps);
    gState.vmax_arr = vmax_arr;

    uint32_t len = steps_to_accel;
    if (len > RAMP_BUFFER_SIZE) len = RAMP_BUFFER_SIZE;
    gState.ramp_len = (len == 0) ? 0 : len;

    for (uint32_t i = 0; i < len; ++i) {
        float s_mm = (float)(i + 1U) / (float)XY_STEPS_PER_MM;
        /* v = sqrt(2*A*s) with A=500 mm/s^2 */
        float v_mm_s = sqrtf(2.0f * 500.0f * s_mm);
        if (v_mm_s > vmax_mm_s) v_mm_s = vmax_mm_s;
        float sps = v_mm_s * (float)XY_STEPS_PER_MM;
        uint16_t arr = SPS_to_ARR(sps);
        if (arr < vmax_arr) arr = vmax_arr; /* do not exceed Vmax speed (lower ARR = higher freq) */
        gState.ramp_buffer[i] = arr;
    }
}

/* UPDATED: Gate the slave by writing CCR=0 for that channel (no CCER toggling). */
static void Apply_ARR(uint16_t arr, bool slave_pulse_enabled)
{
    gState.current_arr = arr;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);

    /* 50% duty for active channel; 0 for gated (no pulse) */
    uint32_t pulse = (arr + 1U) / 2U;
    uint32_t slave_pulse = slave_pulse_enabled ? pulse : 0U;

    if (gState.master_is_x) {
        /* X = master on CH1, Y = slave on CH4 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, slave_pulse);
    } else {
        /* Y = master on CH4, X = slave on CH1 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, slave_pulse);
    }
}

static void Set_Direction_Pins(void)
{
    HAL_GPIO_WritePin(X_DIR_GPIO_PORT, X_DIR_GPIO_PIN,
                      gState.x_dir_positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Y_DIR_GPIO_PORT, Y_DIR_GPIO_PIN,
                      gState.y_dir_positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Enable_Drivers(bool enable)
{
    /* Assuming ENA active LOW (common). Adjust if inverted. */
    GPIO_PinState state = enable ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(X_ENA_GPIO_PORT, X_ENA_GPIO_PIN, state);
    HAL_GPIO_WritePin(Y_ENA_GPIO_PORT, Y_ENA_GPIO_PIN, state);
}

static void StopMotion(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim2);

    Enable_Drivers(false);

    gState.busy               = false;
    gState.ramp_state         = RAMP_IDLE;
    gState.steps_done_master  = 0;
    gState.steps_done_slave   = 0;
    gState.current_arr        = 0;
    gState.ramp_len           = 0;
    gState.y_accumulator      = 0;
}
