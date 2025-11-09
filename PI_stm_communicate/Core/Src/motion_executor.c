#include "motion_executor.h"

// Local TIM handle owned by this module
static TIM_HandleTypeDef htim2;

// Local state
static MotionXState xState = {
    .busy = false,
    .target_steps = 0,
    .steps_done = 0,
    .dir_positive = true
};

// Forward declarations
static uint32_t TIM2_Get_Clock_Hz(void);
static bool     Compute_Timer_Params(uint32_t steps_per_sec, uint16_t* out_psc, uint16_t* out_arr);
static void     TIM2_Configure_PWM(uint16_t psc, uint16_t arr);
static void     Step_Output_IdleLow(void);

void Motion_InitX(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitTypeDef gpio_dir = {0};
    gpio_dir.Pin   = X_DIR_GPIO_PIN;
    gpio_dir.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_dir.Pull  = GPIO_NOPULL;
    gpio_dir.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(X_DIR_GPIO_PORT, &gpio_dir);
    HAL_GPIO_WritePin(X_DIR_GPIO_PORT, X_DIR_GPIO_PIN, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio_step = {0};
    gpio_step.Pin   = X_STEP_GPIO_PIN;
    gpio_step.Mode  = GPIO_MODE_AF_PP;
    gpio_step.Pull  = GPIO_NOPULL;
    gpio_step.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(X_STEP_GPIO_PORT, &gpio_step);

    TIM2_Configure_PWM(/*psc*/ 7199, /*arr*/ 999);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    Step_Output_IdleLow();
}

bool Move_X(int32_t target_mm, uint32_t speed_mm_min)
{
    if (speed_mm_min == 0) return false;
    if (xState.busy)       return false;

    if (target_mm >= 0) {
        xState.dir_positive = true;
        HAL_GPIO_WritePin(X_DIR_GPIO_PORT, X_DIR_GPIO_PIN, GPIO_PIN_SET);
    } else {
        xState.dir_positive = false;
        HAL_GPIO_WritePin(X_DIR_GPIO_PORT, X_DIR_GPIO_PIN, GPIO_PIN_RESET);
    }

    uint32_t steps = (uint32_t)((target_mm >= 0) ? target_mm : -target_mm);
    steps *= X_STEPS_PER_MM;
    if (steps == 0) return true;

    uint64_t sps64 = ((uint64_t)speed_mm_min * (uint64_t)X_STEPS_PER_MM) / 60ULL;
    if (sps64 == 0ULL) return false;
    if (sps64 > 100000ULL) sps64 = 100000ULL;
    uint32_t steps_per_sec = (uint32_t)sps64;

    uint16_t psc = 0, arr = 0;
    if (!Compute_Timer_Params(steps_per_sec, &psc, &arr)) return false;

    TIM2_Configure_PWM(psc, arr);

    xState.target_steps = steps;
    xState.steps_done   = 0;
    xState.busy         = true;

    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        xState.busy = false;
        return false;
    }
    __HAL_TIM_ENABLE(&htim2);
    return true;
}

bool Motion_X_IsBusy(void) { return xState.busy; }

void Motion_X_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim2);
    Step_Output_IdleLow();
    xState.busy         = false;
    xState.target_steps = 0;
    xState.steps_done   = 0;
}

const MotionXState* Motion_X_GetState(void) { return &xState; }

static uint32_t TIM2_Get_Clock_Hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t cfgr = RCC->CFGR;
    uint32_t ppre1 = (cfgr & RCC_CFGR_PPRE1);
    bool ppre1_div1 = (ppre1 == RCC_CFGR_PPRE1_DIV1);
    return ppre1_div1 ? pclk1 : (pclk1 * 2U);
}

static bool Compute_Timer_Params(uint32_t steps_per_sec, uint16_t* out_psc, uint16_t* out_arr)
{
    if (steps_per_sec == 0) return false;

    uint32_t tim_clk = TIM2_Get_Clock_Hz();
    uint64_t period_ticks = (uint64_t)tim_clk / (uint64_t)steps_per_sec;
    if (period_ticks < 2ULL) period_ticks = 2ULL;

    uint64_t psc_calc = 0;
    if (period_ticks > 65536ULL) {
        psc_calc = (period_ticks + 65535ULL) / 65536ULL;
        if (psc_calc == 0) psc_calc = 1;
        if (psc_calc > 65536ULL) psc_calc = 65536ULL;
        psc_calc -= 1ULL;
    } else {
        psc_calc = 0;
    }

    uint32_t psc = (uint32_t)psc_calc;
    uint32_t arr = (uint32_t)(period_ticks / (psc + 1ULL));
    if (arr == 0) arr = 1;
    arr -= 1U;

    if (psc > 0xFFFFU || arr > 0xFFFFU) return false;

    *out_psc = (uint16_t)psc;
    *out_arr = (uint16_t)arr;
    return true;
}

static void TIM2_Configure_PWM(uint16_t psc, uint16_t arr)
{
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = psc;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = arr;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_DeInit(&htim2);
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = (uint32_t)((arr + 1U) / 2U);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
}

static void Step_Output_IdleLow(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = X_STEP_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(X_STEP_GPIO_PORT, &gpio);
    HAL_GPIO_WritePin(X_STEP_GPIO_PORT, X_STEP_GPIO_PIN, GPIO_PIN_RESET);

    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(X_STEP_GPIO_PORT, &gpio);
}

/* Renamed IRQ callback (call this from stm32f1xx_it.c TIM2_IRQHandler) */
void Motion_TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

            if (xState.busy) {
                xState.steps_done++;
                if (xState.steps_done >= xState.target_steps) {
                    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
                    __HAL_TIM_DISABLE(&htim2);
                    Step_Output_IdleLow();
                    xState.busy = false;
                }
            }
        }
    }
}
