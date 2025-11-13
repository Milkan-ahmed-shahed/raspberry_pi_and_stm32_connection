#include "servo_executor.h"

/* Local TIM4 handle */
static TIM_HandleTypeDef htim4;
/* Tick frequency after prescaler (Hz) as float for easy pulse math */
static float g_tim4_tick_hz = 0.0f;

/* Map [0..180] degrees to [SERVO_MIN_US..SERVO_MAX_US] microseconds (linear) */
static uint32_t angle_to_us(uint8_t angle)
{
    if (angle > 180U) angle = 180U;
    /* us = MIN + (angle/180)*(MAX-MIN) */
    uint32_t span = (SERVO_MAX_US - SERVO_MIN_US);
    uint32_t us = SERVO_MIN_US + (uint32_t)((span * (uint32_t)angle) / 180U);
    return us;
}

/* Convert microseconds to CCR ticks for TIM4 (using current tick rate) */
static uint32_t us_to_ccr(uint32_t us)
{
    float ticks_f = (g_tim4_tick_hz * (float)us) / 1e6f;
    if (ticks_f < 0.0f) ticks_f = 0.0f;
    uint32_t ticks = (uint32_t)(ticks_f + 0.5f);
    return ticks;
}

/* Get TIM4 clock in Hz (doubles PCLK1 if APB1 prescaler != 1) */
static uint32_t TIM4_Get_Clock_Hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t cfgr  = RCC->CFGR;
    uint32_t ppre1 = (cfgr & RCC_CFGR_PPRE1);
    bool ppre1_div1 = (ppre1 == RCC_CFGR_PPRE1_DIV1);
    return ppre1_div1 ? pclk1 : (pclk1 * 2U);
}

/* Configure timer base to 50 Hz using PSC fixed, compute ARR dynamically */
static void TIM4_Configure_50Hz(void)
{
    /* Compute ARR for 50 Hz given PSC */
    uint32_t tim_clk = TIM4_Get_Clock_Hz();                    /* ~48 MHz */
    g_tim4_tick_hz = (float)tim_clk / (float)(TIM4_FIXED_PSC + 1U); /* ~666,666.7 Hz */
    float period_ticks_f = g_tim4_tick_hz / 50.0f;             /* ~13333.33 */
    uint32_t arr = (uint32_t)(period_ticks_f + 0.5f);
    if (arr == 0) arr = 1;
    arr -= 1U;  /* ARR is 0-based */

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = TIM4_FIXED_PSC;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = (uint16_t)arr;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_DeInit(&htim4);
    HAL_TIM_PWM_Init(&htim4);

    /* Configure CH1 and CH2 PWM1 with preload enabled */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0; /* set later */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

    __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_2);
}

void Servo_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* PB6/PB7 as AF Push-Pull (TIM4 CH1/CH2) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = SERVO_ROT_GPIO_PIN; HAL_GPIO_Init(SERVO_ROT_GPIO_PORT, &gpio);
    gpio.Pin = SERVO_GRIP_GPIO_PIN; HAL_GPIO_Init(SERVO_GRIP_GPIO_PORT, &gpio);

    TIM4_Configure_50Hz();

    /* Start PWM channels with default mid positions */
    uint32_t ccr_mid = us_to_ccr(1500U); /* 1.5 ms neutral */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr_mid);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr_mid);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void Servo_Set_Rotation(uint8_t angle_deg)
{
    uint32_t us  = angle_to_us(angle_deg);
    uint32_t ccr = us_to_ccr(us);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

void Servo_Set_Gripper(uint8_t angle_deg)
{
    uint32_t us  = angle_to_us(angle_deg);
    uint32_t ccr = us_to_ccr(us);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr);
}
