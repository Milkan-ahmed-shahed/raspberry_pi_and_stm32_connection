#ifndef MOTION_EXECUTOR_Z_H
#define MOTION_EXECUTOR_Z_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Z Axis Pin Map (Blue Pill)
 * STEP: PB0 (TIM3_CH3)
 * DIR : PB1 (GPIO Output)
 * ENA : PB2 (GPIO Output, assumed active LOW; invert if needed)
 */
#define Z_STEP_GPIO_PORT        GPIOB
#define Z_STEP_GPIO_PIN         GPIO_PIN_0   /* TIM3_CH3 */
#define Z_DIR_GPIO_PORT         GPIOB
#define Z_DIR_GPIO_PIN          GPIO_PIN_1
#define Z_ENA_GPIO_PORT         GPIOB
#define Z_ENA_GPIO_PIN          GPIO_PIN_2

/* Motion parameters for Z (T8 lead screw, 8 mm lead, 1/16 microstepping) */
#define Z_STEPS_PER_MM          400U
#define Z_MAX_VELOCITY_MM_MIN   5000U
#define Z_ACCELERATION_MM_S2    500.0f

/* Timer base configuration:
 * TIM3 clock = 48 MHz (APB1 prescaler /2 => timer clock doubled).
 */
#define TIM3_FIXED_PSC          71U          /* 48 MHz / (71+1) = ~666666.7 Hz */

#ifndef Z_RAMP_BUFFER_SIZE
#define Z_RAMP_BUFFER_SIZE      512U
#endif

typedef enum {
    Z_RAMP_IDLE = 0,
    Z_RAMP_UP,
    Z_RAMP_CONST,
    Z_RAMP_DOWN
} Z_RampState;

typedef struct {
    volatile bool     busy;
    volatile bool     dir_positive;

    volatile uint32_t total_steps;
    volatile uint32_t steps_done;
    volatile uint32_t steps_to_accel;
    volatile uint32_t steps_to_decel;
    volatile Z_RampState ramp_state;

    volatile uint16_t vmax_arr;
    volatile uint16_t current_arr;

    volatile uint16_t ramp_buffer[Z_RAMP_BUFFER_SIZE];
    volatile uint32_t ramp_len;
} Z_MotionState;

/* API */
void Motion_InitZ(void);
bool Move_Z(int32_t z_mm, uint32_t speed_mm_min);
bool Motion_Z_IsBusy(void);
void Motion_Z_Stop(void);
const Z_MotionState* Motion_Z_GetState(void);

/* IRQ trampoline (called from TIM3_IRQHandler) */
void Motion_TIM3_IRQHandler(void);

#endif /* MOTION_EXECUTOR_Z_H */
