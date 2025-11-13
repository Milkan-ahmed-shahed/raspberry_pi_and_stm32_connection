#ifndef MOTION_EXECUTOR_H
#define MOTION_EXECUTOR_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include <stdbool.h>
#include <stdint.h>

/* Pin Map (Blue Pill)
 * X Axis: STEP PA0 (TIM2_CH1), DIR PA1, ENA PA2
 * Y Axis: STEP PA3 (TIM2_CH4), DIR PA4, ENA PA5
 */
#define X_STEP_GPIO_PORT      GPIOA
#define X_STEP_GPIO_PIN       GPIO_PIN_0
#define X_DIR_GPIO_PORT       GPIOA
#define X_DIR_GPIO_PIN        GPIO_PIN_1
#define X_ENA_GPIO_PORT       GPIOA
#define X_ENA_GPIO_PIN        GPIO_PIN_2

#define Y_STEP_GPIO_PORT      GPIOA
#define Y_STEP_GPIO_PIN       GPIO_PIN_3
#define Y_DIR_GPIO_PORT       GPIOA
#define Y_DIR_GPIO_PIN        GPIO_PIN_4
#define Y_ENA_GPIO_PORT       GPIOA
#define Y_ENA_GPIO_PIN        GPIO_PIN_5

/* Mechanics / profile */
#define XY_STEPS_PER_MM               80U
#define MAX_VELOCITY_MM_MIN           5000U          /* global cap (mm/min) */

/* Timer base:
 * For 48 MHz TIM2 clock, PSC=71 => tick = 48e6 / 72 = 666666.7 Hz
 */
#define TIM2_FIXED_PSC                71U

/* Pre-calculated trapezoid buffer capacity (ARR values for accel/decel) */
#ifndef RAMP_BUFFER_SIZE
#define RAMP_BUFFER_SIZE              512U           /* tune per SRAM budget */
#endif

/* Ramp state machine (master axis only) */
typedef enum {
    RAMP_IDLE = 0,
    RAMP_UP,
    RAMP_CONST,
    RAMP_DOWN
} RampState;

/* Unified gantry motion state (no floating point fields) */
typedef struct {
    volatile bool     busy;

    /* Total steps for each axis (absolute distances) */
    volatile uint32_t x_total_steps;
    volatile uint32_t y_total_steps;

    /* Direction flags */
    volatile bool     x_dir_positive;
    volatile bool     y_dir_positive;

    /* Master/slave selection */
    volatile bool     master_is_x;          /* true => X drives timing; false => Y drives */
    volatile uint32_t master_total_steps;   /* convenience copy */
    volatile uint32_t slave_total_steps;    /* convenience copy */

    /* Execution counters */
    volatile uint32_t steps_done_master;
    volatile uint32_t steps_done_slave;

    /* Ramping boundaries (in master steps) */
    volatile uint32_t steps_to_accel;       /* length of acceleration segment (master) */
    volatile uint32_t steps_to_decel;       /* first master step index where decel starts */
    volatile RampState ramp_state;

    /* Timing data (all integer) */
    volatile uint16_t vmax_arr;             /* ARR value for constant Vmax */
    volatile uint16_t current_arr;          /* current ARR (period -1) */

    /* Pre-calculated trapezoid (ARR per step) for accel; decel is mirrored.
       ramp_len == steps_to_accel, capped to buffer size. */
    volatile uint16_t ramp_buffer[RAMP_BUFFER_SIZE];
    volatile uint32_t ramp_len;

    /* Bresenham-style accumulator for slave axis (integer only).
       Each master step: acc += slave_total_steps; if acc >= 0 -> enable slave channel,
       then acc -= master_total_steps. */
    volatile int32_t  y_accumulator;

} GantryState;

/* API */
void Motion_InitXY(void);
bool Move_Gantry_XY(int32_t x_mm, int32_t y_mm, uint32_t speed_mm_min);
bool Move_X(int32_t x_mm, uint32_t speed_mm_min);
bool Motion_XY_IsBusy(void);
void Motion_XY_Stop(void);
const GantryState* Motion_XY_GetState(void);

/* IRQ trampoline (called from TIM2_IRQHandler in stm32f1xx_it.c) */
void Motion_TIM2_IRQHandler(void);

#endif /* MOTION_EXECUTOR_H */
