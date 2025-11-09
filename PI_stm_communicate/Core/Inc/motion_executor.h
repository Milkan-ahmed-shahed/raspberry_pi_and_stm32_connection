#ifndef MOTION_EXECUTOR_H
#define MOTION_EXECUTOR_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include <stdbool.h>
#include <stdint.h>

// Pin map for X-axis
#define X_STEP_GPIO_PORT   GPIOA
#define X_STEP_GPIO_PIN    GPIO_PIN_0     // TIM2_CH1 (PA0)
#define X_DIR_GPIO_PORT    GPIOA
#define X_DIR_GPIO_PIN     GPIO_PIN_1     // DIR output (GPIO)

// Motion parameters
#define X_STEPS_PER_MM     80U            // GT2 belt, 20T pulley -> 80 steps/mm (with configured microstepping)

// Public X-axis state
typedef struct {
    volatile bool     busy;
    volatile uint32_t target_steps;
    volatile uint32_t steps_done;
    volatile bool     dir_positive;       // true if moving +X, false if -X
} MotionXState;

// Initialize GPIO and TIM2 for X-axis motion (does not start motion)
void Motion_InitX(void);

// Start a non-blocking move on X axis.
// - target_mm can be positive (forward) or negative (reverse).
// - speed_mm_min is the linear speed in mm/min (must be > 0).
// Returns true if the move was started, false if busy or invalid params.
bool Move_X(int32_t target_mm, uint32_t speed_mm_min);

// Query if X-axis is currently moving
bool Motion_X_IsBusy(void);

// Immediately stop X motion (best-effort stop at next interrupt boundary)
void Motion_X_Stop(void);

// Accessor for state (read-only)
const MotionXState* Motion_X_GetState(void);

#endif // MOTION_EXECUTOR_H
