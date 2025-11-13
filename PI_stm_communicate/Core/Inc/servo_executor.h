#ifndef SERVO_EXECUTOR_H
#define SERVO_EXECUTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * 2x 50 Hz servos on TIM4:
 *  - PB6: TIM4_CH1 => Claw Rotation
 *  - PB7: TIM4_CH2 => Gripper
 *
 * PWM period = 20 ms (50 Hz). Pulse width ~ 1.0 ms (0 deg) to 2.0 ms (180 deg).
 */

#define SERVO_ROT_GPIO_PORT    GPIOB
#define SERVO_ROT_GPIO_PIN     GPIO_PIN_6   /* TIM4_CH1 */
#define SERVO_GRIP_GPIO_PORT   GPIOB
#define SERVO_GRIP_GPIO_PIN    GPIO_PIN_7   /* TIM4_CH2 */

/* Timer base (TIM4 on APB1), same PSC as other timers for high resolution.
 * APB1 prescaler is 2 => TIM4 clock = 2 * PCLK1 = 48 MHz (with SYSCLK=48 MHz)
 * PSC=71 -> tick = 48 MHz / 72 ~= 666,666.7 Hz
 * 50 Hz period ticks ~= 666,666.7 / 50 = 13,333.33 -> ARR ~= 13333 (0-based)
 */
#define TIM4_FIXED_PSC         71U

/* Servo pulse limits (microseconds) */
#define SERVO_MIN_US           1000U
#define SERVO_MAX_US           2000U

void Servo_Init(void);
void Servo_Set_Rotation(uint8_t angle_deg);
void Servo_Set_Gripper(uint8_t angle_deg);

#endif /* SERVO_EXECUTOR_H */
