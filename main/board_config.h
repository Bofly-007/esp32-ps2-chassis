#pragma once

#include "driver/gpio.h"

// Each BTS7960 has two PWM signals and two separate enable wires.
#define LEFT_RPWM_GPIO GPIO_NUM_25
#define LEFT_LPWM_GPIO GPIO_NUM_26
#define LEFT_R_EN_GPIO GPIO_NUM_32
#define LEFT_L_EN_GPIO GPIO_NUM_33
#define RIGHT_RPWM_GPIO GPIO_NUM_16
#define RIGHT_LPWM_GPIO GPIO_NUM_17
#define RIGHT_R_EN_GPIO GPIO_NUM_22
#define RIGHT_L_EN_GPIO GPIO_NUM_27
#define LEFT_MOTOR_REVERSED 0
#define RIGHT_MOTOR_REVERSED 1  // Initial setting for mirror-mounted wheels.
#define MOTOR_PWM_FREQUENCY_HZ 20000

// PS2 receiver wiring confirmed by the user. VCC=3V3; share GND with ESP32.
#define PS2_DAT_GPIO GPIO_NUM_19
#define PS2_CMD_GPIO GPIO_NUM_23
#define PS2_CS_GPIO GPIO_NUM_21
#define PS2_CLK_GPIO GPIO_NUM_18
