#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MAX_DUTY_PERCENT 100
#define MOTOR_CONTROL_PERIOD_MS 20
#define MOTOR_RAMP_STEP_PERCENT 1
#define MOTOR_REVERSE_WAIT_MS 500
#define MOTOR_COMMAND_TIMEOUT_MS 10000

// All times use the same monotonic clock. Call step once per control period.
typedef struct {
    int target_percent;
    int output_percent;
    int last_direction;
    uint64_t zero_since_ms;
    uint64_t last_command_ms;
    uint32_t command_timeout_ms;
} motor_ramp_t;

void motor_ramp_init(motor_ramp_t *state, uint64_t now_ms);
bool motor_ramp_set_target(motor_ramp_t *state, int percent, uint64_t now_ms);
bool motor_ramp_set_target_timed(motor_ramp_t *state, int percent, uint64_t now_ms,
                                  uint32_t timeout_ms);
void motor_ramp_stop(motor_ramp_t *state, uint64_t now_ms);
// Returns true only when an active command expires and the output is disabled.
bool motor_ramp_step(motor_ramp_t *state, uint64_t now_ms);
