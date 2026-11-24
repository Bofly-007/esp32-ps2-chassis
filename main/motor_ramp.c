#include "motor_ramp.h"

static int direction(int value)
{
    return (value > 0) - (value < 0);
}

static int approach(int value, int target)
{
    if (value < target) {
        const int next = value + MOTOR_RAMP_STEP_PERCENT;
        return next > target ? target : next;
    }
    if (value > target) {
        const int next = value - MOTOR_RAMP_STEP_PERCENT;
        return next < target ? target : next;
    }
    return value;
}

void motor_ramp_init(motor_ramp_t *state, uint64_t now_ms)
{
    *state = (motor_ramp_t) {
        .zero_since_ms = now_ms,
        .last_command_ms = now_ms,
        .command_timeout_ms = MOTOR_COMMAND_TIMEOUT_MS,
    };
}

bool motor_ramp_set_target(motor_ramp_t *state, int percent, uint64_t now_ms)
{
    return motor_ramp_set_target_timed(state, percent, now_ms, MOTOR_COMMAND_TIMEOUT_MS);
}

bool motor_ramp_set_target_timed(motor_ramp_t *state, int percent, uint64_t now_ms,
                                  uint32_t timeout_ms)
{
    if (percent < -MOTOR_MAX_DUTY_PERCENT || percent > MOTOR_MAX_DUTY_PERCENT ||
        timeout_ms == 0) {
        return false;
    }
    state->target_percent = percent;
    state->last_command_ms = now_ms;
    state->command_timeout_ms = timeout_ms;
    return true;
}

void motor_ramp_stop(motor_ramp_t *state, uint64_t now_ms)
{
    if (state->output_percent != 0) {
        state->zero_since_ms = now_ms;
    }
    state->target_percent = 0;
    state->output_percent = 0;
    state->last_command_ms = now_ms;
    // Keep the previous direction and zero timestamp across stop/start commands.
}

bool motor_ramp_step(motor_ramp_t *state, uint64_t now_ms)
{
    if ((state->target_percent != 0 || state->output_percent != 0) &&
        now_ms - state->last_command_ms >= state->command_timeout_ms) {
        motor_ramp_stop(state, now_ms);
        return true;
    }

    const int old_output = state->output_percent;
    const int target_direction = direction(state->target_percent);
    int ramp_target = state->target_percent;

    // Opposite commands first ramp down to zero, never across zero in one step.
    if (old_output != 0 && target_direction != 0 &&
        direction(old_output) != target_direction) {
        ramp_target = 0;
    }

    if (old_output == 0 && target_direction != 0 &&
        state->last_direction != 0 && state->last_direction != target_direction &&
        now_ms - state->zero_since_ms < MOTOR_REVERSE_WAIT_MS) {
        return false;
    }

    state->output_percent = approach(old_output, ramp_target);
    if (old_output != 0 && state->output_percent == 0) {
        state->zero_since_ms = now_ms;
    }
    if (state->output_percent != 0) {
        state->last_direction = direction(state->output_percent);
    }
    return false;
}
