#include "ps2_control.h"

#include <stdlib.h>

#include "motor_ramp.h"

bool ps2_decode_frame(const uint8_t *data, size_t size, ps2_sample_t *sample)
{
    if (data == NULL || sample == NULL || size < 3 ||
        data[0] != 0xff || data[2] != 0x5a) {
        return false;
    }
    const size_t expected = data[1] == 0x73 ? 9 : data[1] == 0x79 ? 21 : 0;
    if (expected == 0 || size != expected) {
        return false;
    }
    *sample = (ps2_sample_t) {
        .mode = data[1],
        .buttons = (uint16_t)~((uint16_t)data[3] | ((uint16_t)data[4] << 8)),
        .right_x = data[5], .right_y = data[6],
        .left_x = data[7], .left_y = data[8],
    };
    return true;
}

int ps2_y_to_percent(uint8_t value)
{
    const int low = PS2_AXIS_CENTER - PS2_AXIS_DEADZONE;
    const int high = PS2_AXIS_CENTER + PS2_AXIS_DEADZONE;
    // Round away from zero so the first value outside the dead zone gives 1%.
    if (value < low) {
        return ((low - value) * MOTOR_MAX_DUTY_PERCENT + low - 1) / low;
    }
    if (value > high) {
        const int span = 255 - high;
        return -(((value - high) * MOTOR_MAX_DUTY_PERCENT + span - 1) / span);
    }
    return 0;
}

void ps2_control_init(ps2_control_t *state)
{
    *state = (ps2_control_t) { .mode = INPUT_PS2 };
}

void ps2_control_set_mode(ps2_control_t *state, input_mode_t mode)
{
    state->mode = mode;
    state->ready = false;
}

void ps2_control_stop(ps2_control_t *state)
{
    state->ready = false;
}

ps2_action_t ps2_control_step(ps2_control_t *state, const ps2_sample_t *sample,
                              uint64_t now_ms, ps2_wheel_targets_t *targets)
{
    *targets = (ps2_wheel_targets_t) {0};
    // Check the gap BEFORE accepting a recovered frame: never resume a stale
    // deflected-stick command after the reader task has stalled.
    if (state->connected && now_ms - state->last_valid_ms >= PS2_COMMAND_TIMEOUT_MS) {
        state->connected = false;
        state->ready = false;
    }
    if (sample != NULL) {
        state->sample = *sample;
        state->connected = true;
        state->last_valid_ms = now_ms;
    }
    if (state->mode == INPUT_SERIAL) {
        state->ready = false;
        return PS2_KEEP_OUTPUT;
    }
    if (!state->connected) {
        state->ready = false;
        return PS2_STOP_OUTPUT;
    }
    if (sample == NULL) {
        return PS2_KEEP_OUTPUT;
    }
    if (sample->buttons & PS2_BUTTON_CROSS) {
        state->ready = false;
        return PS2_STOP_OUTPUT;
    }
    const int throttle = ps2_y_to_percent(sample->left_y);
    const int turn = -ps2_y_to_percent(sample->left_x); // Right is positive.
    const bool neutral = throttle == 0 && turn == 0;
    const bool held = (sample->buttons & PS2_BUTTON_L1) != 0;
    if (!state->ready) {
        if (neutral && !held) {
            state->ready = true;
        }
        return PS2_STOP_OUTPUT;
    }
    if (!held) {
        // Releasing L1 while deflected requires another neutral/released frame.
        if (!neutral) {
            state->ready = false;
        }
        return PS2_STOP_OUTPUT;
    }
    if (neutral) {
        return PS2_STOP_OUTPUT;
    }
    const int left = throttle + turn;
    const int right = throttle - turn;
    const int peak = abs(left) > abs(right) ? abs(left) : abs(right);
    // Scale both together to preserve the turn ratio at diagonal extremes.
    const int scale = peak > MOTOR_MAX_DUTY_PERCENT ? peak : MOTOR_MAX_DUTY_PERCENT;
    targets->left = left * MOTOR_MAX_DUTY_PERCENT / scale;
    targets->right = right * MOTOR_MAX_DUTY_PERCENT / scale;
    return PS2_SET_TARGET;
}
