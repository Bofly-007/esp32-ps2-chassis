#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PS2_POLL_PERIOD_MS 20
#define PS2_COMMAND_TIMEOUT_MS 200
#define PS2_AXIS_CENTER 128
#define PS2_AXIS_DEADZONE 12
#define PS2_FRAME_MAX_BYTES 21
#define PS2_BUTTON_L1 (1U << 10)
#define PS2_BUTTON_CROSS (1U << 14)

typedef struct {
    uint8_t mode;
    uint16_t buttons; // Active high after decoding; wire protocol is active low.
    uint8_t right_x;
    uint8_t right_y;
    uint8_t left_x;
    uint8_t left_y;
} ps2_sample_t;

typedef enum { INPUT_PS2, INPUT_SERIAL } input_mode_t;
typedef enum { PS2_KEEP_OUTPUT, PS2_STOP_OUTPUT, PS2_SET_TARGET } ps2_action_t;

typedef struct {
    int left;
    int right;
} ps2_wheel_targets_t;

typedef struct {
    input_mode_t mode;
    bool connected; // Recent valid receiver frames, not proof of the radio link.
    bool ready;
    uint64_t last_valid_ms;
    ps2_sample_t sample;
} ps2_control_t;

// Only complete 0x73/0x79 analog frames with the expected headers are accepted.
bool ps2_decode_frame(const uint8_t *data, size_t size, ps2_sample_t *sample);
int ps2_y_to_percent(uint8_t value);
void ps2_control_init(ps2_control_t *state);
void ps2_control_set_mode(ps2_control_t *state, input_mode_t mode);
void ps2_control_stop(ps2_control_t *state);
// NULL means no valid frame this poll. STOP disables both bridges immediately.
ps2_action_t ps2_control_step(ps2_control_t *state, const ps2_sample_t *sample,
                              uint64_t now_ms, ps2_wheel_targets_t *targets);
