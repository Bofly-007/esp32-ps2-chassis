#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_ramp.h"
#include "ps2_control.h"

static ps2_sample_t sample(uint8_t y, uint16_t buttons)
{
    return (ps2_sample_t) { .mode = 0x73, .left_x = 128, .left_y = y, .buttons = buttons };
}

static ps2_action_t feed(ps2_control_t *state, uint8_t y, uint16_t buttons,
                          uint64_t now, ps2_wheel_targets_t *target)
{
    const ps2_sample_t frame = sample(y, buttons);
    return ps2_control_step(state, &frame, now, target);
}

static void test_frames(void)
{
    uint8_t data[21] = {0xff, 0x73, 0x5a, 0xff, 0xbb, 10, 20, 30, 40};
    ps2_sample_t parsed;
    assert(ps2_decode_frame(data, 9, &parsed));
    assert(parsed.mode == 0x73 && parsed.left_y == 40 && parsed.left_x == 30);
    assert(parsed.right_x == 10 && parsed.right_y == 20);
    assert(parsed.buttons == (PS2_BUTTON_L1 | PS2_BUTTON_CROSS));
    assert(!ps2_decode_frame(data, 8, &parsed));
    assert(!ps2_decode_frame(data, 21, &parsed));
    data[1] = 0x79;
    assert(!ps2_decode_frame(data, 9, &parsed));
    assert(ps2_decode_frame(data, 21, &parsed));
    assert(parsed.mode == 0x79);
    data[1] = 0x41; // Digital packets must never supply fabricated analog axes.
    assert(!ps2_decode_frame(data, 5, &parsed));
    data[1] = 0xf3; // Configuration reply is not a control packet.
    assert(!ps2_decode_frame(data, 9, &parsed));
    data[1] = 0x73;
    data[2] = 0;
    assert(!ps2_decode_frame(data, 9, &parsed));
    data[2] = 0x5a;
    data[0] = 0;
    assert(!ps2_decode_frame(data, 9, &parsed));
    memset(data, 0xff, sizeof(data));
    assert(!ps2_decode_frame(data, 9, &parsed));
    memset(data, 0, sizeof(data));
    assert(!ps2_decode_frame(data, 9, &parsed));
    assert(!ps2_decode_frame(NULL, 9, &parsed));
    assert(!ps2_decode_frame(data, 0, &parsed));
    assert(!ps2_decode_frame(data, 9, NULL));
}

static void test_axis(void)
{
    assert(ps2_y_to_percent(0) == 100);
    assert(ps2_y_to_percent(255) == -100);
    assert(ps2_y_to_percent(115) == 1);
    assert(ps2_y_to_percent(141) == -1);
    int previous = 100;
    for (unsigned y = 0; y < 256; ++y) {
        const int percent = ps2_y_to_percent((uint8_t)y);
        assert(percent >= -100 && percent <= 100 && percent <= previous);
        if (y >= 116 && y <= 140) {
            assert(percent == 0);
        } else {
            assert(percent != 0);
        }
        previous = percent;
    }
}

static void test_startup_l1_and_neutral(void)
{
    ps2_control_t c;
    ps2_control_init(&c);
    ps2_wheel_targets_t target = {123, 123};
    assert(c.mode == INPUT_PS2 && !c.ready && !c.connected);
    assert(ps2_control_step(&c, NULL, 0, &target) == PS2_STOP_OUTPUT && target.left == 0 && target.right == 0);
    assert(feed(&c, 0, PS2_BUTTON_L1, 20, &target) == PS2_STOP_OUTPUT);
    assert(!c.ready && c.connected);
    assert(feed(&c, 128, PS2_BUTTON_L1, 40, &target) == PS2_STOP_OUTPUT && !c.ready);
    assert(feed(&c, 128, 0, 60, &target) == PS2_STOP_OUTPUT && c.ready);
    assert(feed(&c, 0, PS2_BUTTON_L1, 80, &target) == PS2_SET_TARGET && target.left == 100 && target.right == 100);
    assert(feed(&c, 255, PS2_BUTTON_L1, 100, &target) == PS2_SET_TARGET && target.left == -100 && target.right == -100);
    assert(feed(&c, 128, PS2_BUTTON_L1, 120, &target) == PS2_STOP_OUTPUT && c.ready);
    assert(feed(&c, 0, PS2_BUTTON_L1, 140, &target) == PS2_SET_TARGET);
    assert(feed(&c, 0, 0, 160, &target) == PS2_STOP_OUTPUT && !c.ready);
    assert(feed(&c, 0, PS2_BUTTON_L1, 180, &target) == PS2_STOP_OUTPUT && !c.ready);
    assert(feed(&c, 128, 0, 200, &target) == PS2_STOP_OUTPUT && c.ready);
}

static void test_stop_latches_and_modes(void)
{
    ps2_control_t c;
    ps2_control_init(&c);
    ps2_wheel_targets_t target;
    feed(&c, 128, 0, 0, &target);
    assert(feed(&c, 0, PS2_BUTTON_L1, 20, &target) == PS2_SET_TARGET);
    ps2_control_stop(&c); // Serial 'stop' cannot be undone by a held stick.
    assert(feed(&c, 0, PS2_BUTTON_L1, 40, &target) == PS2_STOP_OUTPUT && !c.ready);
    feed(&c, 128, 0, 60, &target);
    assert(feed(&c, 0, PS2_BUTTON_L1 | PS2_BUTTON_CROSS, 80, &target) == PS2_STOP_OUTPUT);
    assert(!c.ready);
    assert(feed(&c, 0, PS2_BUTTON_L1, 100, &target) == PS2_STOP_OUTPUT);
    assert(feed(&c, 128, PS2_BUTTON_CROSS, 120, &target) == PS2_STOP_OUTPUT && !c.ready);
    feed(&c, 128, 0, 140, &target);
    assert(c.ready);

    ps2_control_set_mode(&c, INPUT_SERIAL);
    assert(!c.ready);
    assert(feed(&c, 0, PS2_BUTTON_L1, 160, &target) == PS2_KEEP_OUTPUT);
    assert(feed(&c, 0, PS2_BUTTON_CROSS, 180, &target) == PS2_KEEP_OUTPUT);
    assert(ps2_control_step(&c, NULL, 380, &target) == PS2_KEEP_OUTPUT && !c.connected);
    ps2_control_set_mode(&c, INPUT_PS2);
    assert(feed(&c, 0, PS2_BUTTON_L1, 400, &target) == PS2_STOP_OUTPUT);
    feed(&c, 128, 0, 420, &target);
    assert(feed(&c, 0, PS2_BUTTON_L1, 440, &target) == PS2_SET_TARGET);
}

static void test_loss_and_recovery(void)
{
    ps2_control_t c;
    ps2_control_init(&c);
    ps2_wheel_targets_t target;
    feed(&c, 128, 0, 0, &target);
    feed(&c, 0, PS2_BUTTON_L1, 20, &target);
    assert(ps2_control_step(&c, NULL, 219, &target) == PS2_KEEP_OUTPUT);
    assert(c.connected && c.last_valid_ms == 20);
    assert(ps2_control_step(&c, NULL, 220, &target) == PS2_STOP_OUTPUT);
    assert(!c.connected && !c.ready);
    assert(feed(&c, 0, PS2_BUTTON_L1, 240, &target) == PS2_STOP_OUTPUT);
    assert(c.connected && !c.ready);
    feed(&c, 128, 0, 260, &target);
    assert(feed(&c, 0, PS2_BUTTON_L1, 280, &target) == PS2_SET_TARGET);
    // A fresh valid frame AFTER a stalled reader must also require neutral.
    assert(feed(&c, 0, PS2_BUTTON_L1, 480, &target) == PS2_STOP_OUTPUT && !c.ready);
    feed(&c, 128, 0, 500, &target);
    // The protocol has no sequence number: identical valid frames stay valid.
    for (uint64_t now = 520; now <= 2520; now += 20) {
        assert(feed(&c, 0, PS2_BUTTON_L1, now, &target) == PS2_SET_TARGET);
    }
}

static void test_differential_drive(void)
{
    ps2_control_t c;
    ps2_control_init(&c);
    ps2_wheel_targets_t target;
    ps2_sample_t frame = sample(128, 0);
    frame.left_x = 255;
    // Centering only Y must not let an already deflected steering axis arm.
    assert(ps2_control_step(&c, &frame, 0, &target) == PS2_STOP_OUTPUT && !c.ready);
    feed(&c, 128, 0, 20, &target);
    assert(c.ready);
    frame.buttons = PS2_BUTTON_L1;
    assert(ps2_control_step(&c, &frame, 40, &target) == PS2_SET_TARGET);
    assert(target.left == 100 && target.right == -100);
    frame.left_x = 0;
    assert(ps2_control_step(&c, &frame, 60, &target) == PS2_SET_TARGET);
    assert(target.left == -100 && target.right == 100);
    frame.left_x = 197;
    frame.left_y = 0;
    assert(ps2_control_step(&c, &frame, 80, &target) == PS2_SET_TARGET);
    assert(target.left == 100 && target.right == 33);
    frame.left_x = 255;
    assert(ps2_control_step(&c, &frame, 100, &target) == PS2_SET_TARGET);
    assert(target.left == 100 && target.right == 0);
    frame.left_y = 255;
    assert(ps2_control_step(&c, &frame, 120, &target) == PS2_SET_TARGET);
    assert(target.left == 0 && target.right == -100);
    frame.left_y = 128;
    frame.buttons = 0;
    assert(ps2_control_step(&c, &frame, 140, &target) == PS2_STOP_OUTPUT && !c.ready);
    assert(target.left == 0 && target.right == 0);
    frame.buttons = PS2_BUTTON_L1;
    assert(ps2_control_step(&c, &frame, 160, &target) == PS2_STOP_OUTPUT && !c.ready);
}

static void test_independent_motor_lease(void)
{
    motor_ramp_t motor;
    motor_ramp_init(&motor, 0);
    assert(motor_ramp_set_target_timed(&motor, 100, 0, PS2_COMMAND_TIMEOUT_MS));
    for (uint64_t now = 20; now < PS2_COMMAND_TIMEOUT_MS; now += 20) {
        assert(!motor_ramp_step(&motor, now));
        assert(motor.output_percent > 0);
    }
    assert(!motor_ramp_set_target_timed(&motor, 100, 199, 0));
    assert(!motor_ramp_set_target_timed(&motor, 101, 199, 200));
    assert(motor.last_command_ms == 0 && motor.command_timeout_ms == 200);
    // No PS2 task or stop call here: motor logic itself expires the lease.
    assert(motor_ramp_step(&motor, 200));
    assert(motor.target_percent == 0 && motor.output_percent == 0);

    assert(motor_ramp_set_target_timed(&motor, 100, 220, 200));
    assert(!motor_ramp_step(&motor, 400));
    assert(motor_ramp_set_target_timed(&motor, 100, 400, 200));
    assert(!motor_ramp_step(&motor, 599));
    assert(motor_ramp_step(&motor, 600));
    assert(motor_ramp_set_target(&motor, 100, 620));
    assert(motor.command_timeout_ms == MOTOR_COMMAND_TIMEOUT_MS);
    assert(!motor_ramp_step(&motor, 10619));
    assert(motor_ramp_step(&motor, 10620));
}

int main(void)
{
    test_frames();
    test_axis();
    test_startup_l1_and_neutral();
    test_stop_latches_and_modes();
    test_loss_and_recovery();
    test_differential_drive();
    test_independent_motor_lease();
    puts("PASS: PS2 frames, differential drive, neutral/L1/stop/mode gates, loss/recovery, motor lease");
    return 0;
}
