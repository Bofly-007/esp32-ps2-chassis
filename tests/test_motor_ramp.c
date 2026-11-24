#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_ramp.h"

static uint64_t step(motor_ramp_t *motor, uint64_t now, unsigned count)
{
    while (count-- != 0) {
        now += MOTOR_CONTROL_PERIOD_MS;
        assert(!motor_ramp_step(motor, now));
    }
    return now;
}

static void test_startup_limits_and_ramp(void)
{
    motor_ramp_t m;
    motor_ramp_init(&m, 0);
    assert(m.output_percent == 0 && m.target_percent == 0);
    assert(!motor_ramp_set_target(&m, 101, 10));
    assert(!motor_ramp_set_target(&m, -101, 10));
    assert(!motor_ramp_set_target(&m, INT_MIN, 10));
    assert(!motor_ramp_set_target(&m, INT_MAX, 10));
    assert(m.last_command_ms == 0 && m.output_percent == 0);
    assert(motor_ramp_set_target(&m, 100, 0));
    for (int i = 1; i <= 100; ++i) {
        step(&m, (uint64_t)(i - 1) * 20, 1);
        assert(m.output_percent == i);
    }
    step(&m, 2000, 10);
    assert(m.output_percent == 100);
    assert(motor_ramp_set_target(&m, 0, 2200));
    step(&m, 2200, 100);
    assert(m.output_percent == 0 && m.target_percent == 0);
}

static void test_full_scale_reversal(void)
{
    motor_ramp_t m;
    motor_ramp_init(&m, 0);
    assert(motor_ramp_set_target(&m, 100, 0));
    uint64_t now = step(&m, 0, 100);
    assert(m.output_percent == 100);
    assert(motor_ramp_set_target(&m, -100, now));
    now = step(&m, now, 100);
    assert(m.output_percent == 0);
    now = step(&m, now, 24);
    assert(m.output_percent == 0);
    now = step(&m, now, 100);
    assert(m.output_percent == -100);
    motor_ramp_stop(&m, now);
    assert(m.output_percent == 0 && m.target_percent == 0);
}

static void test_reversal_wait_and_stop_cannot_bypass(void)
{
    motor_ramp_t m;
    motor_ramp_init(&m, 0);
    assert(motor_ramp_set_target(&m, 10, 0));
    uint64_t now = step(&m, 0, 10);
    assert(motor_ramp_set_target(&m, -10, now));
    now = step(&m, now, 10);
    assert(m.output_percent == 0);
    const uint64_t zero_time = now;
    now = step(&m, now, 24);
    assert(m.output_percent == 0);
    now = step(&m, now, 1);
    assert(now - zero_time == 500 && m.output_percent == -1);

    motor_ramp_stop(&m, now);
    const uint64_t stopped = now;
    assert(m.output_percent == 0 && m.target_percent == 0);
    assert(motor_ramp_set_target(&m, 10, now));
    now = step(&m, now, 10);
    motor_ramp_stop(&m, now);
    assert(m.zero_since_ms == stopped);
    assert(motor_ramp_set_target(&m, 10, now));
    now = step(&m, now, 14);
    assert(m.output_percent == 0);
    step(&m, now, 1);
    assert(m.output_percent == 1);
}

static void test_timeout_and_rejected_command(void)
{
    motor_ramp_t m;
    motor_ramp_init(&m, 0);
    assert(motor_ramp_set_target(&m, 20, 0));
    step(&m, 0, 20);
    assert(!motor_ramp_step(&m, 9999));
    assert(!motor_ramp_set_target(&m, 101, 9999));
    assert(motor_ramp_step(&m, 10000));
    assert(m.output_percent == 0 && m.target_percent == 0);
    assert(!motor_ramp_step(&m, 10020));

    assert(motor_ramp_set_target(&m, -20, 10020));
    step(&m, 10020, 30);
    assert(motor_ramp_set_target(&m, -20, 19000));
    assert(!motor_ramp_step(&m, 28999));
    assert(motor_ramp_step(&m, 29000));
    assert(m.output_percent == 0);
}

static void test_command_changes_during_reversal(void)
{
    motor_ramp_t m;
    motor_ramp_init(&m, 0);
    assert(motor_ramp_set_target(&m, 10, 0));
    uint64_t now = step(&m, 0, 10);
    assert(motor_ramp_set_target(&m, -10, now));
    now = step(&m, now, 5);
    assert(m.output_percent == 5);
    assert(motor_ramp_set_target(&m, 10, now));
    now = step(&m, now, 5);
    assert(m.output_percent == 10);
    assert(motor_ramp_set_target(&m, -10, now));
    now = step(&m, now, 10);
    assert(motor_ramp_set_target(&m, 0, now));
    step(&m, now, 40);
    assert(m.output_percent == 0 && m.target_percent == 0);
}

static void test_many_command_sequences(void)
{
    motor_ramp_t m;
    motor_ramp_init(&m, 0);
    unsigned random = 12345;
    int previous_direction = 0;
    uint64_t zero_time = 0;
    for (uint64_t now = 20; now <= 200000; now += 20) {
        random = random * 1664525U + 1013904223U;
        const int before = m.output_percent;
        bool immediate_stop = false;
        if (random % 19 == 0) {
            motor_ramp_stop(&m, now);
            immediate_stop = true;
        } else if (random % 7 == 0) {
            assert(motor_ramp_set_target(&m, (int)(random % 201) - 100, now));
        }
        const bool timed_out = motor_ramp_step(&m, now);
        const int after = m.output_percent;
        assert(abs(after) <= 100);
        if (!immediate_stop && !timed_out) {
            assert(abs(after - before) <= 1);
        }
        if (before != 0 && after == 0) {
            zero_time = now;
        }
        if (after != 0) {
            const int direction = after > 0 ? 1 : -1;
            if (previous_direction != 0 && direction != previous_direction) {
                assert(before == 0 && now - zero_time >= 500);
            }
            previous_direction = direction;
        }
    }
}

int main(void)
{
    test_startup_limits_and_ramp();
    test_full_scale_reversal();
    test_reversal_wait_and_stop_cannot_bypass();
    test_timeout_and_rejected_command();
    test_command_changes_during_reversal();
    test_many_command_sequences();
    puts("PASS: +/-100% limits, full-scale ramp/reversal, stop, timeout, 10000 mixed steps");
    return 0;
}
