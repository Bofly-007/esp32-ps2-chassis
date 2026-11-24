#include "bts7960.h"

#include "board_config.h"
#include "motor_ramp.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "bts7960";
static SemaphoreHandle_t s_lock;
static bool s_ready;
static bool s_fault;

#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_RESOLUTION_BITS LEDC_TIMER_10_BIT

typedef struct {
    gpio_num_t rpwm_gpio, lpwm_gpio, r_en_gpio, l_en_gpio;
    ledc_channel_t rpwm_channel, lpwm_channel;
    bool reversed;
} wheel_config_t;

enum { LEFT_WHEEL, RIGHT_WHEEL, WHEEL_COUNT };
static const wheel_config_t s_wheels[WHEEL_COUNT] = {
    [LEFT_WHEEL] = {
        LEFT_RPWM_GPIO, LEFT_LPWM_GPIO, LEFT_R_EN_GPIO, LEFT_L_EN_GPIO,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEFT_MOTOR_REVERSED,
    },
    [RIGHT_WHEEL] = {
        RIGHT_RPWM_GPIO, RIGHT_LPWM_GPIO, RIGHT_R_EN_GPIO, RIGHT_L_EN_GPIO,
        LEDC_CHANNEL_2, LEDC_CHANNEL_3, RIGHT_MOTOR_REVERSED,
    },
};
static motor_ramp_t s_ramps[WHEEL_COUNT];

static uint64_t now_ms(void)
{
    return (uint64_t)esp_timer_get_time() / 1000;
}

static esp_err_t set_pwm(ledc_channel_t channel, uint32_t duty)
{
    // All callers hold s_lock; no concurrent LEDC updates occur.
    esp_err_t err = ledc_set_duty(PWM_MODE, channel, duty);
    return err == ESP_OK ? ledc_update_duty(PWM_MODE, channel) : err;
}

static esp_err_t set_enable(const wheel_config_t *wheel, bool enabled)
{
    // Always write both pins, including when the first write fails.
    const esp_err_t right_err = gpio_set_level(wheel->r_en_gpio, enabled);
    const esp_err_t left_err = gpio_set_level(wheel->l_en_gpio, enabled);
    if (enabled && (right_err != ESP_OK || left_err != ESP_OK)) {
        gpio_set_level(wheel->r_en_gpio, 0);
        gpio_set_level(wheel->l_en_gpio, 0);
    }
    return right_err != ESP_OK ? right_err : left_err;
}

static esp_err_t clear_pwm(const wheel_config_t *wheel)
{
    const esp_err_t r_err = set_pwm(wheel->rpwm_channel, 0);
    const esp_err_t l_err = set_pwm(wheel->lpwm_channel, 0);
    return r_err != ESP_OK ? r_err : l_err;
}

static esp_err_t apply_output(const wheel_config_t *wheel, int percent)
{
    if (percent == 0) {
        // EN low first: stopping must not depend on the PWM update succeeding.
        const esp_err_t enable_err = set_enable(wheel, false);
        const esp_err_t pwm_err = clear_pwm(wheel);
        return enable_err != ESP_OK ? enable_err : pwm_err;
    }

    if (wheel->reversed) {
        percent = -percent;
    }
    const unsigned magnitude = (unsigned)(percent > 0 ? percent : -percent);
    // At 10-bit resolution ESP32 supports duty=1024 for a true 100% output.
    // Keep the period scale as 2^bits, rather than clipping it to 2^bits - 1.
    const uint32_t period_counts = 1U << PWM_RESOLUTION_BITS;
    const uint32_t duty = (period_counts * magnitude) / 100U;
    const ledc_channel_t inactive = percent > 0 ? wheel->lpwm_channel : wheel->rpwm_channel;
    const ledc_channel_t active = percent > 0 ? wheel->rpwm_channel : wheel->lpwm_channel;
    ESP_RETURN_ON_ERROR(set_pwm(inactive, 0), TAG, "clear inactive PWM");
    ESP_RETURN_ON_ERROR(set_pwm(active, duty), TAG, "set active PWM");
    return set_enable(wheel, true);
}

// Drop all enables before updating PWM on either bridge.
static esp_err_t stop_both(uint64_t now)
{
    esp_err_t result = ESP_OK;
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const esp_err_t err = set_enable(&s_wheels[i], false);
        if (err != ESP_OK) {
            result = err;
        }
        motor_ramp_stop(&s_ramps[i], now);
    }
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const esp_err_t err = clear_pwm(&s_wheels[i]);
        if (err != ESP_OK) {
            result = err;
        }
    }
    return result;
}

static void latch_fault(esp_err_t err)
{
    s_fault = true;
    stop_both(now_ms());
    ESP_LOGE(TAG, "Motor driver API failed (%s); both bridges disabled, reset to retry",
             esp_err_to_name(err));
}

static void control_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        const uint64_t now = now_ms();
        int previous[WHEEL_COUNT];
        bool timed_out = false;
        for (int i = 0; i < WHEEL_COUNT; ++i) {
            previous[i] = s_ramps[i].output_percent;
            if (motor_ramp_step(&s_ramps[i], now)) {
                timed_out = true;
            }
        }
        if (timed_out) {
            const esp_err_t err = stop_both(now);
            if (err != ESP_OK) {
                latch_fault(err);
            }
        } else if (!s_fault) {
            for (int i = 0; i < WHEEL_COUNT; ++i) {
                if (previous[i] == s_ramps[i].output_percent) {
                    continue;
                }
                const esp_err_t err = apply_output(&s_wheels[i], s_ramps[i].output_percent);
                if (err != ESP_OK) {
                    latch_fault(err);
                    break;
                }
            }
        }
        xSemaphoreGive(s_lock);
        if (timed_out) {
            ESP_LOGW(TAG, "Motor command expired: both outputs disabled");
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

static esp_err_t init_wheel_gpio(const wheel_config_t *wheel)
{
    // Preload low output levels before connecting pins to the output drivers.
    ESP_RETURN_ON_ERROR(set_enable(wheel, false), TAG, "clear R_EN and L_EN");
    ESP_RETURN_ON_ERROR(gpio_set_level(wheel->rpwm_gpio, 0), TAG, "clear RPWM");
    ESP_RETURN_ON_ERROR(gpio_set_level(wheel->lpwm_gpio, 0), TAG, "clear LPWM");
    const gpio_config_t pins = {
        .pin_bit_mask = (1ULL << wheel->r_en_gpio) | (1ULL << wheel->l_en_gpio) |
                        (1ULL << wheel->rpwm_gpio) | (1ULL << wheel->lpwm_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&pins);
}

esp_err_t bts7960_init(void)
{
    if (s_lock != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(init_wheel_gpio(&s_wheels[i]), TAG, "configure wheel GPIO");
    }

    const ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION_BITS,
        .timer_num = PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "configure PWM timer");
    ledc_channel_config_t channel = {
        .speed_mode = PWM_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const wheel_config_t *wheel = &s_wheels[i];
        channel.gpio_num = wheel->rpwm_gpio;
        channel.channel = wheel->rpwm_channel;
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "configure RPWM");
        channel.gpio_num = wheel->lpwm_gpio;
        channel.channel = wheel->lpwm_channel;
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "configure LPWM");
        motor_ramp_init(&s_ramps[i], now_ms());
        ESP_LOGI(TAG, "%s wheel: RPWM=%d LPWM=%d R_EN=%d L_EN=%d reversed=%d",
                 i == LEFT_WHEEL ? "Left" : "Right", wheel->rpwm_gpio, wheel->lpwm_gpio,
                 wheel->r_en_gpio, wheel->l_en_gpio, wheel->reversed);
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(control_task, "motor_control", 3072, NULL, 5, NULL) != pdPASS) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_ready = true;
    return ESP_OK;
}

esp_err_t bts7960_set_targets(int left, int right, uint32_t timeout_ms)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (left < -MOTOR_MAX_DUTY_PERCENT || left > MOTOR_MAX_DUTY_PERCENT ||
        right < -MOTOR_MAX_DUTY_PERCENT || right > MOTOR_MAX_DUTY_PERCENT || timeout_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t result = s_fault ? ESP_ERR_INVALID_STATE : ESP_OK;
    if (!s_fault) {
        const uint64_t now = now_ms();
        motor_ramp_set_target_timed(&s_ramps[LEFT_WHEEL], left, now, timeout_ms);
        motor_ramp_set_target_timed(&s_ramps[RIGHT_WHEEL], right, now, timeout_ms);
    }
    xSemaphoreGive(s_lock);
    return result;
}

esp_err_t bts7960_stop(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t err = stop_both(now_ms());
    if (err != ESP_OK) {
        latch_fault(err);
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t bts7960_get_status(bts7960_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *status = (bts7960_status_t) {
        .left = { s_ramps[LEFT_WHEEL].target_percent, s_ramps[LEFT_WHEEL].output_percent },
        .right = { s_ramps[RIGHT_WHEEL].target_percent, s_ramps[RIGHT_WHEEL].output_percent },
        .fault = s_fault,
    };
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
