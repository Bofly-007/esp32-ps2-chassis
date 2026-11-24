#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "bts7960.h"
#include "motor_ramp.h"
#include "ps2_bus.h"
#include "ps2_control.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "motor";
// Lock order is always input lock -> BTS7960 lock. Bus I/O holds neither.
static SemaphoreHandle_t s_input_lock;
static ps2_control_t s_input;
static uint8_t s_raw_mode = 0xff;

static uint64_t now_ms(void)
{
    return (uint64_t)esp_timer_get_time() / 1000;
}

static void ps2_task(void *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(200)); // Receiver power-up; motor stays disabled.
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_retry_ms = 0;
    bool attempted_config = false;
    bool motor_error_reported = false;
    while (true) {
        ps2_sample_t sample;
        uint8_t raw_mode;
        const bool valid = ps2_bus_read(&sample, &raw_mode);
        const uint64_t now = now_ms();

        xSemaphoreTake(s_input_lock, portMAX_DELAY);
        const bool was_connected = s_input.connected;
        s_raw_mode = raw_mode;
        ps2_wheel_targets_t targets;
        const ps2_action_t action = ps2_control_step(&s_input, valid ? &sample : NULL,
                                                     now, &targets);
        esp_err_t err = ESP_OK;
        if (action == PS2_SET_TARGET) {
            err = bts7960_set_targets(targets.left, targets.right, PS2_COMMAND_TIMEOUT_MS);
        } else if (action == PS2_STOP_OUTPUT) {
            err = bts7960_stop();
        }
        if (err != ESP_OK) {
            ps2_control_stop(&s_input);
        }
        const bool connected = s_input.connected;
        xSemaphoreGive(s_input_lock);

        if (connected != was_connected) {
            if (connected) {
                ESP_LOGI(TAG, "PS2 receiver frames OK; center left stick and release L1 to arm");
            } else {
                ESP_LOGW(TAG, "PS2 receiver frame timeout; PS2 output disabled, neutral required");
            }
        }
        if (err != ESP_OK && !motor_error_reported) {
            ESP_LOGE(TAG, "PS2 motor command failed: %s", esp_err_to_name(err));
            motor_error_reported = true;
        }

        // Retry in bounded attempts outside the input lock. The independent
        // motor task still enforces the 200 ms command lease during bus work.
        if (!valid && (!attempted_config || now - last_retry_ms >= 1000)) {
            ps2_bus_configure_analog();
            attempted_config = true;
            last_retry_ms = now;
            last_wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PS2_POLL_PERIOD_MS));
    }
}

static bool parse_percent(const char *arg, int *result)
{
    errno = 0;
    char *end;
    const long percent = strtol(arg, &end, 10);
    if (errno != 0 || end == arg || *end != '\0' ||
        percent < -MOTOR_MAX_DUTY_PERCENT || percent > MOTOR_MAX_DUTY_PERCENT) {
        return false;
    }
    *result = (int)percent;
    return true;
}

static int command_motor(int argc, char **argv)
{
    int left, right = 0;
    if ((argc != 2 && argc != 3) || !parse_percent(argv[1], &left) ||
        (argc == 3 && !parse_percent(argv[2], &right))) {
        printf("Usage: motor <left> [right] (integer PWM from -%d to %d)\n",
               MOTOR_MAX_DUTY_PERCENT, MOTOR_MAX_DUTY_PERCENT);
        return 1;
    }
    xSemaphoreTake(s_input_lock, portMAX_DELAY);
    if (s_input.mode != INPUT_SERIAL) {
        xSemaphoreGive(s_input_lock);
        printf("PS2 mode active. Enter 'mode serial' before serial motor tests.\n");
        return 1;
    }
    const esp_err_t err = bts7960_set_targets(left, right, MOTOR_COMMAND_TIMEOUT_MS);
    xSemaphoreGive(s_input_lock);
    if (err != ESP_OK) {
        printf("Motor command failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("Target PWM: left=%+d%% right=%+d%%. Refresh within %d seconds.\n",
           left, right, MOTOR_COMMAND_TIMEOUT_MS / 1000);
    return 0;
}

static int command_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    xSemaphoreTake(s_input_lock, portMAX_DELAY);
    ps2_control_stop(&s_input);
    const esp_err_t err = bts7960_stop();
    const bool ps2 = s_input.mode == INPUT_PS2;
    xSemaphoreGive(s_input_lock);
    printf("Bridge disable: %s (coast stop, no active brake).\n", esp_err_to_name(err));
    if (ps2) {
        printf("To rearm: center left stick and release L1 and CROSS.\n");
    }
    return err != ESP_OK;
}

static int command_mode(int argc, char **argv)
{
    if (argc != 2 || (strcmp(argv[1], "ps2") != 0 && strcmp(argv[1], "serial") != 0)) {
        printf("Usage: mode <ps2|serial> (switching always stops the motor)\n");
        return 1;
    }
    const input_mode_t mode = strcmp(argv[1], "ps2") == 0 ? INPUT_PS2 : INPUT_SERIAL;
    xSemaphoreTake(s_input_lock, portMAX_DELAY);
    ps2_control_set_mode(&s_input, mode);
    const esp_err_t err = bts7960_stop();
    xSemaphoreGive(s_input_lock);
    printf("Mode=%s, bridge disable=%s.\n", argv[1], esp_err_to_name(err));
    if (mode == INPUT_PS2) {
        printf("Center left stick and release L1/CROSS, then hold L1 to drive.\n");
    }
    return err != ESP_OK;
}

static int command_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bts7960_status_t status;
    xSemaphoreTake(s_input_lock, portMAX_DELAY);
    const esp_err_t err = bts7960_get_status(&status);
    const ps2_control_t input = s_input;
    const uint8_t raw_mode = s_raw_mode;
    xSemaphoreGive(s_input_lock);
    if (err != ESP_OK) {
        printf("Status failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("Left: Target=%+d%% Output=%+d%% | Right: Target=%+d%% Output=%+d%% | Fault=%s\n",
           status.left.target_percent, status.left.output_percent,
           status.right.target_percent, status.right.output_percent, status.fault ? "yes" : "no");
    printf("Mode=%s PS2_frames=%s Ready=%s Raw_mode=0x%02x\n",
           input.mode == INPUT_PS2 ? "ps2" : "serial",
           input.connected ? "OK" : "missing", input.ready ? "yes" : "no", raw_mode);
    if (input.connected) {
        printf("Last frame: age=%" PRIu64 "ms LX=%u LY=%u L1=%s CROSS=%s\n",
               now_ms() - input.last_valid_ms, input.sample.left_x, input.sample.left_y,
               input.sample.buttons & PS2_BUTTON_L1 ? "held" : "released",
               input.sample.buttons & PS2_BUTTON_CROSS ? "held" : "released");
    }
    return 0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Motor project started");
    ESP_LOGI(TAG, "Target: %s | ESP-IDF: %s",
             CONFIG_IDF_TARGET, esp_get_idf_version());
    ESP_ERROR_CHECK(bts7960_init());
    ESP_ERROR_CHECK(ps2_bus_init());
    s_input_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_input_lock != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ps2_control_init(&s_input);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "motor> ";
    repl_config.max_cmdline_length = 80;
    const esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    const esp_console_cmd_t commands[] = {
        { .command = "mode", .help = "Select PS2 or serial control; always stops first",
          .hint = "<ps2|serial>", .func = command_mode },
        { .command = "motor", .help = "Serial mode: wheel PWM -100..100 (omitted right = 0)",
          .hint = "<left> [right]", .func = command_motor },
        { .command = "stop", .help = "Disable both bridges immediately (wheels coast)",
          .func = command_stop },
        { .command = "status", .help = "Show commanded PWM; does not refresh timeout",
          .func = command_status },
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }

    ESP_ERROR_CHECK(xTaskCreate(ps2_task, "ps2_input", 4096, NULL, 4, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "PS2 differential drive, open-loop PWM, default mode=ps2");
    ESP_LOGI(TAG, "PS2 DAT=%d CMD=%d CS=%d CLK=%d; hold L1 + left stick, CROSS stops",
             PS2_DAT_GPIO, PS2_CMD_GPIO, PS2_CS_GPIO, PS2_CLK_GPIO);
    ESP_LOGI(TAG, "Boot output OFF. Center/release to arm. Serial test: mode serial | motor 0 40");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
