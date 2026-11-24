#include "ps2_bus.h"

#include "board_config.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// PS2 uses idle-high CLK, LSB first, active-low CS. Read DAT during CLK low.
#define PS2_HALF_CLOCK_US 10
#define PS2_BYTE_GAP_US 16
static portMUX_TYPE s_byte_lock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t transfer_byte(uint8_t output)
{
    uint8_t input = 0;
    // Bound interrupt masking to ONE byte (~160 us), not an entire packet.
    portENTER_CRITICAL(&s_byte_lock);
    for (unsigned bit = 0; bit < 8; ++bit) {
        gpio_set_level(PS2_CMD_GPIO, (output >> bit) & 1U);
        gpio_set_level(PS2_CLK_GPIO, 0);
        esp_rom_delay_us(PS2_HALF_CLOCK_US);
        if (gpio_get_level(PS2_DAT_GPIO)) {
            input |= (uint8_t)(1U << bit);
        }
        gpio_set_level(PS2_CLK_GPIO, 1);
        esp_rom_delay_us(PS2_HALF_CLOCK_US);
    }
    gpio_set_level(PS2_CMD_GPIO, 1);
    portEXIT_CRITICAL(&s_byte_lock);
    esp_rom_delay_us(PS2_BYTE_GAP_US);
    return input;
}

static void begin_transaction(void)
{
    gpio_set_level(PS2_CMD_GPIO, 1);
    gpio_set_level(PS2_CLK_GPIO, 1);
    gpio_set_level(PS2_CS_GPIO, 0);
    esp_rom_delay_us(PS2_BYTE_GAP_US);
}

static void send_command(const uint8_t *command, size_t size)
{
    begin_transaction();
    for (size_t i = 0; i < size; ++i) {
        transfer_byte(command[i]);
    }
    gpio_set_level(PS2_CS_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

esp_err_t ps2_bus_init(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(PS2_CLK_GPIO, 1), "ps2", "CLK idle");
    ESP_RETURN_ON_ERROR(gpio_set_level(PS2_CMD_GPIO, 1), "ps2", "CMD idle");
    ESP_RETURN_ON_ERROR(gpio_set_level(PS2_CS_GPIO, 1), "ps2", "CS idle");
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << PS2_CLK_GPIO) | (1ULL << PS2_CMD_GPIO) |
                        (1ULL << PS2_CS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&outputs), "ps2", "output pins");
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << PS2_DAT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&input);
}

bool ps2_bus_read(ps2_sample_t *sample, uint8_t *raw_mode)
{
    uint8_t data[PS2_FRAME_MAX_BYTES];
    begin_transaction();
    data[0] = transfer_byte(0x01);
    data[1] = transfer_byte(0x42);
    data[2] = transfer_byte(0x00);
    const size_t size = data[1] == 0x79 ? 21 : data[1] == 0x41 ? 5 : 9;
    for (size_t i = 3; i < size; ++i) {
        data[i] = transfer_byte(0x00);
    }
    gpio_set_level(PS2_CS_GPIO, 1);
    *raw_mode = data[1];
    return ps2_decode_frame(data, size, sample);
}

void ps2_bus_configure_analog(void)
{
    // Standard PS2 protocol commands, no rumble or pressure-mode request.
    static const uint8_t enter[] = {0x01, 0x43, 0x00, 0x01, 0x00};
    static const uint8_t analog[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0, 0, 0, 0};
    static const uint8_t exit[] = {0x01, 0x43, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a};
    send_command(enter, sizeof(enter));
    send_command(analog, sizeof(analog));
    send_command(exit, sizeof(exit));
}
