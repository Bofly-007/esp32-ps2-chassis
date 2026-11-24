#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ps2_control.h"

// Only the PS2 reader task may access the bus after initialization.
esp_err_t ps2_bus_init(void);
bool ps2_bus_read(ps2_sample_t *sample, uint8_t *raw_mode);
// One bounded configuration attempt; success is confirmed by the next read.
void ps2_bus_configure_analog(void);
