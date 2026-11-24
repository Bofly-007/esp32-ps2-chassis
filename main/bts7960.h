#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int target_percent;
    int output_percent;
} bts7960_wheel_status_t;

typedef struct {
    bts7960_wheel_status_t left;
    bts7960_wheel_status_t right;
    bool fault;
} bts7960_status_t;

// Initializes both bridges and starts a shared control task with zero output.
esp_err_t bts7960_init(void);
// Update both targets together. Positive means forward after polarity mapping.
esp_err_t bts7960_set_targets(int left, int right, uint32_t timeout_ms);
// Disables both bridges immediately. The wheels coast.
esp_err_t bts7960_stop(void);
esp_err_t bts7960_get_status(bts7960_status_t *status);
