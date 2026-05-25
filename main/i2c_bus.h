// i2c_bus.h — single shared I²C0 bus for TCA9554 + CST820.
#pragma once

#include "esp_err.h"

// Initialises I²C0 on the locked board pins (SDA=15, SCL=7) at 400 kHz.
// Idempotent — safe to call once during boot.
esp_err_t bsp_i2c_init(void);
