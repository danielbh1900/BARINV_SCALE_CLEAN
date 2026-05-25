// cst820_touch.h — CST820 capacitive touch via the CST816S-compatible driver.
#pragma once

#include "esp_err.h"
#include "esp_lcd_touch.h"

// Resets the touch panel (via TCA9554 EXIO2) and creates the esp_lcd_touch
// handle ready for LVGL wiring. Requires bsp_i2c_init() + bsp_tca9554_init().
esp_err_t bsp_cst820_init(esp_lcd_touch_handle_t *out_touch);
