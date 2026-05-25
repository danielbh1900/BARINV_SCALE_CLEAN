// st7701_panel.h — ST7701S display bring-up.
//
//   - 3-wire SPI init bus: SCK = GPIO 2, MOSI = GPIO 1, CS via TCA9554 EXIO3.
//   - RGB stream: see board_config.h for pin map + timing.
//   - LCD reset is on TCA9554 EXIO1 (driven by this module before init).
//
// Requires bsp_i2c_init() and bsp_tca9554_init() to have succeeded first.
#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"

// Brings up the panel and returns the panel handle for LVGL wiring.
// On failure returns NULL and logs detail.
esp_err_t bsp_st7701_init(esp_lcd_panel_handle_t *out_panel,
                          esp_lcd_panel_io_handle_t *out_io);
