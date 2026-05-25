// lvgl_glue.h — esp_lvgl_port wiring for our RGB panel + I²C touch.
#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

// Initialises LVGL + registers the given panel + touch device with
// esp_lvgl_port, allocates draw buffers in PSRAM, and starts the LVGL task.
// On success returns the LVGL display handle (do not store directly; use
// lv_disp_get_default() in screen code).
esp_err_t bsp_lvgl_init(esp_lcd_panel_handle_t panel,
                        esp_lcd_panel_io_handle_t panel_io,
                        esp_lcd_touch_handle_t   touch);
