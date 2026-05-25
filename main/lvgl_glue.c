// lvgl_glue.c — esp_lvgl_port integration.
#include "lvgl_glue.h"
#include "board_config.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

static const char *TAG = "lvgl";

esp_err_t bsp_lvgl_init(esp_lcd_panel_handle_t panel,
                        esp_lcd_panel_io_handle_t panel_io,
                        esp_lcd_touch_handle_t   touch) {
    if (panel == NULL || panel_io == NULL) return ESP_ERR_INVALID_ARG;

    // Pin LVGL task to CPU1 so the IDLE0 task always has CPU0 to itself.
    // With task_affinity = -1 (default) FreeRTOS happily keeps LVGL on CPU0
    // and the flush bursts starve IDLE0 → task_wdt fires during draw_buf_flush.
    // Stack bumped to 8 KB — the default 4 KB is tight once LVGL recurses
    // through paint + PSRAM flush + theme.
    const lvgl_port_cfg_t port_cfg = {
        .task_priority     = 4,
        .task_stack        = 8192,
        .task_affinity     = 1,           // CPU1
        .task_max_sleep_ms = 500,
        .timer_period_ms   = 5,
    };
    esp_err_t ret = lvgl_port_init(&port_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "port_init: %s", esp_err_to_name(ret));
        return ret;
    }

    // ---- Display ----------------------------------------------------------
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle    = panel_io,
        .panel_handle = panel,
        .buffer_size  = BSP_LCD_H_RES * 40,   // 40-line tear-buffer in PSRAM
        .double_buffer = true,
        .hres         = BSP_LCD_H_RES,
        .vres         = BSP_LCD_V_RES,
        .monochrome   = false,
        .rotation = {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma    = false,
            .buff_spiram = true,
        },
    };

    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    if (disp == NULL) {
        ESP_LOGE(TAG, "add_disp returned NULL");
        return ESP_FAIL;
    }

    // ---- Touch (optional — UI still renders if NULL) ----------------------
    if (touch != NULL) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp   = disp,
            .handle = touch,
        };
        if (lvgl_port_add_touch(&touch_cfg) == NULL) {
            ESP_LOGW(TAG, "add_touch returned NULL — UI runs without taps");
        }
    }

    ESP_LOGI(TAG, "LVGL up — disp=%p, touch=%p", (void *)disp, (void *)touch);
    return ESP_OK;
}
