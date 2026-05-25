// BARINV Scale Clean — Phase 1 (no HX711 yet)
//
// Boot order, all proven via SAFEBOOT-1/2/3:
//   1. NVS
//   2. I2C + TCA9554
//   3. Backlight LEDC (configured, OFF)
//   4. ST7701 RGB panel
//   5. LVGL (esp_lvgl_port) — touch added next
//   6. CST820 touch -> registered with LVGL
//   7. UI (single scale screen)
//   8. Backlight ON
//
// No tasks spawned here. The LVGL port already runs its own task.
// HX711 + its owner task come in Phase 2.

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "i2c_bus.h"
#include "tca9554.h"
#include "backlight.h"
#include "st7701_panel.h"
#include "cst820_touch.h"
#include "lvgl_glue.h"
#include "ui.h"

static const char *TAG = "barinv";

void app_main(void) {
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, " BARINV Scale Clean — boot");
    ESP_LOGI(TAG, " target : Waveshare ESP32-S3 Touch LCD 2.1");
    ESP_LOGI(TAG, "============================================================");

    // ---- NVS (Phase 2 will use it for tare offset + cal factor) ----------
    {
        esp_err_t r = nvs_flash_init();
        if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            r = nvs_flash_init();
        }
        ESP_ERROR_CHECK(r);
    }

    // ---- I2C + TCA9554 ---------------------------------------------------
    ESP_ERROR_CHECK(bsp_i2c_init());
    ESP_ERROR_CHECK(bsp_tca9554_init());
    ESP_LOGI(TAG, "[1/6] I2C + TCA9554 ready");

    // ---- Backlight LEDC (off; turned on after panel is up) ---------------
    ESP_ERROR_CHECK(bsp_backlight_init());

    // ---- ST7701 ----------------------------------------------------------
    esp_lcd_panel_handle_t    panel = NULL;
    esp_lcd_panel_io_handle_t io    = NULL;
    ESP_ERROR_CHECK(bsp_st7701_init(&panel, &io));
    ESP_LOGI(TAG, "[2/6] ST7701 up");

    // ---- LVGL (touch added separately below) -----------------------------
    ESP_ERROR_CHECK(bsp_lvgl_init(panel, io, NULL));
    ESP_LOGI(TAG, "[3/6] LVGL up");

    // ---- CST820 touch ----------------------------------------------------
    esp_lcd_touch_handle_t touch = NULL;
    esp_err_t tret = bsp_cst820_init(&touch);
    if (tret == ESP_OK) {
        const lvgl_port_touch_cfg_t tcfg = {
            .disp   = lv_disp_get_default(),
            .handle = touch,
        };
        if (lvgl_port_add_touch(&tcfg) == NULL) {
            ESP_LOGW(TAG, "lvgl_port_add_touch returned NULL — UI inert");
        }
        ESP_LOGI(TAG, "[4/6] CST820 wired into LVGL");
    } else {
        ESP_LOGW(TAG, "[4/6] CST820 init failed (%s) — UI renders without taps",
                 esp_err_to_name(tret));
    }

    // ---- UI --------------------------------------------------------------
    ui_init();
    ESP_LOGI(TAG, "[5/6] UI built");

    // ---- Backlight ON ----------------------------------------------------
    ESP_ERROR_CHECK(bsp_backlight_on());
    ESP_LOGI(TAG, "[6/6] backlight ON");

    ESP_LOGI(TAG, "ready — BARINV Scale Clean Phase 1");
    // app_main returns; LVGL port task and FreeRTOS idle keep the system alive.
}
