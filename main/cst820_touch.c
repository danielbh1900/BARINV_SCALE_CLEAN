// cst820_touch.c — CST820 brought up through the CST816S protocol driver
// (the silicon-to-driver compatibility note comes from the Waveshare-authored
// board header — both chips share the same minimal register interface).
#include "cst820_touch.h"
#include "board_config.h"
#include "tca9554.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cst820";

esp_err_t bsp_cst820_init(esp_lcd_touch_handle_t *out_touch) {
    if (out_touch == NULL) return ESP_ERR_INVALID_ARG;

    // ---- 1. Reset the touch controller via TCA9554 EXIO2 -----------------
    bsp_tca9554_set(BSP_EXIO_TP_RST_BIT, false);
    vTaskDelay(pdMS_TO_TICKS(30));
    bsp_tca9554_set(BSP_EXIO_TP_RST_BIT, true);
    vTaskDelay(pdMS_TO_TICKS(50));

    // ---- 2. Build the I²C panel-IO for the touch controller --------------
    //  ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG() in esp_lcd_touch_cst816s 1.0.6
    //  sets .scl_speed_hz = 400000.  esp_lcd_new_panel_io_i2c_v1 (the legacy
    //  i2c driver path we are on — see i2c_bus.c using driver/i2c.h) rejects
    //  any non-zero scl_speed_hz with ESP_ERR_INVALID_ARG, because clock is
    //  configured by i2c_param_config at bus-init time.  Drop the const and
    //  zero the field; everything else from the macro stays.
    esp_lcd_panel_io_handle_t tp_io = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    io_cfg.scl_speed_hz = 0;
    esp_err_t ret = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM,
                                              &io_cfg, &tp_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_io_i2c: %s", esp_err_to_name(ret));
        return ret;
    }

    // ---- 3. Build the esp_lcd_touch driver -------------------------------
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max        = BSP_LCD_H_RES,
        .y_max        = BSP_LCD_V_RES,
        .rst_gpio_num = BSP_TOUCH_PIN_RST,   // -1: managed via expander above
        .int_gpio_num = BSP_TOUCH_PIN_INT,
        .levels = {
            .reset     = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy  = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ret = esp_lcd_touch_new_i2c_cst816s(tp_io, &tp_cfg, out_touch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cst816s_new (CST820): %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "CST820 ready @ I²C 0x%02X, INT GPIO %d",
             BSP_TOUCH_ADDR, BSP_TOUCH_PIN_INT);
    return ESP_OK;
}
