// backlight.c
#include "backlight.h"
#include "board_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "backlight";
static bool s_inited = false;

esp_err_t bsp_backlight_init(void) {
    if (s_inited) return ESP_OK;

    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BSP_LCD_BACKLIGHT_RES_BITS,
        .timer_num       = BSP_LCD_BACKLIGHT_LEDC_TIMER,
        .freq_hz         = BSP_LCD_BACKLIGHT_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&tcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ccfg = {
        .gpio_num   = BSP_LCD_BACKLIGHT_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BSP_LCD_BACKLIGHT_LEDC_CH,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = BSP_LCD_BACKLIGHT_LEDC_TIMER,
        .duty       = 0,        // start dark; turned on after panel init
        .hpoint     = 0,
    };
    ret = ledc_channel_config(&ccfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config: %s", esp_err_to_name(ret));
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "backlight LEDC ready (GPIO %d, %d Hz, 10-bit)",
             BSP_LCD_BACKLIGHT_PIN, BSP_LCD_BACKLIGHT_FREQ_HZ);
    return ESP_OK;
}

esp_err_t bsp_backlight_set(uint16_t duty_0_to_1023) {
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (duty_0_to_1023 > 1023) duty_0_to_1023 = 1023;
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                   BSP_LCD_BACKLIGHT_LEDC_CH,
                                   duty_0_to_1023);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_BACKLIGHT_LEDC_CH);
}

esp_err_t bsp_backlight_on(void) {
    return bsp_backlight_set(BSP_LCD_BACKLIGHT_DEFAULT);
}

esp_err_t bsp_backlight_off(void) {
    return bsp_backlight_set(0);
}
