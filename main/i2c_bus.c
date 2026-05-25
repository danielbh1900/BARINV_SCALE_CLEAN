// i2c_bus.c
#include "i2c_bus.h"
#include "board_config.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";
static bool s_inited = false;

esp_err_t bsp_i2c_init(void) {
    if (s_inited) return ESP_OK;

    const i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BSP_I2C_PIN_SDA,
        .scl_io_num       = BSP_I2C_PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_HZ,
    };

    esp_err_t ret = i2c_param_config(BSP_I2C_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(BSP_I2C_NUM, cfg.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C0 up — SDA=%d SCL=%d @ %d Hz",
             BSP_I2C_PIN_SDA, BSP_I2C_PIN_SCL, BSP_I2C_HZ);
    s_inited = true;
    return ESP_OK;
}
