// tca9554.c — thin wrapper over Espressif's esp_io_expander_tca9554 component.
#include "tca9554.h"
#include "board_config.h"
#include "esp_io_expander_tca9554.h"
#include "esp_log.h"

static const char *TAG = "tca9554";
static esp_io_expander_handle_t s_handle = NULL;

esp_err_t bsp_tca9554_init(void) {
    if (s_handle != NULL) return ESP_OK;

    esp_err_t ret = esp_io_expander_new_i2c_tca9554(
        BSP_I2C_NUM,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,  // resolves to 0x20
        &s_handle
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // All 8 pins → outputs.
    ret = esp_io_expander_set_dir(s_handle, 0xFF, IO_EXPANDER_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_dir failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // All outputs → LOW (matches factory EXIO_Init → TCA9554PWR_Init(0x00)).
    ret = esp_io_expander_set_level(s_handle, 0xFF, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_level(0) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "TCA9554 @ 0x%02X ready (all EXIO = OUT/LOW)",
             BSP_TCA9554_ADDR);
    return ESP_OK;
}

esp_io_expander_handle_t bsp_tca9554_handle(void) {
    return s_handle;
}

esp_err_t bsp_tca9554_set(uint8_t exio_bit_1indexed, bool level) {
    if (s_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (exio_bit_1indexed < 1 || exio_bit_1indexed > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_io_expander_set_level(
        s_handle,
        BSP_EXIO_MASK(exio_bit_1indexed),
        level ? 1 : 0
    );
}
