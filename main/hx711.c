// hx711.c — Phase 2 H1 SKELETON ONLY.
//
// All function bodies intentionally return ESP_ERR_NOT_SUPPORTED.
// No gpio_config, no GPIO reads or writes, no tasks, no logging.
// Goal of H1: keep build green and reserve the API surface without changing
// any device behavior. H2 will fill in the bit-bang implementation.

#include "hx711.h"
#include "board_config.h"

esp_err_t hx711_init(int sck_gpio, int dout_gpio)
{
    (void)sck_gpio;
    (void)dout_gpio;
    return ESP_ERR_NOT_SUPPORTED;   // H2: gpio_config SCK=OUT(low), DOUT=IN
}

bool hx711_data_ready(void)
{
    return false;                   // H2: return gpio_get_level(BSP_HX711_DOUT) == 0
}

esp_err_t hx711_read_raw_blocking(int32_t *out, uint32_t timeout_ms)
{
    (void)out;
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;   // H2: bit-bang 25/26/27-clock burst here
}

esp_err_t hx711_set_gain_next(hx711_gain_t g)
{
    (void)g;
    return ESP_ERR_NOT_SUPPORTED;   // H2: select next gain via clock count
}
