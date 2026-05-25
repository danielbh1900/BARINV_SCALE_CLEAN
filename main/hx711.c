// hx711.c — bit-bang driver for the HX711 24-bit load-cell ADC.
//
// Phase 2 H2 implementation:
//   * real gpio_config of SCK (output, low) and DOUT (input)
//   * data_ready = DOUT level == 0
//   * read_raw_blocking = yielding wait for DOUT-low, then a critical-section
//     burst of 24 SCK pulses (capturing MSB-first into 24-bit value), then
//     1-3 extra pulses for next-sample gain/channel selection
//   * 24-bit two's-complement sign-extend to int32_t
//   * single-owner task at ~10 Hz that ESP_LOGIs raw / dout / ready only —
//     no UI updates, no tare, no calibration
//
// Console is on USB-Serial-JTAG (CONFIG_ESP_CONSOLE_UART_NUM=-1), so the
// default UART0 TX=GPIO43 / RX=GPIO44 pins are free for the HX711.
//
// SCK timing: HX711 needs SCK-high ≥ 0.2 µs and ≤ 50 µs (the chip enters
// power-down if SCK stays high > 60 µs). esp_rom_delay_us(1) is comfortably
// inside that window. The critical section around the burst is ~50-60 µs
// total — far below any task_wdt threshold.

#include "hx711.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static const char *TAG = "hx711";

static int          s_sck          = -1;
static int          s_dout         = -1;
static hx711_gain_t s_gain_next    = HX711_GAIN_128_A;
static bool         s_inited       = false;
static bool         s_task_started = false;
static portMUX_TYPE s_burst_mux    = portMUX_INITIALIZER_UNLOCKED;

static inline int gain_total_pulses(hx711_gain_t g) {
    switch (g) {
        case HX711_GAIN_128_A: return 25;
        case HX711_GAIN_32_B:  return 26;
        case HX711_GAIN_64_A:  return 27;
        default:               return 25;
    }
}

esp_err_t hx711_init(int sck_gpio, int dout_gpio) {
    if (sck_gpio < 0 || dout_gpio < 0) return ESP_ERR_INVALID_ARG;

    gpio_config_t sck_cfg = {
        .pin_bit_mask = 1ULL << sck_gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t r = gpio_config(&sck_cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config SCK (GPIO%d) failed: %s",
                 sck_gpio, esp_err_to_name(r));
        return r;
    }
    gpio_set_level(sck_gpio, 0);

    gpio_config_t dt_cfg = {
        .pin_bit_mask = 1ULL << dout_gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    r = gpio_config(&dt_cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config DOUT (GPIO%d) failed: %s",
                 dout_gpio, esp_err_to_name(r));
        return r;
    }

    s_sck    = sck_gpio;
    s_dout   = dout_gpio;
    s_inited = true;
    ESP_LOGI(TAG, "init OK: SCK=GPIO%d DOUT=GPIO%d (BSP_HX711_SWAP_PINS=%d)",
             s_sck, s_dout, (int)BSP_HX711_SWAP_PINS);
    return ESP_OK;
}

bool hx711_data_ready(void) {
    if (!s_inited) return false;
    return gpio_get_level(s_dout) == 0;
}

esp_err_t hx711_read_raw_blocking(int32_t *out, uint32_t timeout_ms) {
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    /* Wait for DOUT LOW.  Yielding wait — DO NOT disable interrupts here.
       HX711 default RATE pin = LOW => 10 Hz output, so DOUT goes low every
       ~100 ms.  timeout_ms gives an upper bound the caller can react to. */
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (gpio_get_level(s_dout) != 0) {
        if (xTaskGetTickCount() >= deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);   // 1 tick (= 1 ms with FREERTOS_HZ=1000)
    }

    const int total = gain_total_pulses(s_gain_next);
    uint32_t  value = 0;

    /* Critical section ONLY around the short clock burst (≈ 50-60 µs total).
       portENTER_CRITICAL only locks this core — the other core continues. */
    portENTER_CRITICAL(&s_burst_mux);
    for (int i = 0; i < 24; ++i) {
        gpio_set_level(s_sck, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | (uint32_t)gpio_get_level(s_dout);
        gpio_set_level(s_sck, 0);
        esp_rom_delay_us(1);
    }
    for (int i = 24; i < total; ++i) {
        gpio_set_level(s_sck, 1);
        esp_rom_delay_us(1);
        gpio_set_level(s_sck, 0);
        esp_rom_delay_us(1);
    }
    portEXIT_CRITICAL(&s_burst_mux);

    /* 24-bit two's complement → sign-extend to int32_t. */
    if (value & 0x00800000u) value |= 0xFF000000u;
    *out = (int32_t)value;
    return ESP_OK;
}

esp_err_t hx711_set_gain_next(hx711_gain_t g) {
    if (g != HX711_GAIN_128_A && g != HX711_GAIN_32_B && g != HX711_GAIN_64_A)
        return ESP_ERR_INVALID_ARG;
    s_gain_next = g;
    return ESP_OK;
}

/* ------------------------------------------------------------------------
 * Owner task — H2: serial-log only.
 *
 *   * pinned to CPU0 (LVGL task is on CPU1, so we keep them on opposite
 *     cores by default — neither starves the other)
 *   * priority 3 (LVGL is 4, so LVGL still wins ties)
 *   * 4 KB stack — bit-bang + ESP_LOGI fit comfortably
 *   * ~10 Hz cadence via vTaskDelay(100 ms); HX711 internal RATE is 10 Hz
 *     so we naturally land near each DRDY edge
 *   * NO UI calls, NO tare, NO calibration, NO mock fallback
 * ------------------------------------------------------------------------ */
static void hx711_task(void *arg) {
    (void)arg;
    int32_t  raw           = 0;
    int32_t  prev_raw      = 0x7FFFFFFF;   /* impossible 24-bit value */
    uint32_t stuck_count   = 0;            /* consecutive identical reads */
    ESP_LOGI(TAG, "owner task running on core %d @ ~10 Hz "
                  "(raw-only, SCK=GPIO%d DOUT=GPIO%d, BSP_HX711_SWAP_PINS=%d)",
             (int)xPortGetCoreID(),
             s_sck, s_dout, (int)BSP_HX711_SWAP_PINS);

    for (;;) {
        int  dout_pre = gpio_get_level(s_dout);
        bool ready    = (dout_pre == 0);

        esp_err_t r = hx711_read_raw_blocking(&raw, 200);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "raw=%ld  dout=%d  ready=%d",
                     (long)raw, dout_pre, (int)ready);

            /* Stuck-value detection (H3 observability): warn loudly every
             * ~2 s if every read returns the same value AND it's one of
             * the obvious failure sentinels (0 or -1 = 0xFFFFFF). */
            if (raw == prev_raw) {
                if (++stuck_count == 20 && (raw == 0 || raw == -1)) {
                    ESP_LOGW(TAG, "STUCK: raw=%ld for %u consecutive reads "
                                  "(~2 s). SCK/DOUT wiring or HX711 power "
                                  "likely wrong — current mapping SCK=GPIO%d "
                                  "DOUT=GPIO%d (BSP_HX711_SWAP_PINS=%d).",
                             (long)raw, (unsigned)stuck_count,
                             s_sck, s_dout, (int)BSP_HX711_SWAP_PINS);
                }
            } else {
                stuck_count = 0;
                prev_raw    = raw;
            }
        } else if (r == ESP_ERR_TIMEOUT) {
            int dout_now = gpio_get_level(s_dout);
            ESP_LOGW(TAG, "TIMEOUT  dout=%d (stuck %s) — HX711 not asserting "
                          "DRDY; check 3.3 V / GND, wiring (current SCK=GPIO%d "
                          "DOUT=GPIO%d, SWAP=%d)",
                     dout_now, dout_now ? "HIGH" : "LOW",
                     s_sck, s_dout, (int)BSP_HX711_SWAP_PINS);
        } else {
            ESP_LOGE(TAG, "read error: %s", esp_err_to_name(r));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t hx711_owner_start(void) {
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (s_task_started) return ESP_OK;

    BaseType_t ok = xTaskCreatePinnedToCore(
        hx711_task,
        "hx711_owner",
        4096,                 /* stack */
        NULL,                 /* arg */
        3,                    /* priority — below LVGL's 4 */
        NULL,
        0);                   /* CPU0 — LVGL lives on CPU1 */
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return ESP_ERR_NO_MEM;
    }
    s_task_started = true;
    return ESP_OK;
}
