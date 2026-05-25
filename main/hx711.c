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

/* H8: floating-point math */
#include <math.h>

/* H7: NVS persistence for calibration only.  TARE stays RAM-only — load
 * cell zero drifts with temperature / mounting, so it must be a user
 * action after every boot. */
#include "nvs.h"

#define HX711_NVS_NAMESPACE     "barinv_scale"
#define HX711_NVS_K_VERSION     "cal_ver"
#define HX711_NVS_K_VALID       "cal_valid"
#define HX711_NVS_K_FACTOR      "cal_factor"
#define HX711_NVS_K_KNOWN       "cal_known"
#define HX711_NVS_SCHEMA_VER    1u

static const char *TAG = "hx711";

static int          s_sck          = -1;
static int          s_dout         = -1;
static hx711_gain_t s_gain_next    = HX711_GAIN_128_A;
static bool         s_inited       = false;
static bool         s_task_started = false;
static portMUX_TYPE s_burst_mux    = portMUX_INITIALIZER_UNLOCKED;

/* H4: latest-raw cache. Written by the owner task, read by any consumer.
 * int32_t writes are atomic on ESP32-S3 (32-bit aligned single word), so a
 * mutex isn't needed.  volatile prevents the compiler from caching either
 * side in a register across a yield point. */
static volatile int32_t s_latest_raw  = 0;
static volatile bool    s_have_sample = false;

/* H5: TARE state — owner task is sole writer; non-owner consumers read via
 * hx711_get_snapshot().  s_cache_mux makes (raw, net) updates atomic as a
 * pair — without it a reader could see new raw with old net or vice versa.
 * RAM-only in H5; no NVS yet. */
static volatile int32_t s_latest_net   = 0;
static volatile int32_t s_tare_offset  = 0;
static portMUX_TYPE     s_cache_mux    = portMUX_INITIALIZER_UNLOCKED;

/* Owner task handle stored at create time so hx711_request_tare() can
 * post a notification to it. */
static TaskHandle_t     s_owner_task   = NULL;

#define HX711_NOTIFY_TARE_BIT   (1u << 0)
#define HX711_TARE_SAMPLES      8

/* H6: calibration state.  Sole writer is the owner task; readers go
 * through hx711_get_snapshot_full under s_cache_mux.  Sign of
 * s_cal_factor is preserved (can be negative if cell polarity is
 * reversed), so runtime grams come out correctly oriented either way. */
static volatile float   s_cal_factor       = 0.0f;   /* counts per gram */
static volatile bool    s_cal_valid        = false;
static volatile float   s_latest_grams     = 0.0f;
static volatile float   s_cal_known_grams  = 0.0f;   /* set by request fn */

/* H6: busy guards — set by owner task at start of TARE/CAL collection,
 * cleared when done.  Request functions reject (and log "ignored —
 * already in progress") when set.  Single-writer (owner task), so
 * reading from another task can race a stale value, but the worst case
 * is a duplicate request that's harmless. */
static volatile bool    s_tare_in_progress = false;
static volatile bool    s_cal_in_progress  = false;

/* H7.1: boot auto-tare guard.  True from the moment the owner task
 * starts until the boot auto-tare completes (or is skipped).  Manual
 * TARE/CAL requests during this window are rejected with a distinct
 * "boot auto-tare in progress" log line. */
static volatile bool    s_boot_autotare_in_progress = false;

/* H8 + H8.1: display filter state.  All fields are owner-task private —
 * NOT exposed through the snapshot getter; only the post-filter grams
 * + stable flag travel through s_cache_mux. */
static float            s_filter_ema             = 0.0f;
static float            s_filter_last_accepted   = 0.0f;
static float            s_filter_window[BSP_STABLE_WINDOW_SAMPLES];
static int              s_filter_window_idx      = 0;
static int              s_filter_window_filled   = 0;
/* H8.1: pending-step candidate (replaces the H8 outlier_streak). */
static bool             s_step_pending           = false;
static float            s_step_candidate         = 0.0f;
static int              s_step_confirm_count     = 0;
static volatile bool    s_stable                 = false;

static bool filter_window_is_stable(void) {
    if (s_filter_window_filled < BSP_STABLE_WINDOW_SAMPLES) return false;
    float lo = s_filter_window[0];
    float hi = s_filter_window[0];
    for (int i = 1; i < BSP_STABLE_WINDOW_SAMPLES; ++i) {
        float v = s_filter_window[i];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    return (hi - lo) < BSP_STABLE_THRESHOLD_GRAMS;
}

/* Reset the filter to a known value (after TARE/CAL/auto-tare applies).
 * Pre-fills the stability window so STABLE returns true immediately for
 * a steady platform at the new zero.  Also clears any in-flight pending
 * step candidate. */
static void filter_reset(float to_grams) {
    s_filter_ema            = to_grams;
    s_filter_last_accepted  = to_grams;
    s_step_pending          = false;
    s_step_candidate        = 0.0f;
    s_step_confirm_count    = 0;
    for (int i = 0; i < BSP_STABLE_WINDOW_SAMPLES; ++i) {
        s_filter_window[i] = to_grams;
    }
    s_filter_window_idx    = 0;
    s_filter_window_filled = BSP_STABLE_WINDOW_SAMPLES;
    s_stable               = true;
}

/* Helper: push an accepted value through the EMA and stability window. */
static inline float ema_and_push(float accepted) {
    s_filter_ema = s_filter_ema + BSP_FILTER_EMA_ALPHA * (accepted - s_filter_ema);
    s_filter_window[s_filter_window_idx] = s_filter_ema;
    s_filter_window_idx = (s_filter_window_idx + 1) % BSP_STABLE_WINDOW_SAMPLES;
    if (s_filter_window_filled < BSP_STABLE_WINDOW_SAMPLES) {
        s_filter_window_filled++;
    }
    return s_filter_ema;
}

/* H8.1 — Process one raw post-calibration grams sample.  Pipeline:
 *
 *   1. delta = |new - last_accepted_raw|
 *   2. If delta <= OUTLIER_THRESHOLD: normal path — accept the sample
 *      directly, EMA-smooth, push into stability window.
 *   3. If delta > OUTLIER_THRESHOLD AND no pending candidate: mark
 *      this sample as the pending candidate.  Hold display.  Force
 *      stable=false (transition starting).
 *   4. If a pending candidate exists and new sample is within
 *      STEP_CONFIRM_THRESHOLD of the (running-averaged) candidate:
 *      increment confirm count, refine candidate.  Once confirm count
 *      reaches STEP_CONFIRM_SAMPLES, accept the candidate as a step
 *      change — EMA jumps toward it, push into window.
 *   5. If a pending candidate exists and new sample is far from both
 *      last_accepted AND candidate: glitch path — drop pending, hold
 *      display.  Next normal-delta sample resumes the normal flow.
 *
 * While pending or holding, *out_stable is forced false so the STABLE
 * indicator does not stay green through a real transition. */
static float filter_step(float new_grams, bool *out_stable) {
    float delta = fabsf(new_grams - s_filter_last_accepted);

    if (delta <= BSP_OUTLIER_THRESHOLD_GRAMS) {
        /* Normal flow — clear any pending candidate, accept sample. */
        s_step_pending       = false;
        s_step_confirm_count = 0;
        s_filter_last_accepted = new_grams;
        float ema = ema_and_push(new_grams);
        *out_stable = filter_window_is_stable();
        return ema;
    }

    /* delta > OUTLIER — pending-candidate machinery. */
    if (!s_step_pending) {
        /* First suspicious sample — start a candidate. */
        s_step_pending       = true;
        s_step_candidate     = new_grams;
        s_step_confirm_count = 0;
        *out_stable = false;          /* transition starting */
        return s_filter_ema;          /* hold display */
    }

    /* Already have a candidate from a previous cycle — does this sample
     * agree with it? */
    float confirm_delta = fabsf(new_grams - s_step_candidate);
    if (confirm_delta <= BSP_STEP_CONFIRM_THRESHOLD_GRAMS) {
        s_step_confirm_count++;
        /* Refine candidate with a running mean across pending+confirms. */
        s_step_candidate =
            (s_step_candidate * (float)s_step_confirm_count + new_grams) /
            (float)(s_step_confirm_count + 1);
        if (s_step_confirm_count >= BSP_STEP_CONFIRM_SAMPLES) {
            /* Confirmed step — accept the running-averaged candidate. */
            s_filter_last_accepted = s_step_candidate;
            s_step_pending         = false;
            s_step_confirm_count   = 0;
            float ema = ema_and_push(s_filter_last_accepted);
            *out_stable = filter_window_is_stable();
            return ema;
        }
        /* Still need more confirmations — continue holding. */
        *out_stable = false;
        return s_filter_ema;
    }

    /* New sample agrees with NEITHER last_accepted (delta > OUTLIER) NOR
     * the pending candidate — most likely a different glitch sample.
     * Drop pending, hold display, stay not-stable.  Next sample restarts
     * the gate fresh. */
    s_step_pending       = false;
    s_step_confirm_count = 0;
    *out_stable = false;
    return s_filter_ema;
}

#define HX711_NOTIFY_CAL_BIT    (1u << 1)
#define HX711_CAL_SAMPLES       16
#define HX711_CAL_MIN_ABS_NET   1000           /* reject if |avg_net| below */

/* H7: load saved calibration from NVS into RAM cache.  Called from
 * hx711_owner_start BEFORE the owner task spawns so the very first
 * sample produces meaningful grams.  Any failure path leaves the
 * cache uncalibrated (s_cal_valid=false) — the app keeps booting and
 * the UI shows "----" until a fresh CAL is performed. */
static esp_err_t cal_load_from_nvs(void) {
    nvs_handle_t h;
    esp_err_t r = nvs_open(HX711_NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) {
        if (r == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "no NVS calibration namespace yet — uncalibrated");
        } else {
            ESP_LOGW(TAG, "nvs_open (read) failed: %s — uncalibrated",
                     esp_err_to_name(r));
        }
        return r;
    }

    uint8_t ver = 0;
    if (nvs_get_u8(h, HX711_NVS_K_VERSION, &ver) != ESP_OK
            || ver != HX711_NVS_SCHEMA_VER) {
        ESP_LOGI(TAG, "no compatible NVS calibration (schema_ver=%u, "
                      "expected %u) — uncalibrated",
                 (unsigned)ver, (unsigned)HX711_NVS_SCHEMA_VER);
        nvs_close(h);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    uint8_t valid = 0;
    if (nvs_get_u8(h, HX711_NVS_K_VALID, &valid) != ESP_OK || valid != 1) {
        ESP_LOGI(TAG, "NVS calibration cleared (cal_valid=0) — uncalibrated");
        nvs_close(h);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    float factor = 0.0f, known = 0.0f;
    size_t sz = sizeof(float);
    if (nvs_get_blob(h, HX711_NVS_K_FACTOR, &factor, &sz) != ESP_OK
            || sz != sizeof(float)) {
        ESP_LOGW(TAG, "NVS cal_factor missing/corrupt — uncalibrated");
        nvs_close(h);
        return ESP_FAIL;
    }
    sz = sizeof(float);
    if (nvs_get_blob(h, HX711_NVS_K_KNOWN, &known, &sz) != ESP_OK
            || sz != sizeof(float)) {
        ESP_LOGW(TAG, "NVS cal_known missing/corrupt — uncalibrated");
        nvs_close(h);
        return ESP_FAIL;
    }
    nvs_close(h);

    if (factor == 0.0f) {
        ESP_LOGW(TAG, "NVS cal_factor=0 — would div by zero — uncalibrated");
        return ESP_FAIL;
    }

    /* Apply to the RAM cache.  No mux needed: owner task isn't running
     * yet, no readers can race us. */
    s_cal_factor      = factor;
    s_cal_valid       = true;
    s_cal_known_grams = known;
    ESP_LOGI(TAG, "calibration loaded from NVS: factor=%.4f counts/g, "
                  "known=%.1fg (schema v%u)",
             (double)factor, (double)known, (unsigned)HX711_NVS_SCHEMA_VER);
    return ESP_OK;
}

/* H7: persist the just-applied calibration.  Called by the CAL handler
 * after a successful RAM apply.  All keys are written then nvs_commit;
 * partial writes never become observable.  Failure is logged loudly but
 * never crashes the app — the RAM calibration is still active for the
 * current session, it just won't survive reboot. */
static esp_err_t cal_save_to_nvs(float factor, float known) {
    nvs_handle_t h;
    esp_err_t r = nvs_open(HX711_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open (rw) failed: %s — calibration in RAM only",
                 esp_err_to_name(r));
        return r;
    }
    esp_err_t e = ESP_OK;
    if (e == ESP_OK) e = nvs_set_u8  (h, HX711_NVS_K_VERSION, HX711_NVS_SCHEMA_VER);
    if (e == ESP_OK) e = nvs_set_u8  (h, HX711_NVS_K_VALID,   1);
    if (e == ESP_OK) e = nvs_set_blob(h, HX711_NVS_K_FACTOR,  &factor, sizeof(float));
    if (e == ESP_OK) e = nvs_set_blob(h, HX711_NVS_K_KNOWN,   &known,  sizeof(float));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "calibration saved to NVS: factor=%.4f counts/g, "
                      "known=%.1fg", (double)factor, (double)known);
    } else {
        ESP_LOGE(TAG, "calibration save FAILED: %s — RAM only this boot",
                 esp_err_to_name(e));
    }
    return e;
}

/* H6.1: read-and-discard for `settle_ms` while keeping the cache
 * (raw / net / grams) live.  Used as a finger-release window before
 * TARE/CAL sample collection — the screen is mechanically coupled to
 * the load cell, so the few hundred ms of finger pressure on the button
 * would otherwise contaminate the average.  Each iteration is one
 * normal HX711 read (~100 ms), so ~10 reads in 1000 ms.  Cache stays
 * live so the UI's "raw / zero" line keeps ticking. */
static void hx711_settle(uint32_t settle_ms) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(settle_ms);
    while (xTaskGetTickCount() < deadline) {
        int32_t s;
        if (hx711_read_raw_blocking(&s, 200) != ESP_OK) continue;
        int32_t cur_net = s - s_tare_offset;
        float   cur_g   = s_cal_valid ? ((float)cur_net / s_cal_factor) : 0.0f;
        portENTER_CRITICAL(&s_cache_mux);
        s_latest_raw   = s;
        s_latest_net   = cur_net;
        s_latest_grams = cur_g;
        portEXIT_CRITICAL(&s_cache_mux);
    }
}

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
    int32_t  raw                = 0;
    int32_t  prev_raw           = 0x7FFFFFFF;   /* impossible 24-bit value */
    uint32_t stuck_count        = 0;            /* consecutive identical reads */
    uint32_t serial_log_counter = 0;            /* H4: throttle serial to ~1 Hz */
    ESP_LOGI(TAG, "owner task running on core %d @ ~10 Hz "
                  "(SCK=GPIO%d DOUT=GPIO%d, BSP_HX711_SWAP_PINS=%d)",
             (int)xPortGetCoreID(),
             s_sck, s_dout, (int)BSP_HX711_SWAP_PINS);

#if BSP_BOOT_AUTOTARE_ENABLE
    /* H7.1: boot auto-tare.  COMMERCIAL-SCALE NOTE: this assumes the
     * platform is empty at boot.  If an object is on the platform during
     * boot, that object becomes the new zero — same as many commercial
     * kitchen scales.  RAM-only; never written to NVS.  Runs once as
     * the first action of the owner task, before any notifications are
     * checked, so it cannot collide with manual TARE/CAL. */
    s_boot_autotare_in_progress = true;
    ESP_LOGI(TAG, "BOOT auto-tare — settling %d ms...",
             BSP_BOOT_AUTOTARE_SETTLE_MS);
    hx711_settle(BSP_BOOT_AUTOTARE_SETTLE_MS);
    ESP_LOGI(TAG, "BOOT auto-tare: collecting %d samples...",
             BSP_BOOT_AUTOTARE_SAMPLES);
    {
        int64_t bt_sum = 0;
        int     bt_n   = 0;
        for (int i = 0; i < BSP_BOOT_AUTOTARE_SAMPLES; ++i) {
            int32_t s;
            if (hx711_read_raw_blocking(&s, 200) != ESP_OK) continue;
            int32_t cur_net = s - s_tare_offset;
            float   cur_g   = s_cal_valid
                              ? ((float)cur_net / s_cal_factor) : 0.0f;
            portENTER_CRITICAL(&s_cache_mux);
            s_latest_raw   = s;
            s_latest_net   = cur_net;
            s_latest_grams = cur_g;
            portEXIT_CRITICAL(&s_cache_mux);
            bt_sum += s;
            bt_n++;
        }
        if (bt_n > 0) {
            int32_t new_offset = (int32_t)(bt_sum / bt_n);
            int32_t new_net    = s_latest_raw - new_offset;
            float   new_g      = s_cal_valid
                                 ? ((float)new_net / s_cal_factor) : 0.0f;
            /* H8: anchor filter at 0 so the boot platform shows STABLE
             * 0 g immediately when cal is loaded. */
            filter_reset(new_g);
            portENTER_CRITICAL(&s_cache_mux);
            s_tare_offset  = new_offset;
            s_latest_net   = new_net;
            s_latest_grams = new_g;
            s_stable       = true;
            portEXIT_CRITICAL(&s_cache_mux);
            s_have_sample = true;
            ESP_LOGI(TAG, "BOOT auto-tare applied: tare_offset=%ld "
                          "(avg of %d samples)",
                     (long)new_offset, bt_n);
        } else {
            ESP_LOGW(TAG, "BOOT auto-tare failed: zero successful samples "
                          "— continuing without tare (user can press TARE)");
        }
    }
    s_boot_autotare_in_progress = false;
#endif

    for (;;) {
        int  dout_pre = gpio_get_level(s_dout);
        bool ready    = (dout_pre == 0);

        esp_err_t r = hx711_read_raw_blocking(&raw, 200);
        if (r == ESP_OK) {
            /* H4 + H5 + H6 + H8: compute raw post-cal grams, then push
             * through the H8 filter pipeline to get the value that goes
             * on screen.  Publish (raw, net, filtered_grams, stable)
             * atomically. */
            int32_t net    = raw - s_tare_offset;
            float   g_raw  = s_cal_valid ? ((float)net / s_cal_factor) : 0.0f;
            bool    st     = false;
            float   g_filt = s_cal_valid ? filter_step(g_raw, &st) : 0.0f;
            portENTER_CRITICAL(&s_cache_mux);
            s_latest_raw   = raw;
            s_latest_net   = net;
            s_latest_grams = g_filt;
            s_stable       = st;
            portEXIT_CRITICAL(&s_cache_mux);
            s_have_sample = true;

            /* H4: throttle the per-sample ESP_LOGI to ~1 Hz so the serial
             * console is readable when the UI is doing the primary display.
             * Every 10th read = once per second at 10 Hz cadence. */
            if (++serial_log_counter >= 10) {
                if (s_cal_valid) {
                    ESP_LOGI(TAG, "raw=%ld  net=%ld  g_raw=%.1f  g_filt=%.1f  "
                                  "stable=%d  dout=%d ready=%d",
                             (long)raw, (long)net, (double)g_raw,
                             (double)g_filt, (int)st, dout_pre, (int)ready);
                } else {
                    ESP_LOGI(TAG, "raw=%ld  net=%ld  dout=%d ready=%d "
                                  "(not calibrated)",
                             (long)raw, (long)net, dout_pre, (int)ready);
                }
                serial_log_counter = 0;
            }

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

        /* H5 + H6: process any pending TARE / CAL notifications.
         * Non-blocking poll (timeout = 0) — never stalls the read cadence
         * when no request is pending. */
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notif, 0) == pdPASS) {
            if (notif & HX711_NOTIFY_TARE_BIT) {
                s_tare_in_progress = true;
                ESP_LOGI(TAG, "TARE requested — release scale, settling "
                              "%d ms...", BSP_TARE_SETTLE_MS);
                hx711_settle(BSP_TARE_SETTLE_MS);
                ESP_LOGI(TAG, "TARE: collecting %d samples for averaging...",
                              HX711_TARE_SAMPLES);
                int64_t sum = 0;
                int     n   = 0;
                int32_t tare_min = INT32_MAX;
                int32_t tare_max = INT32_MIN;
                for (int i = 0; i < HX711_TARE_SAMPLES; ++i) {
                    int32_t s;
                    if (hx711_read_raw_blocking(&s, 200) != ESP_OK) continue;
                    /* Keep the cache flowing during collection so the UI
                     * doesn't visibly freeze for ~1 s while we tare. */
                    int32_t cur_net = s - s_tare_offset;
                    float   cur_g   = s_cal_valid
                                      ? ((float)cur_net / s_cal_factor) : 0.0f;
                    portENTER_CRITICAL(&s_cache_mux);
                    s_latest_raw   = s;
                    s_latest_net   = cur_net;       /* old offset */
                    s_latest_grams = cur_g;
                    portEXIT_CRITICAL(&s_cache_mux);
                    sum += s;
                    n++;
                    if (s < tare_min) tare_min = s;
                    if (s > tare_max) tare_max = s;
                }
                if (n == 0) {
                    ESP_LOGW(TAG, "TARE failed: zero successful samples");
                } else {
                    int32_t span = tare_max - tare_min;
                    if (span > BSP_TARE_MAX_SPAN_COUNTS) {
                        /* H8.1: window too noisy — reject, don't touch
                         * tare_offset or filter.  User should release
                         * the screen / let the platform settle and try
                         * again. */
                        ESP_LOGW(TAG, "TARE rejected: unstable samples "
                                      "span=%ld counts (>%d) — previous "
                                      "tare preserved.  Release the "
                                      "platform and try again.",
                                 (long)span, BSP_TARE_MAX_SPAN_COUNTS);
                    } else {
                        int32_t new_offset = (int32_t)(sum / n);
                        int32_t new_net    = s_latest_raw - new_offset;
                        float   new_g      = s_cal_valid
                                             ? ((float)new_net / s_cal_factor) : 0.0f;
                        /* H8: reset the display filter to 0 so the new zero
                         * is STABLE immediately and not treated as an outlier
                         * jump from the pre-tare value. */
                        filter_reset(new_g);
                        portENTER_CRITICAL(&s_cache_mux);
                        s_tare_offset  = new_offset;
                        s_latest_net   = new_net;
                        s_latest_grams = new_g;
                        s_stable       = true;
                        portEXIT_CRITICAL(&s_cache_mux);
                        ESP_LOGI(TAG, "TARE applied: tare_offset=%ld "
                                      "span=%ld counts (avg of %d samples)",
                                 (long)new_offset, (long)span, n);
                    }
                }
                s_tare_in_progress = false;
                serial_log_counter = 0;  /* force next normal log soon */
            }
            if (notif & HX711_NOTIFY_CAL_BIT) {
                s_cal_in_progress = true;
                float known = s_cal_known_grams;
                ESP_LOGI(TAG, "CAL requested — known_weight=%.1fg, release "
                              "button, settling %d ms...",
                              (double)known, BSP_CAL_SETTLE_MS);
                hx711_settle(BSP_CAL_SETTLE_MS);
                ESP_LOGI(TAG, "CAL: collecting %d samples for averaging...",
                              HX711_CAL_SAMPLES);
                int64_t sum_net = 0;
                int     n       = 0;
                int32_t net_min = INT32_MAX;
                int32_t net_max = INT32_MIN;
                for (int i = 0; i < HX711_CAL_SAMPLES; ++i) {
                    int32_t s;
                    if (hx711_read_raw_blocking(&s, 200) != ESP_OK) continue;
                    int32_t cur_net = s - s_tare_offset;
                    float   cur_g   = s_cal_valid
                                      ? ((float)cur_net / s_cal_factor) : 0.0f;
                    portENTER_CRITICAL(&s_cache_mux);
                    s_latest_raw   = s;
                    s_latest_net   = cur_net;
                    s_latest_grams = cur_g;
                    portEXIT_CRITICAL(&s_cache_mux);
                    sum_net += cur_net;
                    n++;
                    if (cur_net < net_min) net_min = cur_net;
                    if (cur_net > net_max) net_max = cur_net;
                }
                if (n == 0) {
                    ESP_LOGW(TAG, "CAL failed: zero successful samples — "
                                  "previous calibration state preserved");
                } else if (known <= 0.0f) {
                    ESP_LOGW(TAG, "CAL rejected: known_grams=%.1f must be > 0 "
                                  "— previous calibration state preserved",
                             (double)known);
                } else if ((net_max - net_min) > BSP_CAL_MAX_SPAN_COUNTS) {
                    /* H8.1: too much motion during collection — reject
                     * BEFORE computing the factor, so we never overwrite
                     * a good NVS value with a bad one. */
                    ESP_LOGW(TAG, "CAL rejected: unstable samples "
                                  "span=%ld counts (>%d) — previous "
                                  "calibration preserved.  Wait for the "
                                  "STABLE indicator before tapping CAL.",
                             (long)(net_max - net_min),
                             BSP_CAL_MAX_SPAN_COUNTS);
                } else {
                    int64_t avg_net = sum_net / n;
                    int64_t abs_avg = avg_net < 0 ? -avg_net : avg_net;
                    if (abs_avg < HX711_CAL_MIN_ABS_NET) {
                        ESP_LOGW(TAG, "CAL rejected: |avg_net|=%lld < %d "
                                      "(no meaningful load on cell) — "
                                      "previous calibration state preserved. "
                                      "Place the known weight and try again.",
                                 (long long)abs_avg, HX711_CAL_MIN_ABS_NET);
                    } else {
                        float new_factor = (float)avg_net / known;
                        float new_g      = (float)s_latest_net / new_factor;
                        /* H8: anchor the filter at the just-measured
                         * grams (≈ known load) so the next live samples
                         * are not flagged as outliers vs the previous
                         * (possibly uncalibrated) EMA. */
                        filter_reset(new_g);
                        portENTER_CRITICAL(&s_cache_mux);
                        s_cal_factor   = new_factor;
                        s_cal_valid    = true;
                        s_latest_grams = new_g;
                        s_stable       = true;
                        portEXIT_CRITICAL(&s_cache_mux);
                        ESP_LOGI(TAG, "CAL applied: factor=%.4f counts/g, "
                                      "avg_net=%lld span=%ld counts "
                                      "(avg of %d samples, known=%.1fg)",
                                 (double)new_factor, (long long)avg_net,
                                 (long)(net_max - net_min), n,
                                 (double)known);
                        /* H7: persist immediately so a reboot restores
                         * the calibration.  Failure logs but does not
                         * disturb the live RAM cal.  H8.1: only reached
                         * after span check passed — bad windows never
                         * touch NVS. */
                        (void)cal_save_to_nvs(new_factor, known);
                    }
                }
                s_cal_in_progress = false;
                serial_log_counter = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool hx711_get_latest_raw(int32_t *out_raw) {
    if (out_raw == NULL || !s_have_sample) return false;
    *out_raw = s_latest_raw;   /* 32-bit aligned volatile read — atomic */
    return true;
}

bool hx711_get_snapshot(int32_t *out_raw, int32_t *out_net) {
    if (!s_have_sample) return false;
    portENTER_CRITICAL(&s_cache_mux);
    if (out_raw) *out_raw = s_latest_raw;
    if (out_net) *out_net = s_latest_net;
    portEXIT_CRITICAL(&s_cache_mux);
    return true;
}

bool hx711_get_snapshot_full(int32_t *out_raw, int32_t *out_net,
                              float *out_grams, bool *out_cal_valid,
                              bool *out_stable) {
    if (!s_have_sample) return false;
    portENTER_CRITICAL(&s_cache_mux);
    if (out_raw)       *out_raw       = s_latest_raw;
    if (out_net)       *out_net       = s_latest_net;
    if (out_grams)     *out_grams     = s_latest_grams;
    if (out_cal_valid) *out_cal_valid = s_cal_valid;
    if (out_stable)    *out_stable    = s_stable;
    portEXIT_CRITICAL(&s_cache_mux);
    return true;
}

esp_err_t hx711_request_tare(void) {
    if (!s_task_started || s_owner_task == NULL) return ESP_ERR_INVALID_STATE;
    if (s_boot_autotare_in_progress) {
        ESP_LOGW(TAG, "TARE ignored — boot auto-tare in progress");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tare_in_progress || s_cal_in_progress) {
        ESP_LOGW(TAG, "TARE ignored — already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    xTaskNotify(s_owner_task, HX711_NOTIFY_TARE_BIT, eSetBits);
    return ESP_OK;
}

esp_err_t hx711_request_calibrate(float known_grams) {
    if (!s_task_started || s_owner_task == NULL) return ESP_ERR_INVALID_STATE;
    if (known_grams <= 0.0f) return ESP_ERR_INVALID_ARG;
    if (s_boot_autotare_in_progress) {
        ESP_LOGW(TAG, "CAL ignored — boot auto-tare in progress");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tare_in_progress || s_cal_in_progress) {
        ESP_LOGW(TAG, "CAL ignored — already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    s_cal_known_grams = known_grams;   /* atomic 32-bit store on ESP32-S3 */
    xTaskNotify(s_owner_task, HX711_NOTIFY_CAL_BIT, eSetBits);
    return ESP_OK;
}

esp_err_t hx711_owner_start(void) {
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (s_task_started) return ESP_OK;

    /* H7: load saved calibration (if any) BEFORE the task spawns so the
     * very first cached sample already has correct grams.  Soft-failure:
     * any error path simply leaves us uncalibrated.  Never blocks boot. */
    (void)cal_load_from_nvs();

    BaseType_t ok = xTaskCreatePinnedToCore(
        hx711_task,
        "hx711_owner",
        4096,                 /* stack */
        NULL,                 /* arg */
        3,                    /* priority — below LVGL's 4 */
        &s_owner_task,        /* H5: capture handle for xTaskNotify */
        0);                   /* CPU0 — LVGL lives on CPU1 */
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return ESP_ERR_NO_MEM;
    }
    s_task_started = true;
    return ESP_OK;
}
