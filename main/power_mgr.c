// power_mgr.c — H9.0 soft-standby implementation.
//
// Idle detection: a 1 Hz esp_timer checks (now - last_activity).
// When it exceeds BSP_IDLE_TO_STANDBY_MS the backlight is snapped to
// BSP_STANDBY_BACKLIGHT_DUTY (= 0).  Any call to
// power_mgr_register_activity() resets the clock and, if we were in
// standby, snaps the backlight back to BSP_ACTIVE_BACKLIGHT_DUTY.
//
// Snap (no fade) is intentional for H9.0 — keeps the change small and
// avoids running a blocking fade loop inside the esp_timer task.  A
// hardware-LEDC fade can land in H9.0.1 without touching this file's
// state machine.

#include "power_mgr.h"
#include "board_config.h"
#include "backlight.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

static const char *TAG = "power_mgr";

typedef enum {
    PM_ACTIVE,
    PM_SOFT_STANDBY,
} pm_state_t;

static volatile pm_state_t  s_state              = PM_ACTIVE;
static volatile int64_t     s_last_activity_us   = 0;
static esp_timer_handle_t   s_idle_timer         = NULL;

static void enter_soft_standby(void) {
    s_state = PM_SOFT_STANDBY;
    (void)bsp_backlight_set(BSP_STANDBY_BACKLIGHT_DUTY);
    ESP_LOGI(TAG, "entering SOFT_STANDBY after %d ms idle "
                  "(backlight duty -> %d)",
             BSP_IDLE_TO_STANDBY_MS, BSP_STANDBY_BACKLIGHT_DUTY);
}

static void exit_soft_standby(void) {
    s_state = PM_ACTIVE;
    (void)bsp_backlight_set(BSP_ACTIVE_BACKLIGHT_DUTY);
    ESP_LOGI(TAG, "wake requested by touch — ACTIVE "
                  "(backlight duty -> %d)",
             BSP_ACTIVE_BACKLIGHT_DUTY);
}

/* esp_timer task — runs every 1 s.  Keep work tiny here — no blocking,
 * no log spam unless transitioning. */
static void idle_check_cb(void *arg) {
    (void)arg;
    if (s_state != PM_ACTIVE) return;
    int64_t now_us  = esp_timer_get_time();
    int64_t idle_ms = (now_us - s_last_activity_us) / 1000;
    if (idle_ms >= (int64_t)BSP_IDLE_TO_STANDBY_MS) {
        enter_soft_standby();
    }
}

void power_mgr_register_activity(void) {
    s_last_activity_us = esp_timer_get_time();
    if (s_state == PM_SOFT_STANDBY) {
        exit_soft_standby();
    }
}

bool power_mgr_is_standby(void) {
    return s_state == PM_SOFT_STANDBY;
}

esp_err_t power_mgr_init(void) {
    s_last_activity_us = esp_timer_get_time();
    s_state            = PM_ACTIVE;

    const esp_timer_create_args_t args = {
        .callback = idle_check_cb,
        .name     = "pm_idle",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_idle_timer),
                        TAG, "esp_timer_create failed");
    ESP_RETURN_ON_ERROR(
        esp_timer_start_periodic(s_idle_timer, 1000ULL * 1000ULL /* 1 s */),
        TAG, "esp_timer_start_periodic failed");

    ESP_LOGI(TAG, "ACTIVE — idle->SOFT_STANDBY after %d ms; "
                  "snap (no fade); HX711 stays at 10 Hz",
             BSP_IDLE_TO_STANDBY_MS);
    return ESP_OK;
}
