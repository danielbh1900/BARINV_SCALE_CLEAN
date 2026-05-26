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

#include "driver/gpio.h"
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

/* H9.0.1: dedicated poll timer that only runs while we're in
 * SOFT_STANDBY.  Reads GPIO16 (touch INT) every BSP_STANDBY_INT_POLL_MS
 * ms and wakes the system if the line is asserted low.  Costs zero
 * when ACTIVE — timer is stopped on wake and re-started on next sleep. */
static esp_timer_handle_t   s_standby_poll_timer = NULL;

/* H9.0.1: wake-tap window.  Set to esp_timer_get_time() on every
 * SOFT_STANDBY → ACTIVE transition.  power_mgr_consume_wake_tap()
 * checks (now - this) < BSP_WAKE_TAP_WINDOW_MS and consumes if so.
 * Zero means "no pending wake tap". */
static volatile int64_t     s_wake_tap_window_us = 0;

static void enter_soft_standby(void) {
    s_state = PM_SOFT_STANDBY;
    (void)bsp_backlight_set(BSP_STANDBY_BACKLIGHT_DUTY);
    ESP_LOGI(TAG, "entering SOFT_STANDBY after %d ms idle "
                  "(backlight duty -> %d; GPIO%d poll @ %d ms)",
             BSP_IDLE_TO_STANDBY_MS, BSP_STANDBY_BACKLIGHT_DUTY,
             BSP_TOUCH_PIN_INT, BSP_STANDBY_INT_POLL_MS);
    /* H9.0.1: start the dedicated GPIO16 poll timer.  Safe to call
     * even if already running — esp_timer_start_periodic returns
     * ESP_ERR_INVALID_STATE which we ignore. */
    if (s_standby_poll_timer) {
        (void)esp_timer_start_periodic(
            s_standby_poll_timer,
            (uint64_t)BSP_STANDBY_INT_POLL_MS * 1000ULL);
    }
}

/* Reason argument distinguishes the two wake paths for the log line. */
static void exit_soft_standby(const char *reason) {
    /* Idempotent: races between GPIO poll wake and LVGL press wake are
     * fine — whichever lands first does the transition, the other is a
     * no-op. */
    if (s_state == PM_ACTIVE) return;
    s_state = PM_ACTIVE;
    s_wake_tap_window_us = esp_timer_get_time();   /* arm wake-tap gate */
    s_last_activity_us   = s_wake_tap_window_us;   /* reset idle clock */
    (void)bsp_backlight_set(BSP_ACTIVE_BACKLIGHT_DUTY);
    if (s_standby_poll_timer) {
        (void)esp_timer_stop(s_standby_poll_timer);
    }
    ESP_LOGI(TAG, "wake requested by %s — ACTIVE "
                  "(backlight duty -> %d; tap window %d ms)",
             reason, BSP_ACTIVE_BACKLIGHT_DUTY, BSP_WAKE_TAP_WINDOW_MS);
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

/* H9.0.1: standby-only GPIO16 poll.  Runs ONLY when SOFT_STANDBY is
 * active (the timer is stopped on wake).  When CST820 / CST816S
 * asserts INT (active LOW) for any reason — a real touch OR the
 * chip's own auto-sleep wake spike — we immediately bring the
 * backlight up.  This is the path that fixes the "second tap needed"
 * bug from H9.0 where the LVGL+CST820 stack sometimes consumed the
 * first tap without delivering an event. */
static void standby_int_poll_cb(void *arg) {
    (void)arg;
    if (s_state != PM_SOFT_STANDBY) return;
    if (gpio_get_level(BSP_TOUCH_PIN_INT) == 0) {
        exit_soft_standby("GPIO touch INT");
    }
}

void power_mgr_register_activity(void) {
    s_last_activity_us = esp_timer_get_time();
    if (s_state == PM_SOFT_STANDBY) {
        exit_soft_standby("LVGL press");
    }
}

bool power_mgr_is_standby(void) {
    return s_state == PM_SOFT_STANDBY;
}

bool power_mgr_consume_wake_tap(void) {
    int64_t armed = s_wake_tap_window_us;
    if (armed == 0) return false;
    int64_t now = esp_timer_get_time();
    if ((now - armed) <= (int64_t)BSP_WAKE_TAP_WINDOW_MS * 1000) {
        s_wake_tap_window_us = 0;   /* consumed */
        return true;
    }
    /* Window expired without a click landing — clear so future clicks
     * are normal. */
    s_wake_tap_window_us = 0;
    return false;
}

esp_err_t power_mgr_init(void) {
    s_last_activity_us = esp_timer_get_time();
    s_state            = PM_ACTIVE;

    const esp_timer_create_args_t idle_args = {
        .callback = idle_check_cb,
        .name     = "pm_idle",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&idle_args, &s_idle_timer),
                        TAG, "esp_timer_create (idle) failed");
    ESP_RETURN_ON_ERROR(
        esp_timer_start_periodic(s_idle_timer, 1000ULL * 1000ULL /* 1 s */),
        TAG, "esp_timer_start_periodic (idle) failed");

    /* H9.0.1: create the standby GPIO16 poll timer but DO NOT start
     * it yet — it only runs while SOFT_STANDBY (started by
     * enter_soft_standby, stopped by exit_soft_standby). */
    const esp_timer_create_args_t poll_args = {
        .callback = standby_int_poll_cb,
        .name     = "pm_int",
    };
    ESP_RETURN_ON_ERROR(
        esp_timer_create(&poll_args, &s_standby_poll_timer),
        TAG, "esp_timer_create (standby poll) failed");

    ESP_LOGI(TAG, "ACTIVE — idle->SOFT_STANDBY after %d ms; "
                  "wake via LVGL press OR GPIO%d poll @ %d ms; "
                  "wake-tap window %d ms; HX711 stays at 10 Hz",
             BSP_IDLE_TO_STANDBY_MS, BSP_TOUCH_PIN_INT,
             BSP_STANDBY_INT_POLL_MS, BSP_WAKE_TAP_WINDOW_MS);
    return ESP_OK;
}
