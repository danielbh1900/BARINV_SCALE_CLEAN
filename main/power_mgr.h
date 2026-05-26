// power_mgr.h — H9.0 soft-standby power state machine.
//
// State model:
//   ACTIVE         — normal: backlight on, full UI, 10 Hz weighing
//   SOFT_STANDBY   — display backlight = 0; LVGL, touch, HX711 all
//                    still running.  Any touch wakes back to ACTIVE.
//
// Scope of H9.0:
//   * NO deep sleep (planned for H9.1)
//   * NO HX711 rate change (planned for H9.1)
//   * NO Wi-Fi / BLE / OTA touched
//   * Hardware single-owner contract preserved — power_mgr only calls
//     bsp_backlight_set(); it never touches HX711, ST7701, CST820,
//     LVGL internals, or the panel framebuffer.

#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Spawn the idle-watchdog esp_timer and put the system into ACTIVE.
// Must be called after bsp_backlight_init() and bsp_backlight_on().
esp_err_t power_mgr_init(void);

// Reset the idle timer.  If currently SOFT_STANDBY, transition back to
// ACTIVE (backlight snaps to BSP_ACTIVE_BACKLIGHT_DUTY).  Safe to call
// from any task or LVGL event callback.
void power_mgr_register_activity(void);

// True iff currently in SOFT_STANDBY.  Cheap atomic read.
bool power_mgr_is_standby(void);

// H9.6: blanket lockout of UI actions immediately after a wake from
// SOFT_STANDBY.  Returns TRUE for BSP_WAKE_TOUCH_GUARD_MS after the
// most recent SOFT_STANDBY → ACTIVE transition.  Unlike
// power_mgr_consume_wake_tap(), this does NOT consume the timestamp
// — multiple checks during the window all return TRUE.  UI button
// CLICKED handlers should call this FIRST and early-return on TRUE.
//
// Replaces the H9.0.1 single-shot consume pattern, which had a race
// where multiple click events from a single touch (LVGL sometimes
// dispatches two CLICKEDs ~100 ms apart for a sustained press) could
// leak through after the first consume cleared the flag.
bool power_mgr_ui_actions_blocked(void);

// H9.0.1: check-and-consume the wake-tap flag.
//   * Returns TRUE if the system just transitioned from SOFT_STANDBY
//     to ACTIVE within the last BSP_WAKE_TAP_WINDOW_MS milliseconds
//     AND this is the first call within that window.  The caller (UI
//     button CLICKED handler) should early-return without executing
//     its action — this tap was the wake gesture.
//   * Returns FALSE otherwise (normal click, proceed with action).
//   * Side effect: clears the flag on TRUE return.
//   * Thread-safe.  Cheap.  Bounded — flag auto-expires after the
//     wake-tap window, so a stale wake from a non-button standby tap
//     does not eat a later real button click.
bool power_mgr_consume_wake_tap(void);
