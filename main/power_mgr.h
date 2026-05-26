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

// True iff currently in SOFT_STANDBY.  Cheap atomic read.  Use this
// from UI event callbacks to decide whether the current tap should be
// consumed as a wake gesture (and thus NOT trigger the underlying
// button action).
bool power_mgr_is_standby(void);
