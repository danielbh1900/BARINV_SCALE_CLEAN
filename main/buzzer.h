// buzzer.h — non-blocking buzzer feedback for TARE / CAL lifecycle.
//
// Hardware: active buzzer wired to TCA9554 expander EXIO8
// (BSP_BUZZER_VIA_EXIO_BIT).  Each transition is one ~1 ms I2C write
// to the expander — bounded and predictable.
//
// All buzzer_*() functions below are NON-BLOCKING.  They post a
// pattern enum to a small queue and return in microseconds.  A
// dedicated low-priority task drains the queue and plays patterns
// using vTaskDelay between transitions.
//
// If BSP_BUZZER_ENABLE = 0, every function is a no-op (compiled-out).
//
// MUST be called after bsp_tca9554_init() succeeded.
#pragma once

#include "esp_err.h"

esp_err_t buzzer_init(void);

// Optional very short chirp on accepted button tap.  Confirms the
// button was registered — does NOT mean the action has completed.
void buzzer_tap(void);

// TARE finished and was applied (offset committed).
void buzzer_success_tare(void);

// CAL passed verification, factor installed, NVS write OK.
void buzzer_success_cal(void);

// TARE/CAL rejected at ANY layer (precheck, hold, span, verify, …).
void buzzer_error(void);

// Low-priority feedback for cooldown / already-in-progress refusals.
// Currently routed to a short chirp; may be no-op in some builds.
void buzzer_busy(void);
