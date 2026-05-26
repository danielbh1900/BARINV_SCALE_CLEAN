// ============================================================================
// board_config.h — Waveshare ESP32-S3-Touch-LCD-2.1 immutable hardware contract.
//
// Every value below is verbatim from the official Waveshare factory ESP-IDF
// + Arduino demo source (delivered in `_reference/`) and cross-confirmed
// against Espressif's Waveshare-authored board header
// `BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_2_1.h`.
//
// DO NOT EDIT without a corresponding update to the validation report.
// ============================================================================
#pragma once

// ---- Panel geometry --------------------------------------------------------
#define BSP_LCD_H_RES                480
#define BSP_LCD_V_RES                480

// ---- RGB timing ------------------------------------------------------------
#define BSP_LCD_RGB_PCLK_HZ          (16 * 1000 * 1000)
#define BSP_LCD_RGB_HSYNC_PULSE      8
#define BSP_LCD_RGB_HSYNC_BACK       10
#define BSP_LCD_RGB_HSYNC_FRONT      50
#define BSP_LCD_RGB_VSYNC_PULSE      3
#define BSP_LCD_RGB_VSYNC_BACK       8
#define BSP_LCD_RGB_VSYNC_FRONT      8
#define BSP_LCD_RGB_FRAME_BUF_NUM    2
#define BSP_LCD_RGB_BOUNCE_BUF_PX    (BSP_LCD_H_RES * 10)

// ---- RGB sync + clock pins -------------------------------------------------
#define BSP_LCD_PIN_HSYNC            38
#define BSP_LCD_PIN_VSYNC            39
#define BSP_LCD_PIN_DE               40
#define BSP_LCD_PIN_PCLK             41
#define BSP_LCD_PIN_DISP             (-1)

// ---- RGB565 data lines (DATA0..DATA15, in panel-stream order) -------------
#define BSP_LCD_PIN_DATA0            5
#define BSP_LCD_PIN_DATA1            45
#define BSP_LCD_PIN_DATA2            48
#define BSP_LCD_PIN_DATA3            47
#define BSP_LCD_PIN_DATA4            21
#define BSP_LCD_PIN_DATA5            14
#define BSP_LCD_PIN_DATA6            13
#define BSP_LCD_PIN_DATA7            12
#define BSP_LCD_PIN_DATA8            11
#define BSP_LCD_PIN_DATA9            10
#define BSP_LCD_PIN_DATA10           9
#define BSP_LCD_PIN_DATA11           46
#define BSP_LCD_PIN_DATA12           3
#define BSP_LCD_PIN_DATA13           8
#define BSP_LCD_PIN_DATA14           18
#define BSP_LCD_PIN_DATA15           17

// ---- ST7701 3-wire init bus (CS lives on the expander) --------------------
#define BSP_LCD_SPI_PIN_SCK          2
#define BSP_LCD_SPI_PIN_MOSI         1

// ---- Backlight -------------------------------------------------------------
#define BSP_LCD_BACKLIGHT_PIN        6
#define BSP_LCD_BACKLIGHT_ON_LEVEL   1
#define BSP_LCD_BACKLIGHT_LEDC_CH    LEDC_CHANNEL_1
#define BSP_LCD_BACKLIGHT_LEDC_TIMER LEDC_TIMER_0
#define BSP_LCD_BACKLIGHT_FREQ_HZ    20000
#define BSP_LCD_BACKLIGHT_RES_BITS   LEDC_TIMER_10_BIT
#define BSP_LCD_BACKLIGHT_DEFAULT    800   // 0..1023, ~78% brightness

// ---- I2C0 (touch + TCA9554) ------------------------------------------------
#define BSP_I2C_NUM                  0
#define BSP_I2C_PIN_SDA              15
#define BSP_I2C_PIN_SCL              7
#define BSP_I2C_HZ                   400000

// ---- TCA9554PWR I/O expander ----------------------------------------------
#define BSP_TCA9554_ADDR             0x20
// EXIO assignments (1-indexed to match factory; 0-indexed in the IDF
// expander API is `(bit - 1)`).
#define BSP_EXIO_LCD_RST_BIT         1
#define BSP_EXIO_TP_RST_BIT          2
#define BSP_EXIO_LCD_CS_BIT          3
#define BSP_EXIO_SD_CS_BIT           4
#define BSP_EXIO_BUZZER_BIT          8
// IO-expander pin mask helpers (0-indexed bit positions).
#define BSP_EXIO_PIN(idx1)           ((idx1) - 1)
#define BSP_EXIO_MASK(idx1)          (1U << BSP_EXIO_PIN(idx1))

// ---- Touch — CST820 (CST816S-protocol compatible) -------------------------
#define BSP_TOUCH_ADDR               0x15
#define BSP_TOUCH_PIN_INT            16
#define BSP_TOUCH_PIN_RST            (-1)   // routed through TCA9554 EXIO2

// ---- SD card (LATER; pins recorded so Step 2 inherits cleanly) ------------
#define BSP_SD_PIN_MISO              42
#define BSP_SD_PIN_MOSI              1     // shared with ST7701 init MOSI
#define BSP_SD_PIN_CLK               2     // shared with ST7701 init CLK
#define BSP_SD_CS_VIA_EXIO_BIT       4     // EXIO4 controls SD CS

// ---- Buzzer (TCA9554 EXIO8, on/off active buzzer) -------------------------
#define BSP_BUZZER_VIA_EXIO_BIT      8

// H9.4: enable/disable the buzzer feedback module at build time.
// Set 0 to compile-out all buzzer code (the buzzer_*() API becomes
// no-ops returning ESP_OK).  Pin is fixed by hardware on TCA9554
// EXIO8 — no GPIO guessing required.
#ifndef BSP_BUZZER_ENABLE
#define BSP_BUZZER_ENABLE            1
#endif

// ---- HX711 24-bit ADC (Phase 2; bit-bang via GPIO, no SPI peripheral) -----
//  Console is on native USB-Serial-JTAG (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y,
//  USB_CDC=n), so the default UART0 TX=GPIO43 / RX=GPIO44 pins are NOT taken
//  by the console and are free for the HX711.
//  Default mapping mirrors the standalone HX711_SCALE_LAB project.
//  If SCK/DOUT turn out reversed on the assembled hardware, define
//  BSP_HX711_SWAP_PINS=1 in sdkconfig.defaults or via -D at build time —
//  no source edit required.
#ifndef BSP_HX711_SWAP_PINS
#define BSP_HX711_SWAP_PINS          1
#endif
#if BSP_HX711_SWAP_PINS
  #define BSP_HX711_SCK              44
  #define BSP_HX711_DOUT             43
#else
  #define BSP_HX711_SCK              43
  #define BSP_HX711_DOUT             44
#endif

// H6 calibration:  known reference weight used by the CAL button.
// Tap TARE on empty pan, place exactly this mass, tap CAL.  The owner
// task averages 16 net samples and stores cal_factor = avg_net / grams.
// RAM-only — no NVS yet.  Change at build time only.
#ifndef BSP_CAL_WEIGHT_GRAMS
#define BSP_CAL_WEIGHT_GRAMS         500.0f
#endif

// H6.1: post-tap settling delay before TARE/CAL sample collection begins.
// The screen is mechanically coupled to the load cell, so a finger press
// on the TARE / CAL button briefly biases the reading.  The owner task
// reads-and-discards for this many milliseconds (keeping the cache live
// so the UI doesn't freeze) before starting the actual averaging window.
// H9.3.2: bumped from 1200 → 3000 ms.  Field testing showed 1.2 s was
// insufficient to absorb finger pressure on this assembly — the
// post-settle stability flag was still in transition when prechecks
// ran.  3 s is a commercial-scale-grade settle window.
#ifndef BSP_TARE_SETTLE_MS
#define BSP_TARE_SETTLE_MS           3000
#endif
#ifndef BSP_CAL_SETTLE_MS
#define BSP_CAL_SETTLE_MS            3000
#endif

// H7.1: boot auto-tare.  Mirrors commercial kitchen-scale behavior — the
// scale wakes up showing zero so the operator can just start weighing.
// Runs ONCE at owner-task startup, RAM-only (never written to NVS),
// blocks no other subsystem.  If an object is already on the platform at
// power-on, that object's weight becomes the new zero (same trade-off as
// most commercial scales).  No object-presence detection is performed
// here; that would be a later (H8+) addition only if needed.
#ifndef BSP_BOOT_AUTOTARE_ENABLE
#define BSP_BOOT_AUTOTARE_ENABLE     1
#endif
#ifndef BSP_BOOT_AUTOTARE_SETTLE_MS
#define BSP_BOOT_AUTOTARE_SETTLE_MS  2000   // HX711 post-power-on settle
#endif
#ifndef BSP_BOOT_AUTOTARE_SAMPLES
#define BSP_BOOT_AUTOTARE_SAMPLES    16
#endif

// H8: weight-display filter (outlier reject + EMA + stability window).
//   * Outlier reject compares each new grams sample to the LAST ACCEPTED
//     raw sample (not the EMA) — this avoids the catch-up starvation
//     where a converging EMA keeps every fresh sample looking like an
//     outlier.  An isolated spike >threshold is ignored; a persistent
//     change (≥2 consecutive over-threshold samples) is accepted.
//   * EMA(alpha) smooths the accepted samples.  alpha=0.4 → 90% in
//     ~5 samples (~500 ms at 10 Hz).
//   * Stability window stores the last N filtered samples; STABLE goes
//     true when max-min < threshold across the full window.  After
//     TARE/CAL/auto-tare, the window is pre-filled with the new zero
//     so STABLE returns true immediately.
#ifndef BSP_STABLE_WINDOW_SAMPLES
#define BSP_STABLE_WINDOW_SAMPLES    8
#endif
#ifndef BSP_STABLE_THRESHOLD_GRAMS
#define BSP_STABLE_THRESHOLD_GRAMS   3.0f   /* H8.1: bumped from 2.0 for noise */
#endif
#ifndef BSP_OUTLIER_THRESHOLD_GRAMS
#define BSP_OUTLIER_THRESHOLD_GRAMS  80.0f
#endif
#ifndef BSP_FILTER_EMA_ALPHA
#define BSP_FILTER_EMA_ALPHA         0.4f
#endif

// H8.1: step-confirmation gate.  When a sample crosses the outlier
// threshold, it is held as a pending candidate.  Only after
// BSP_STEP_CONFIRM_SAMPLES additional consecutive samples are within
// BSP_STEP_CONFIRM_THRESHOLD_GRAMS of the (running-averaged) candidate
// do we accept it as a real step.  This rejects 1- AND 2-sample
// glitches (the previous H8 logic only rejected 1-sample glitches).
// Total accept latency = 1 pending + N confirmations = 200-300 ms.
#ifndef BSP_STEP_CONFIRM_THRESHOLD_GRAMS
#define BSP_STEP_CONFIRM_THRESHOLD_GRAMS  25.0f
#endif
#ifndef BSP_STEP_CONFIRM_SAMPLES
#define BSP_STEP_CONFIRM_SAMPLES          2
#endif

// H8.1: TARE/CAL sample-window quality guards.  Reject the operation
// if the collection window is too noisy (someone bumped the platform,
// finger still on screen, weight sliding, etc.).  Span is computed in
// raw HX711 counts so the same number works whether calibration exists
// or not.  Previous tare/cal state is preserved on rejection.
// H9.3: tightened from prior H8.1 values (3000 / 5000) after field tests
// showed cal_factor drift across repeated CAL attempts on physically
// unstable conditions.  Tighter span guards reject those windows before
// they can overwrite a good NVS calibration.
#ifndef BSP_TARE_MAX_SPAN_COUNTS
#define BSP_TARE_MAX_SPAN_COUNTS     1500   /* was 3000 in H8.1 */
#endif
#ifndef BSP_CAL_MAX_SPAN_COUNTS
#define BSP_CAL_MAX_SPAN_COUNTS      1500   /* was 5000 in H8.1 */
#endif

// H9.3: CAL request prechecks (run on the UI thread inside
// hx711_request_calibrate, BEFORE the owner-task notification is
// posted).  Cheap fast-fail layer in addition to the existing
// post-collection span check.
//
//   * If calibration already exists, require the H8 stability flag
//     and require |current_grams - known_grams| <= known_grams *
//     BSP_CAL_TOLERANCE_FRACTION.  For known=500 g, 10 % == ±50 g, so
//     CAL is accepted only if the live reading is between 450 g and
//     550 g — protects against "user tapped CAL with nothing on the
//     pan" overwriting NVS with a bogus factor.
//   * If no calibration yet (initial setup), no precheck — the
//     post-collection span guard is the only protection during the
//     first CAL.
//   * BSP_CAL_COOLDOWN_MS rejects repeat CAL taps for that long after
//     ANY request (accepted or rejected) — prevents NVS hammering
//     from a stuck button or impatient user.
#ifndef BSP_CAL_TOLERANCE_FRACTION
#define BSP_CAL_TOLERANCE_FRACTION   0.10f
#endif
#ifndef BSP_CAL_COOLDOWN_MS
#define BSP_CAL_COOLDOWN_MS          5000   /* H9.3.2: was 3000 */
#endif

// H9.3.2: stability-hold gate.  After the per-settle precheck passes,
// the owner task takes BSP_ACTION_STABLE_HOLD_SAMPLES fresh raw reads
// in a row.  All must succeed, and (max-min) must be ≤
// BSP_ACTION_STABLE_HOLD_MAX_DRIFT_COUNTS.  Catches "transiently
// stable, drifting underneath" conditions that a single-snapshot
// precheck would let through.  Raw counts (not grams) because raw is
// meaningful even when uncalibrated.
#ifndef BSP_ACTION_STABLE_HOLD_SAMPLES
#define BSP_ACTION_STABLE_HOLD_SAMPLES         5
#endif
#ifndef BSP_ACTION_STABLE_HOLD_MAX_DRIFT_COUNTS
#define BSP_ACTION_STABLE_HOLD_MAX_DRIFT_COUNTS  300
#endif

// H9.3.2: post-CAL verification.  After collection + span pass, the
// owner task computes the candidate factor (avg_net / known) but does
// NOT install it yet.  It takes BSP_CAL_VERIFY_SAMPLES fresh raw reads,
// averages their net, divides by candidate to predict grams, and only
// installs + saves NVS if |predicted - known| ≤ BSP_CAL_VERIFY_TOLERANCE_G.
// Catches "load shifted off the platform during collection" and
// "candidate factor produces nonsense numbers" failure modes.
#ifndef BSP_CAL_VERIFY_SAMPLES
#define BSP_CAL_VERIFY_SAMPLES                4
#endif
#ifndef BSP_CAL_VERIFY_TOLERANCE_G
#define BSP_CAL_VERIFY_TOLERANCE_G          5.0f
#endif

// H9.5: display-only zero-lock with hysteresis.  When calibrated and
// the H8 STABLE flag is true and |filtered grams| <= BSP_DISPLAY_ZERO_LOCK_G,
// the UI displays exactly "0 g" until |grams| exceeds
// BSP_DISPLAY_ZERO_RELEASE_G.  No internal state is modified — tare
// offset, cal factor, snapshot grams, NVS all untouched.  Pure UI
// smoothing to eliminate the visible ±2-5 g flicker near zero on
// mechanically-noisy assemblies.
#ifndef BSP_DISPLAY_ZERO_LOCK_ENABLE
#define BSP_DISPLAY_ZERO_LOCK_ENABLE      1
#endif
#ifndef BSP_DISPLAY_ZERO_LOCK_G
#define BSP_DISPLAY_ZERO_LOCK_G           5.0f
#endif
#ifndef BSP_DISPLAY_ZERO_RELEASE_G
#define BSP_DISPLAY_ZERO_RELEASE_G        8.0f
#endif
/* Reserved for future "near-stable hold" display hint; not used by H9.5. */
#ifndef BSP_DISPLAY_STABLE_HOLD_G
#define BSP_DISPLAY_STABLE_HOLD_G         3.0f
#endif

// H9.5: CAL factor sanity warnings (informational only — DO NOT REJECT).
// When the candidate factor differs from the previously-saved factor
// by more than these fractions, log a warning so the user can see the
// jump in the serial output.  The verification pass remains the
// only hard gate for accepting/rejecting candidate factors.
#ifndef BSP_CAL_FACTOR_DRIFT_WARN_FRACTION
#define BSP_CAL_FACTOR_DRIFT_WARN_FRACTION    0.30f
#endif
#ifndef BSP_CAL_FACTOR_DRIFT_STRONG_FRACTION
#define BSP_CAL_FACTOR_DRIFT_STRONG_FRACTION  0.80f
#endif

// H9.6 — power-mgr weight-activity tracking.  Each idle-check tick,
// power_mgr compares the current filtered grams to the value it saw on
// the previous tick.  A delta > BSP_POWER_WEIGHT_ACTIVITY_DELTA_G is
// treated as user activity and arms a hold timer; standby is blocked
// for BSP_POWER_ACTIVITY_HOLD_MS after the most recent weight change.
// Catches "user just put / removed a 500 g item, walked away briefly".
#ifndef BSP_POWER_WEIGHT_ACTIVITY_DELTA_G
#define BSP_POWER_WEIGHT_ACTIVITY_DELTA_G   10.0f
#endif
#ifndef BSP_POWER_ACTIVITY_HOLD_MS
#define BSP_POWER_ACTIVITY_HOLD_MS          30000
#endif

// H9.6 — wake-touch guard window.  After SOFT_STANDBY → ACTIVE,
// power_mgr_ui_actions_blocked() returns TRUE for this many ms.  UI
// button CLICKED handlers check this FIRST and early-return when
// true.  Unlike consume_wake_tap (single-shot), this is a
// non-consuming gate so multiple click events during the window all
// block — covers the case where LVGL dispatches the CLICKED event a
// few hundred ms after the touch INT fires.
#ifndef BSP_WAKE_TOUCH_GUARD_MS
#define BSP_WAKE_TOUCH_GUARD_MS             1000
#endif

// H9.6 — TARE empty-safe gate (after settle, before sample collection).
// When calibrated, refuse to TARE if |filtered grams| exceeds
// BSP_TARE_EMPTY_LIMIT_G.  Prevents the failure mode where a stray
// touch on the TARE button (including a wake-touch leak that escapes
// the wake-touch guard) accidentally zeroes a 500 g reading.  A
// future container-tare mode would be a separate explicit feature.
#ifndef BSP_TARE_REQUIRE_EMPTY
#define BSP_TARE_REQUIRE_EMPTY              1
#endif
#ifndef BSP_TARE_EMPTY_LIMIT_G
#define BSP_TARE_EMPTY_LIMIT_G              10.0f
#endif

// H9.7 — scale display range.
//   * BSP_SCALE_MAX_DISPLAY_G  — informational soft cap (3 kg today —
//     practical max for typical bottle weighing).  No behavior change
//     at this threshold; here for future warning UI.
//   * BSP_SCALE_OVERLOAD_G    — hard cap.  |display_grams| above this
//     shows "OVER" instead of a number — prevents nonsense readings
//     from cell saturation.
#ifndef BSP_SCALE_MAX_DISPLAY_G
#define BSP_SCALE_MAX_DISPLAY_G            3000.0f
#endif
#ifndef BSP_SCALE_OVERLOAD_G
#define BSP_SCALE_OVERLOAD_G               3500.0f
#endif

// H9.0: soft-standby (display-off) idle timing.
// After this many milliseconds with no touch activity, the backlight
// snaps off (BSP_STANDBY_BACKLIGHT_DUTY).  Any touch wakes the screen
// back to BSP_ACTIVE_BACKLIGHT_DUTY and resets the idle timer.
// HX711 keeps running at full 10 Hz throughout — deep sleep is NOT
// entered in H9.0 (that lives in H9.1).
// The first wake-tap is consumed by the UI so it does not also trigger
// a TARE or CAL action under the finger.
#ifndef BSP_IDLE_TO_STANDBY_MS
#define BSP_IDLE_TO_STANDBY_MS        180000   /* H9.2: 3 min (was 30 s) */
#endif
#ifndef BSP_STANDBY_BACKLIGHT_DUTY
#define BSP_STANDBY_BACKLIGHT_DUTY         0
#endif
#ifndef BSP_ACTIVE_BACKLIGHT_DUTY
#define BSP_ACTIVE_BACKLIGHT_DUTY      BSP_LCD_BACKLIGHT_DEFAULT
#endif
// Reserved for hardware-LEDC fade implementation; H9.0/H9.0.1 use a
// snap transition (no fade) to keep the change surface minimal.
#ifndef BSP_STANDBY_BACKLIGHT_FADE_MS
#define BSP_STANDBY_BACKLIGHT_FADE_MS    200
#endif

// H9.0.1: GPIO16 (touch INT) poll cadence while in SOFT_STANDBY.
// Bypasses LVGL/CST820 first-tap wake jitter — the CST820 sometimes
// burns the first tap waking the chip without delivering an event to
// LVGL, which led to the user-visible "second tap needed" bug in H9.0.
// We poll GPIO16 directly at this rate ONLY while in standby; the
// timer is stopped on wake so it costs nothing in ACTIVE.
#ifndef BSP_STANDBY_INT_POLL_MS
#define BSP_STANDBY_INT_POLL_MS            30
#endif

// H9.0.1: time window after a wake transition during which a button
// CLICKED is treated as the "wake tap" and consumed (does not execute
// the underlying TARE/CAL action).  After this window expires the
// next click is a normal click.  Prevents stale consume-flag from a
// non-button standby tap (which would otherwise eat the next real
// click much later).
#ifndef BSP_WAKE_TAP_WINDOW_MS
#define BSP_WAKE_TAP_WINDOW_MS            500
#endif

// H9.1 (safe rev): HX711 inter-sample idle period.  power_mgr swaps
// between these on SOFT_STANDBY transitions.
//
// SAFETY NOTE: the FIRST H9.1 attempt also tried to wake the HX711
// owner task out of its sleep via the same xTaskNotify slot used for
// TARE/CAL — that interfered with TARE/CAL reliability and was
// rolled back.  This safe revision uses only a volatile period
// variable read into a plain vTaskDelay; the HX711 owner task's
// notification slot is left identical to stable-h9.0.1.  Trade-off:
// a rate change has up to `current_period_ms` of latency to take
// effect (≤ 1 s coming out of standby).  Worth the simpler safety.
#ifndef BSP_HX711_ACTIVE_PERIOD_MS
#define BSP_HX711_ACTIVE_PERIOD_MS        100
#endif
#ifndef BSP_HX711_STANDBY_PERIOD_MS
#define BSP_HX711_STANDBY_PERIOD_MS      1000
#endif

// H9.2: weight-aware standby.  power_mgr's idle check now reads the
// HX711 filtered grams + stable flag from the public snapshot API
// (it never touches HX711 hardware).  Default-deny: only enters
// SOFT_STANDBY when ALL of these are true:
//   * idle for at least BSP_IDLE_TO_STANDBY_MS
//   * calibration is valid                       (else can't trust grams)
//   * |filtered grams| <= BSP_STANDBY_EMPTY_THRESHOLD_G   (gated by
//     BSP_STANDBY_REQUIRE_EMPTY — set 0 to disable the empty check)
//   * stable flag is true                        (gated by
//     BSP_STANDBY_BLOCK_IF_UNSTABLE — set 0 to disable)
// When any condition fails, standby is blocked and a (30-s-throttled)
// log line names the reason.  Re-checked at the existing 1 Hz idle
// poll, so as soon as conditions clear (e.g. user removes weight),
// standby fires within ~1 s.
#ifndef BSP_STANDBY_REQUIRE_EMPTY
#define BSP_STANDBY_REQUIRE_EMPTY            1
#endif
#ifndef BSP_STANDBY_EMPTY_THRESHOLD_G
#define BSP_STANDBY_EMPTY_THRESHOLD_G     10.0f
#endif
#ifndef BSP_STANDBY_BLOCK_IF_UNSTABLE
#define BSP_STANDBY_BLOCK_IF_UNSTABLE        1
#endif
#ifndef BSP_STANDBY_BLOCK_LOG_THROTTLE_MS
#define BSP_STANDBY_BLOCK_LOG_THROTTLE_MS  30000
#endif
