// hx711.h — bit-bang driver for the HX711 24-bit load-cell ADC.
//
// Single-owner contract: only the scale owner task (added in a later H step)
// may call these functions. The UI never touches the HX711 directly.
//
// Phase 2 step ladder:
//   H1 (this commit) — declarations only; bodies are skeletons that return
//                      ESP_ERR_NOT_SUPPORTED. Not called from app_main.
//   H2 — implement hx711_init / data_ready / read_raw_blocking;
//        wire from a low-priority hx711_task at 10 Hz, raw-only serial log.
//   H3 — auto-swap-pin probe if H2 reads stuck 0xFFFFFF or no movement.
//   H4 — feed raw + avg into ui_set_raw/ui_set_weight_kg.
//   H5 — TARE request via queue + NVS persistence.
//   H6 — CAL request via queue + NVS persistence.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    HX711_GAIN_128_A = 1,   // channel A, gain 128 — 25 SCK pulses per sample
    HX711_GAIN_32_B  = 2,   // channel B, gain 32  — 26 SCK pulses per sample
    HX711_GAIN_64_A  = 3,   // channel A, gain 64  — 27 SCK pulses per sample
} hx711_gain_t;

// Configure SCK as output (low) and DOUT as input. Idempotent.
// Phase 2 H2 will implement the body.
esp_err_t hx711_init(int sck_gpio, int dout_gpio);

// True iff DOUT is asserted low (a sample is ready to be clocked out).
// Phase 2 H2 will implement the body.
bool hx711_data_ready(void);

// Block (with interrupts disabled around the 25–27 clock burst) until a
// sample is available or `timeout_ms` elapses. On success, *out receives the
// signed 24-bit value (sign-extended to int32_t). The N-th extra clock pulse
// after the 24 data bits selects the gain/channel for the NEXT read.
// Phase 2 H2 will implement the body.
esp_err_t hx711_read_raw_blocking(int32_t *out, uint32_t timeout_ms);

// Set the gain/channel applied on the NEXT successful read (the chip latches
// the selection at the trailing edge of the (24+N)th SCK pulse).
// Phase 2 H2 will implement the body.
esp_err_t hx711_set_gain_next(hx711_gain_t g);

// Spawn the single HX711 owner task. After this returns OK, NO other task
// (including the UI) may invoke any other hx711_* function — only the owner
// task is allowed. Idempotent: a second call is a no-op.
// In H2 the task only ESP_LOGIs raw/dout/ready at ~10 Hz.
esp_err_t hx711_owner_start(void);

// Thread-safe latest-raw snapshot getter.  This is one of the cache-only
// hx711_* calls non-owner consumers are allowed to make.  Reads a single
// volatile int32_t — naturally atomic on ESP32-S3 — so no lock is needed.
//
//   *out_raw  := most recent successful raw reading (sign-extended int32_t)
//   return    := true if at least one sample has been captured since boot;
//                false if the owner task has never produced a valid read
//                yet (in which case *out_raw is left unchanged).
bool hx711_get_latest_raw(int32_t *out_raw);

// H5 — TARE
// Request the owner task to set a new tare zero.  Non-blocking — the UI
// just enqueues a notification and returns immediately.  The owner task
// processes the request at its next loop iteration: it collects 8 fresh
// raw samples (~800 ms with the 10 Hz HX711 RATE pin), averages them, and
// stores the result as tare_offset.  After that, the cached `net` reads
// near 0 at rest and changes with applied load.
//
// Returns ESP_OK when the request is enqueued.
//         ESP_ERR_INVALID_STATE if owner task not started OR a TARE/CAL is
//                               already in progress (H6 busy guard).
// H5 is RAM-only — no NVS.
esp_err_t hx711_request_tare(void);

// H6 — CAL
// Request the owner task to compute a calibration factor with a known
// reference weight on the cell.  Non-blocking — the UI enqueues and
// returns immediately.  The owner task collects 16 fresh samples
// (~1.6 s), averages their net values (after subtracting the current
// tare_offset), and stores cal_factor = avg_net / known_grams.
//
// Sign is preserved naturally — if the cell is wired with opposite
// polarity, cal_factor will be negative and runtime grams will still
// come out positive when the same load direction is applied.
//
// Rejected (logged, previous cal state preserved) if:
//   * known_grams <= 0
//   * |avg_net| < 1000 counts (no meaningful load on the cell)
//   * zero samples acquired
//
// Returns ESP_OK when the request is enqueued.
//         ESP_ERR_INVALID_STATE if owner task not started OR a TARE/CAL is
//                               already in progress (H6 busy guard).
//         ESP_ERR_INVALID_ARG    if known_grams <= 0.
// H6 is RAM-only — no NVS.
esp_err_t hx711_request_calibrate(float known_grams);

// Atomic snapshot of (raw, net).  Either out pointer may be NULL.
// Returns true if at least one sample has been captured since boot, in
// which case both fields are written under a per-core portMUX so the
// (raw, net) pair is always self-consistent.
//   raw  := last successful raw reading
//   net  := raw - tare_offset  (== raw before the first TARE)
bool hx711_get_snapshot(int32_t *out_raw, int32_t *out_net);

// H6 — full snapshot including calibrated grams.
//   *out_raw         := last successful raw reading
//   *out_net         := raw - tare_offset
//   *out_grams       := net / cal_factor   (valid only if *out_cal_valid)
//   *out_cal_valid   := true iff calibration has been completed at least
//                       once since boot
// Any out pointer may be NULL.  Returns true if at least one sample has
// been captured since boot.  All fields are read under the per-core
// portMUX so the (raw, net, grams, cal_valid) tuple is self-consistent.
bool hx711_get_snapshot_full(int32_t *out_raw, int32_t *out_net,
                              float *out_grams, bool *out_cal_valid);
