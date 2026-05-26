// BARINV Scale Clean — UI
//
// Single screen, dark theme with gold accents (no separate theme module).
// Layout fits inside the 480-px round window — all content within a ~340
// px inscribed square.
//
//   ┌─────────────────────────┐
//   │       BARINV Scale      │   gold, Montserrat 22
//   │                         │
//   │         0.000           │   white, Montserrat 48
//   │           kg            │   gray, Montserrat 22
//   │       STABLE/...        │   gray/amber/green, Montserrat 14
//   │       raw: ----         │   dim, Montserrat 14
//   │                         │
//   │     [TARE]   [CAL]      │   default LVGL buttons (gold primary)
//   └─────────────────────────┘
//
// Phase 2 will hook the buttons up to the HX711 owner task. For Phase 1
// they only log to serial.

#include "ui.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "hx711.h"          /* H4: pull cached raw — no direct hardware access */
#include "board_config.h"   /* H6: BSP_CAL_WEIGHT_GRAMS */
#include "power_mgr.h"      /* H9.0: register activity, wake on tap */
#include "buzzer.h"         /* H9.4: short chirp on accepted tap */
#include "esp_timer.h"      /* H9.7: throttle diagnostic logs */
#include <math.h>           /* H6: fabsf, lroundf, isnan, isinf */

static const char *TAG = "ui";

static lv_obj_t *s_weight_lbl = NULL;
static lv_obj_t *s_unit_lbl   = NULL;   /* H6: needed so we can toggle g/kg */
static lv_obj_t *s_stable_lbl = NULL;
/* H9.0.1: consume-flag moved into power_mgr; UI just queries
 * power_mgr_consume_wake_tap() in each button CLICKED handler. */

/* H9.5: display-only zero-lock state.  Hysteretic — entering at
 * BSP_DISPLAY_ZERO_LOCK_G, leaving at BSP_DISPLAY_ZERO_RELEASE_G.
 * Pure UI; never touches HX711 cache or NVS. */
static bool s_zero_locked = false;
static lv_obj_t *s_raw_lbl    = NULL;

static void on_tare_clicked(lv_event_t *e) {
    (void)e;
    /* H9.6: blanket wake-touch guard — non-consuming, covers multiple
     * click events during BSP_WAKE_TOUCH_GUARD_MS.  Replaces the
     * H9.0.1 consume_wake_tap that could leak when LVGL fired two
     * CLICKEDs from one sustained touch.  Checked FIRST so a wake-tap
     * landing on TARE never zeroes the scale. */
    if (power_mgr_ui_actions_blocked()) {
        ESP_LOGI(TAG, "TARE: UI action blocked — wake touch consumed");
        return;
    }
    /* H9.4: short chirp acknowledges the button press (does NOT
     * mean the action has completed — that's the TARE-success beep
     * from the HX711 owner task several seconds later). */
    buzzer_tap();
    /* H5: UI only enqueues the request.  Owner task does the work
     * (8-sample average) on its next loop iteration. */
    esp_err_t r = hx711_request_tare();
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "TARE clicked — request enqueued");
    } else {
        ESP_LOGW(TAG, "TARE clicked — hx711_request_tare: %s",
                 esp_err_to_name(r));
    }
}

static void on_cal_clicked(lv_event_t *e) {
    (void)e;
    /* H9.6: blanket wake-touch guard (see TARE handler comment). */
    if (power_mgr_ui_actions_blocked()) {
        ESP_LOGI(TAG, "CAL: UI action blocked — wake touch consumed");
        return;
    }
    /* H9.4: tap acknowledgement chirp (see TARE handler comment). */
    buzzer_tap();
    /* H6: UI enqueues a CAL request with the build-time known weight.
     * Owner task does the 16-sample average + factor math; UI never
     * touches HX711 hardware. */
    esp_err_t r = hx711_request_calibrate(BSP_CAL_WEIGHT_GRAMS);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "CAL clicked — calibration request enqueued "
                      "(known=%.1fg)", (double)BSP_CAL_WEIGHT_GRAMS);
    } else {
        ESP_LOGW(TAG, "CAL clicked — hx711_request_calibrate: %s",
                 esp_err_to_name(r));
    }
}

/* H9.0 + H9.0.1: screen-level PRESSED handler.  Fires for every touch
 * press (bubbles up from whichever object was hit).  Just registers
 * activity — power_mgr handles the wake transition AND arms the
 * wake-tap consume window itself.  In H9.0.1 the GPIO16 poll usually
 * fires first (faster than the LVGL+CST820 pipeline), and this handler
 * becomes a no-op for the wake itself; it still resets the idle timer
 * for subsequent activity. */
static void scr_pressed_cb(lv_event_t *e) {
    (void)e;
    power_mgr_register_activity();
}

/* H4 + H5 + H6 + H8: LVGL timer callback — pulls the full HX711 snapshot
 * (raw, net, filtered grams, cal_valid, stable) atomically and refreshes
 * the labels.  No I/O, no protocol, no GPIO — pure cache reads.  Runs
 * from inside lv_timer_handler so the LVGL port lock is already held. */
static void ui_poll_raw_cb(lv_timer_t *t) {
    (void)t;
    if (s_raw_lbl == NULL || s_weight_lbl == NULL || s_unit_lbl == NULL ||
        s_stable_lbl == NULL) return;

    int32_t raw  = 0;
    int32_t net  = 0;
    float   grams = 0.0f;
    bool    cal_valid = false;
    bool    stable    = false;
    if (!hx711_get_snapshot_full(&raw, &net, &grams, &cal_valid, &stable)) return;

    /* Bottom debug line — always shows the underlying counts. */
    lv_label_set_text_fmt(s_raw_lbl, "raw: %ld / zero: %ld",
                          (long)raw, (long)net);

    /* H9.5: display-only zero-lock with hysteresis.  Pure UI smoothing
     * — internal grams/raw/net/cache/NVS are NOT modified, only the
     * value we hand to lv_label_set_text. */
#if BSP_DISPLAY_ZERO_LOCK_ENABLE
    float display_grams = grams;
    if (cal_valid) {
        if (s_zero_locked) {
            /* Stay locked at 0 until grams crosses the wider release
             * threshold — prevents flicker between -2 and +5 g. */
            if (fabsf(grams) > BSP_DISPLAY_ZERO_RELEASE_G) {
                s_zero_locked = false;
            } else {
                display_grams = 0.0f;
            }
        } else {
            /* Enter lock only when the H8 stability flag confirms
             * settled motion AND grams is inside the tighter lock band. */
            if (stable && fabsf(grams) <= BSP_DISPLAY_ZERO_LOCK_G) {
                s_zero_locked = true;
                display_grams = 0.0f;
            }
        }
    } else {
        s_zero_locked = false;   /* uncal → no lock possible */
    }
#else
    float display_grams = grams;
    (void)s_zero_locked;
#endif

    /* H9.7: large weight display.
     *
     * INTEGER-ONLY formatting throughout — our sdkconfig has
     * CONFIG_LV_SPRINTF_USE_FLOAT=n, so LVGL's mini-printf cannot
     * format "%f" / "%.3f".  Previously a kg display silently emitted
     * a single "f"/"F" character instead of "1.680".  Compute the
     * decimal split with integer math and use "%s%ld.%03ld" so the
     * output is identical regardless of LVGL's printf configuration. */
    if (!cal_valid) {
        lv_label_set_text(s_weight_lbl, "----");
        lv_label_set_text(s_unit_lbl,   "g");
    } else if (isnan(display_grams) || isinf(display_grams)) {
        /* Defensive — shouldn't happen since cal_factor is checked for
         * zero on NVS load.  Logged once per 5 s if it ever does. */
        static int64_t s_last_fault_log_us = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_fault_log_us > 5LL * 1000 * 1000) {
            ESP_LOGW(TAG, "display fault — NaN/Inf grams "
                          "(raw=%ld net=%ld cal_valid=%d stable=%d)",
                     (long)raw, (long)net, (int)cal_valid, (int)stable);
            s_last_fault_log_us = now_us;
        }
        lv_label_set_text(s_weight_lbl, "---");
        lv_label_set_text(s_unit_lbl,   "g");
    } else if (fabsf(display_grams) > BSP_SCALE_OVERLOAD_G) {
        /* Beyond safe display range — most likely cell saturation. */
        static int64_t s_last_ovl_log_us = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_ovl_log_us > 5LL * 1000 * 1000) {
            ESP_LOGW(TAG, "display fault — OVERLOAD grams=%ld > limit=%ld "
                          "(raw=%ld net=%ld)",
                     lroundf(display_grams),
                     lroundf(BSP_SCALE_OVERLOAD_G),
                     (long)raw, (long)net);
            s_last_ovl_log_us = now_us;
        }
        lv_label_set_text(s_weight_lbl, "OVER");
        lv_label_set_text(s_unit_lbl,   "kg");
    } else if (fabsf(display_grams) < 1000.0f) {
        lv_label_set_text_fmt(s_weight_lbl, "%ld", lroundf(display_grams));
        lv_label_set_text(s_unit_lbl, "g");
    } else {
        /* INTEGER-only kg format: %s%ld.%03ld.  Handles negatives,
         * works with or without LVGL float-printf support. */
        long total_g = (long)lroundf(display_grams);
        bool neg     = (total_g < 0);
        long ag      = neg ? -total_g : total_g;
        long kg      = ag / 1000;
        long fr      = ag % 1000;
        lv_label_set_text_fmt(s_weight_lbl, "%s%ld.%03ld",
                              neg ? "-" : "", kg, fr);
        lv_label_set_text(s_unit_lbl, "kg");

        /* H9.7: high-weight diagnostic log — throttled to 5 s.  Lets
         * us correlate displayed kg against raw counts / cal_factor
         * for accuracy analysis on multi-kg loads. */
        static int64_t s_last_high_log_us = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_high_log_us > 5LL * 1000 * 1000) {
            ESP_LOGI(TAG, "display kg=%s%ld.%03ld  raw=%ld net=%ld grams=%ld",
                     neg ? "-" : "", kg, fr,
                     (long)raw, (long)net, total_g);
            s_last_high_log_us = now_us;
        }
    }

    /* H8 + H9.5: stability/zero indicator.  Hidden ("---" gray) until
     * calibration is loaded.  Green "ZERO" when display zero-lock is
     * active (calibrated, stable, near zero).  Green "STABLE" when
     * calibrated + stable but not near zero.  Otherwise "---". */
    if (!cal_valid) {
        lv_label_set_text(s_stable_lbl, "---");
        lv_obj_set_style_text_color(s_stable_lbl, lv_color_hex(0x808080), 0);
#if BSP_DISPLAY_ZERO_LOCK_ENABLE
    } else if (s_zero_locked) {
        lv_label_set_text(s_stable_lbl, "ZERO");
        lv_obj_set_style_text_color(s_stable_lbl, lv_color_hex(0x4CAF50), 0);
#endif
    } else if (stable) {
        lv_label_set_text(s_stable_lbl, "STABLE");
        lv_obj_set_style_text_color(s_stable_lbl, lv_color_hex(0x4CAF50), 0);
    } else {
        lv_label_set_text(s_stable_lbl, "---");
        lv_obj_set_style_text_color(s_stable_lbl, lv_color_hex(0x808080), 0);
    }
}

void ui_init(void) {
    if (!lvgl_port_lock(0)) {
        ESP_LOGE(TAG, "lvgl_port_lock failed at ui_init");
        return;
    }

    lv_disp_t *disp  = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(
        disp,
        lv_color_hex(0xD4A056),   // primary: BARINV gold
        lv_color_hex(0x808080),   // secondary: gray
        true,                      // dark mode
        &lv_font_montserrat_14);
    lv_disp_set_theme(disp, theme);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "BARINV Scale");
    lv_obj_set_style_text_color(title, lv_color_hex(0xD4A056), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 80);

    // Weight value (large) — "----" until first calibration, then g or kg
    s_weight_lbl = lv_label_create(scr);
    lv_label_set_text(s_weight_lbl, "----");
    lv_obj_set_style_text_color(s_weight_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_weight_lbl, &lv_font_montserrat_48, 0);
    lv_obj_align(s_weight_lbl, LV_ALIGN_CENTER, 0, -30);

    // Unit — flips between "g" and "kg" at runtime based on magnitude
    s_unit_lbl = lv_label_create(scr);
    lv_label_set_text(s_unit_lbl, "g");
    lv_obj_set_style_text_color(s_unit_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_unit_lbl, &lv_font_montserrat_22, 0);
    lv_obj_align(s_unit_lbl, LV_ALIGN_CENTER, 0, 28);

    // Stable / Unstable
    s_stable_lbl = lv_label_create(scr);
    lv_label_set_text(s_stable_lbl, "---");
    lv_obj_set_style_text_color(s_stable_lbl, lv_color_hex(0x808080), 0);
    lv_obj_set_style_text_font(s_stable_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(s_stable_lbl, LV_ALIGN_CENTER, 0, 70);

    // Raw debug
    s_raw_lbl = lv_label_create(scr);
    lv_label_set_text(s_raw_lbl, "raw: ----");
    lv_obj_set_style_text_color(s_raw_lbl, lv_color_hex(0x606060), 0);
    lv_obj_set_style_text_font(s_raw_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(s_raw_lbl, LV_ALIGN_CENTER, 0, 100);

    // TARE button
    lv_obj_t *tare = lv_btn_create(scr);
    lv_obj_set_size(tare, 110, 60);
    lv_obj_align(tare, LV_ALIGN_BOTTOM_MID, -70, -90);
    lv_obj_add_event_cb(tare, on_tare_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *tare_lbl = lv_label_create(tare);
    lv_label_set_text(tare_lbl, "TARE");
    lv_obj_set_style_text_font(tare_lbl, &lv_font_montserrat_18, 0);
    lv_obj_center(tare_lbl);

    // CAL button
    lv_obj_t *cal = lv_btn_create(scr);
    lv_obj_set_size(cal, 110, 60);
    lv_obj_align(cal, LV_ALIGN_BOTTOM_MID, 70, -90);
    lv_obj_add_event_cb(cal, on_cal_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cal_lbl = lv_label_create(cal);
    lv_label_set_text(cal_lbl, "CAL");
    lv_obj_set_style_text_font(cal_lbl, &lv_font_montserrat_18, 0);
    lv_obj_center(cal_lbl);

    /* H4: 10 Hz LVGL timer that pulls the latest cached HX711 raw from
     * the owner task and refreshes the on-screen "raw:" label.  Runs inside
     * lv_timer_handler → LVGL port lock is already held; safe to touch
     * widgets directly.  No HX711 hardware/protocol access — only the
     * thread-safe cache getter. */
    lv_timer_create(ui_poll_raw_cb, 100, NULL);

    /* H9.0: install screen-level PRESSED handler for activity + wake.
     * Every touch (button or empty area) bubbles a PRESSED event up to
     * the screen, so this single handler covers all taps. */
    lv_obj_add_event_cb(scr, scr_pressed_cb, LV_EVENT_PRESSED, NULL);

    lvgl_port_unlock();
    ESP_LOGI(TAG, "BARINV Scale UI built (H4: raw label polls hx711 cache @ 10 Hz; "
                  "H9.0: tap-to-wake handler installed)");
}

void ui_set_weight_kg(float kg, bool stable) {
    if (s_weight_lbl == NULL) return;
    if (!lvgl_port_lock(0)) return;
    lv_label_set_text_fmt(s_weight_lbl, "%.3f", (double)kg);
    if (s_stable_lbl) {
        lv_label_set_text(s_stable_lbl, stable ? "STABLE" : "...");
        lv_obj_set_style_text_color(
            s_stable_lbl,
            stable ? lv_color_hex(0x4CAF50) : lv_color_hex(0xFFA000),
            0);
    }
    lvgl_port_unlock();
}

void ui_set_raw(int32_t raw) {
    if (s_raw_lbl == NULL) return;
    if (!lvgl_port_lock(0)) return;
    lv_label_set_text_fmt(s_raw_lbl, "raw: %ld", (long)raw);
    lvgl_port_unlock();
}
