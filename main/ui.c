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
#include "hx711.h"     /* H4: pull latest cached raw — no direct hardware access */

static const char *TAG = "ui";

static lv_obj_t *s_weight_lbl = NULL;
static lv_obj_t *s_stable_lbl = NULL;
static lv_obj_t *s_raw_lbl    = NULL;

static void on_tare_clicked(lv_event_t *e) {
    (void)e;
    ESP_LOGI(TAG, "TARE clicked");
}

static void on_cal_clicked(lv_event_t *e) {
    (void)e;
    ESP_LOGI(TAG, "CAL clicked");
}

/* H4: LVGL timer callback — pulls the latest cached HX711 raw (no I/O,
 * no protocol, no GPIO) and updates the raw label.  Runs from inside
 * lv_timer_handler so the LVGL port lock is already held. */
static void ui_poll_raw_cb(lv_timer_t *t) {
    (void)t;
    if (s_raw_lbl == NULL) return;
    int32_t raw;
    if (hx711_get_latest_raw(&raw)) {
        lv_label_set_text_fmt(s_raw_lbl, "raw: %ld", (long)raw);
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

    // Weight value (large)
    s_weight_lbl = lv_label_create(scr);
    lv_label_set_text(s_weight_lbl, "0.000");
    lv_obj_set_style_text_color(s_weight_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_weight_lbl, &lv_font_montserrat_48, 0);
    lv_obj_align(s_weight_lbl, LV_ALIGN_CENTER, 0, -30);

    // Unit
    lv_obj_t *unit = lv_label_create(scr);
    lv_label_set_text(unit, "kg");
    lv_obj_set_style_text_color(unit, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(unit, &lv_font_montserrat_22, 0);
    lv_obj_align(unit, LV_ALIGN_CENTER, 0, 28);

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

    lvgl_port_unlock();
    ESP_LOGI(TAG, "BARINV Scale UI built (H4: raw label polls hx711 cache @ 10 Hz)");
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
