// BARINV Scale Clean — UI public API
#pragma once

#include <stdint.h>
#include <stdbool.h>

// Builds the scale screen. Must be called AFTER bsp_lvgl_init.
void ui_init(void);

// Setters used by Phase 2 (HX711 owner task). Safe to call from any task —
// they take the LVGL port lock internally. No-ops if ui_init wasn't called.
void ui_set_weight_kg(float kg, bool stable);
void ui_set_raw(int32_t raw);
