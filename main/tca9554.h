// tca9554.h — TCA9554PWR I/O expander on the locked I²C bus.
//
// Owns: LCD_RST (EXIO1) · TP_RST (EXIO2) · LCD_CS (EXIO3) ·
//       SD_CS (EXIO4) · BUZZER (EXIO8).
//
// All other modules pull control of their EXIO lines through this driver.
#pragma once

#include "esp_err.h"
#include "esp_io_expander.h"

// Initialises the expander, sets all 8 pins to OUTPUT-LOW, and exposes
// the handle for downstream modules. Idempotent.
esp_err_t bsp_tca9554_init(void);

// Returns the shared expander handle (NULL until bsp_tca9554_init succeeds).
esp_io_expander_handle_t bsp_tca9554_handle(void);

// Convenience wrapper around esp_io_expander_set_level for one 1-indexed pin.
esp_err_t bsp_tca9554_set(uint8_t exio_bit_1indexed, bool level);
