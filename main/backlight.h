// backlight.h — LCD backlight on direct GPIO 6 via LEDC PWM.
#pragma once

#include "esp_err.h"
#include <stdint.h>

// Configures LEDC timer + channel, leaves the backlight OFF.
esp_err_t bsp_backlight_init(void);

// 0..1023 duty value (10-bit resolution).
esp_err_t bsp_backlight_set(uint16_t duty_0_to_1023);

// Convenience: drive to default startup brightness.
esp_err_t bsp_backlight_on(void);
esp_err_t bsp_backlight_off(void);
