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

// ---- Buzzer (LATER; on/off only via TCA9554 EXIO8) ------------------------
#define BSP_BUZZER_VIA_EXIO_BIT      8

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
