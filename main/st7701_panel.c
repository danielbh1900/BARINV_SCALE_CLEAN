// ============================================================================
// st7701_panel.c — ST7701S init via 3-wire SPI (CS on TCA9554 EXIO3) +
// 480x480 RGB565 streaming.
//
// Vendor init command sequence below is from the Espressif Waveshare-authored
// board header (`BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_2_1.h`) which in turn
// matches the factory ESP-IDF demo source.
// ============================================================================

#include "st7701_panel.h"
#include "board_config.h"
#include "tca9554.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"  // esp_lcd_new_panel_io_3wire_spi
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7701.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "st7701";

// ---- Vendor init sequence (verbatim from the locked board contract) -------
static const st7701_lcd_init_cmd_t s_vendor_init_cmds[] = {
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10},                       5,  0},
    {0xC0, (uint8_t[]){0x3B, 0x00},                                         2,  0},
    {0xC1, (uint8_t[]){0x0B, 0x02},                                         2,  0},
    {0xC2, (uint8_t[]){0x07, 0x02},                                         2,  0},
    {0xCC, (uint8_t[]){0x10},                                               1,  0},
    {0xCD, (uint8_t[]){0x08},                                               1,  0},
    {0xB0, (uint8_t[]){0x00,0x11,0x16,0x0E,0x11,0x06,0x05,0x09,
                       0x08,0x21,0x06,0x13,0x10,0x29,0x31,0x18},           16,  0},
    {0xB1, (uint8_t[]){0x00,0x11,0x16,0x0E,0x11,0x07,0x05,0x09,
                       0x09,0x21,0x05,0x13,0x11,0x2A,0x31,0x18},           16,  0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11},                       5,  0},
    {0xB0, (uint8_t[]){0x6D},                                               1,  0},
    {0xB1, (uint8_t[]){0x37},                                               1,  0},
    {0xB2, (uint8_t[]){0x81},                                               1,  0},
    {0xB3, (uint8_t[]){0x80},                                               1,  0},
    {0xB5, (uint8_t[]){0x43},                                               1,  0},
    {0xB7, (uint8_t[]){0x85},                                               1,  0},
    {0xB8, (uint8_t[]){0x20},                                               1,  0},
    {0xC1, (uint8_t[]){0x78},                                               1,  0},
    {0xC2, (uint8_t[]){0x78},                                               1,  0},
    {0xD0, (uint8_t[]){0x88},                                               1,  0},
    {0xE0, (uint8_t[]){0x00, 0x00, 0x02},                                   3,  0},
    {0xE1, (uint8_t[]){0x03,0xA0,0x00,0x00,0x04,0xA0,0x00,0x00,
                       0x00,0x20,0x20},                                    11,  0},
    {0xE2, (uint8_t[]){0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00},                     13,  0},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x11, 0x00},                             4,  0},
    {0xE4, (uint8_t[]){0x22, 0x00},                                         2,  0},
    {0xE5, (uint8_t[]){0x05,0xEC,0xA0,0xA0,0x07,0xEE,0xA0,0xA0,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},           16,  0},
    {0xE6, (uint8_t[]){0x00, 0x00, 0x11, 0x00},                             4,  0},
    {0xE7, (uint8_t[]){0x22, 0x00},                                         2,  0},
    {0xE8, (uint8_t[]){0x06,0xED,0xA0,0xA0,0x08,0xEF,0xA0,0xA0,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},           16,  0},
    {0xEB, (uint8_t[]){0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00},           7,  0},
    {0xED, (uint8_t[]){0xFF,0xFF,0xFF,0xBA,0x0A,0xBF,0x45,0xFF,
                       0xFF,0x54,0xFB,0xA0,0xAB,0xFF,0xFF,0xFF},           16,  0},
    {0xEF, (uint8_t[]){0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F},                 6,  0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13},                       5,  0},
    {0xEF, (uint8_t[]){0x08},                                               1,  0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00},                       5,  0},
    {0x36, (uint8_t[]){0x00},                                               1,  0},
    {0x3A, (uint8_t[]){0x66},                                               1,  0},
    {0x11, (uint8_t[]){0x00},                                               0, 480},
    {0x20, (uint8_t[]){0x00},                                               0, 120},
    {0x29, (uint8_t[]){0x00},                                               0,   0},
};

// ---- Reset the panel via the TCA9554 (EXIO1 = LCD_RST) --------------------
static void lcd_reset_via_expander(void) {
    bsp_tca9554_set(BSP_EXIO_LCD_RST_BIT, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    bsp_tca9554_set(BSP_EXIO_LCD_RST_BIT, true);
    vTaskDelay(pdMS_TO_TICKS(120));
}

esp_err_t bsp_st7701_init(esp_lcd_panel_handle_t *out_panel,
                          esp_lcd_panel_io_handle_t *out_io) {
    if (out_panel == NULL || out_io == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "ST7701 init starting");

    // ---- 1. Reset the panel via the expander before init -----------------
    lcd_reset_via_expander();

    // ---- 2. Build the 3-wire SPI panel-IO with CS on the expander --------
    spi_line_config_t spi_line = {
        .cs_io_type        = IO_TYPE_EXPANDER,
        .cs_expander_pin   = BSP_EXIO_MASK(BSP_EXIO_LCD_CS_BIT),
        .scl_io_type       = IO_TYPE_GPIO,
        .scl_gpio_num      = BSP_LCD_SPI_PIN_SCK,
        .sda_io_type       = IO_TYPE_GPIO,
        .sda_gpio_num      = BSP_LCD_SPI_PIN_MOSI,
        .io_expander       = bsp_tca9554_handle(),
    };

    esp_lcd_panel_io_3wire_spi_config_t io_cfg = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(spi_line, 0);

    esp_err_t ret = esp_lcd_new_panel_io_3wire_spi(&io_cfg, out_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_io_3wire_spi: %s", esp_err_to_name(ret));
        return ret;
    }

    // ---- 3. Build the RGB panel config (timing + data pins) --------------
    esp_lcd_rgb_panel_config_t rgb_cfg = {
        .clk_src           = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align = 64,
        .data_width        = 16,
        .bits_per_pixel    = 16,
        .num_fbs           = BSP_LCD_RGB_FRAME_BUF_NUM,
        .bounce_buffer_size_px = BSP_LCD_RGB_BOUNCE_BUF_PX,
        .de_gpio_num       = BSP_LCD_PIN_DE,
        .pclk_gpio_num     = BSP_LCD_PIN_PCLK,
        .vsync_gpio_num    = BSP_LCD_PIN_VSYNC,
        .hsync_gpio_num    = BSP_LCD_PIN_HSYNC,
        .disp_gpio_num     = BSP_LCD_PIN_DISP,
        .data_gpio_nums = {
            BSP_LCD_PIN_DATA0,  BSP_LCD_PIN_DATA1,  BSP_LCD_PIN_DATA2,
            BSP_LCD_PIN_DATA3,  BSP_LCD_PIN_DATA4,  BSP_LCD_PIN_DATA5,
            BSP_LCD_PIN_DATA6,  BSP_LCD_PIN_DATA7,  BSP_LCD_PIN_DATA8,
            BSP_LCD_PIN_DATA9,  BSP_LCD_PIN_DATA10, BSP_LCD_PIN_DATA11,
            BSP_LCD_PIN_DATA12, BSP_LCD_PIN_DATA13, BSP_LCD_PIN_DATA14,
            BSP_LCD_PIN_DATA15,
        },
        .timings = {
            .pclk_hz           = BSP_LCD_RGB_PCLK_HZ,
            .h_res             = BSP_LCD_H_RES,
            .v_res             = BSP_LCD_V_RES,
            .hsync_pulse_width = BSP_LCD_RGB_HSYNC_PULSE,
            .hsync_back_porch  = BSP_LCD_RGB_HSYNC_BACK,
            .hsync_front_porch = BSP_LCD_RGB_HSYNC_FRONT,
            .vsync_pulse_width = BSP_LCD_RGB_VSYNC_PULSE,
            .vsync_back_porch  = BSP_LCD_RGB_VSYNC_BACK,
            .vsync_front_porch = BSP_LCD_RGB_VSYNC_FRONT,
            .flags.pclk_active_neg = false,
        },
        .flags.fb_in_psram = true,
    };

    // ---- 4. Build the ST7701 vendor config + panel ------------------------
    st7701_vendor_config_t vendor_cfg = {
        .rgb_config = &rgb_cfg,
        .init_cmds       = s_vendor_init_cmds,
        .init_cmds_size  = sizeof(s_vendor_init_cmds) / sizeof(s_vendor_init_cmds[0]),
        .flags = {
            .auto_del_panel_io = 0,
            .mirror_by_cmd     = 1,
        },
    };

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,                     // reset handled via EXIO
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config  = (void *)&vendor_cfg,
    };

    ret = esp_lcd_new_panel_st7701(*out_io, &panel_cfg, out_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_st7701: %s", esp_err_to_name(ret));
        return ret;
    }

    // ---- 5. Reset + init + display on -------------------------------------
    ret = esp_lcd_panel_reset(*out_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_reset: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_panel_init(*out_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_init: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_panel_disp_on_off(*out_panel, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "disp_on_off: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ST7701 ready (%dx%d, RGB565, PCLK %d Hz)",
             BSP_LCD_H_RES, BSP_LCD_V_RES, BSP_LCD_RGB_PCLK_HZ);
    return ESP_OK;
}
