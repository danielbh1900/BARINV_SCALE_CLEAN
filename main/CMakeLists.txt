idf_component_register(
    SRCS
        "main.c"
        "ui.c"
        "i2c_bus.c"
        "tca9554.c"
        "backlight.c"
        "st7701_panel.c"
        "cst820_touch.c"
        "lvgl_glue.c"
        "hx711.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_lcd
        esp_timer
        nvs_flash
)

# LVGL tick fix — same one that was preserved in the proven BARINV build.
idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_compile_definitions(${lvgl_lib} PUBLIC
    "CONFIG_LV_TICK_CUSTOM_SYS_TIME_EXPR=(esp_timer_get_time()/1000ULL)"
)
