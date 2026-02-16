idf_component_register(
    SRCS "touch_helper.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_gpio esp_driver_i2c esp_lcd_touch_cst816s
)

