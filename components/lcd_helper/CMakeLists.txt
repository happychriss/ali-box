idf_component_register(
    SRCS "lcd_helper.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_lcd esp_driver_gpio esp_driver_spi
)

