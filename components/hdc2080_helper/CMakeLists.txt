idf_component_register(
    SRCS "hdc2080_helper.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_gpio esp_driver_i2c hdc2080
)

