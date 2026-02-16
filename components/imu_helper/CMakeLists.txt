idf_component_register(
    SRCS "imu_helper.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_gpio esp_driver_i2c qmi8658_wom
)

