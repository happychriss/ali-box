#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "qmi8658.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    gpio_num_t gpio_int1;
    uint8_t wom_threshold_mg;
    uint8_t wom_blanking_samples;
} imu_helper_config_t;

typedef struct {
    qmi8658_dev_t dev;
    gpio_num_t gpio_int1;
    uint8_t wom_threshold_mg;
    uint8_t wom_blanking_samples;
    bool initialized;
} imu_helper_handle_t;

/**
 * Initialize IMU (QMI8658).
 * Tries both I2C addresses (0x6B then 0x6A).
 */
esp_err_t imu_helper_init(const imu_helper_config_t *cfg, imu_helper_handle_t *out);

/**
 * Arm Wake-on-Motion interrupt for deep sleep wakeup.
 * Configures INT1 to toggle LOW on motion (for EXT1 ANY_LOW).
 */
esp_err_t imu_helper_arm_wom(imu_helper_handle_t *h);

/**
 * Clear WoM status (read STATUS1 to reset INT line).
 */
esp_err_t imu_helper_clear_wom_status(imu_helper_handle_t *h);

/**
 * Get INT1 GPIO level.
 */
int imu_helper_get_int1_level(imu_helper_handle_t *h);

#ifdef __cplusplus
}
#endif

