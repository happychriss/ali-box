#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "hdc2080.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint8_t i2c_addr;
    gpio_num_t gpio_irq;
} hdc2080_helper_config_t;

typedef struct {
    hdc2080_handle_t handle;
    gpio_num_t gpio_irq;
    bool initialized;
} hdc2080_helper_handle_t;

/**
 * Initialize HDC2080 temperature/humidity sensor.
 */
esp_err_t hdc2080_helper_init(const hdc2080_helper_config_t *cfg, hdc2080_helper_handle_t *out);

/**
 * Read temperature (and optionally humidity).
 */
esp_err_t hdc2080_helper_read_temp(hdc2080_helper_handle_t *h, float *temp_c, float *humidity_rh);

/**
 * Arm temperature threshold interrupt for deep sleep wakeup.
 * Configures for rising temperature threshold (baseline + delta).
 */
esp_err_t hdc2080_helper_arm_temp_irq(hdc2080_helper_handle_t *h, float baseline_c, float delta_c);

/**
 * Clear any latched interrupt status.
 */
esp_err_t hdc2080_helper_clear_status(hdc2080_helper_handle_t *h);

/**
 * Check if IRQ pin is currently asserted.
 */
bool hdc2080_helper_irq_asserted(hdc2080_helper_handle_t *h);

/**
 * Get IRQ GPIO level.
 */
int hdc2080_helper_get_irq_level(hdc2080_helper_handle_t *h);

#ifdef __cplusplus
}
#endif

