#pragma once

#include "esp_err.h"
#include "esp_lcd_touch.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* I2C */
    i2c_master_bus_handle_t i2c_bus;
    int i2c_clk_hz;

    /* Pins */
    gpio_num_t gpio_int;
    gpio_num_t gpio_rst;
    int rst_active_level;

    /* Touch panel resolution (portrait native) */
    int x_max;
    int y_max;

    /* Set to true if hardware reset was already performed externally */
    bool skip_hw_reset;
} touch_helper_config_t;

typedef struct {
    esp_lcd_touch_handle_t handle;
    SemaphoreHandle_t irq_sem;
    gpio_num_t gpio_int;
    gpio_num_t gpio_rst;
} touch_helper_handle_t;

/**
 * Initialize touch controller (CST816S).
 * Creates IRQ semaphore and configures touch for interrupt-driven reads.
 */
esp_err_t touch_helper_init(const touch_helper_config_t *cfg, touch_helper_handle_t *out);

/**
 * Read touch data (call after IRQ semaphore is taken).
 * Returns true if touch is pressed, with coordinates in x/y.
 */
bool touch_helper_read(touch_helper_handle_t *h, int32_t *x, int32_t *y);

/**
 * Arm touch GPIO as deep sleep wake source (EXT1 any-low).
 * Call this before entering deep sleep.
 */
esp_err_t touch_helper_arm_wakeup(touch_helper_handle_t *h);

/**
 * Put touch controller into low-power sleep mode.
 * INT pin remains active for wake.
 */
esp_err_t touch_helper_enter_sleep(touch_helper_handle_t *h);

/**
 * Disable touch interrupt (call before deep sleep GPIO reconfiguration).
 */
void touch_helper_disable_irq(touch_helper_handle_t *h);

#ifdef __cplusplus
}
#endif

