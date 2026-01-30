#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Minimal ESP-IDF port of the Arduino HDC2080 API.
 *
 * Contract:
 *  - Call hdc2080_init() once with a configured I2C bus + address.
 *  - All functions return ESP_OK on success (or a value + out-err where appropriate).
 */

typedef struct {
    uint8_t addr;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} hdc2080_t;

/* Register map */
#define HDC2080_REG_TEMP_LOW           0x00
#define HDC2080_REG_TEMP_HIGH          0x01
#define HDC2080_REG_HUMID_LOW          0x02
#define HDC2080_REG_HUMID_HIGH         0x03
#define HDC2080_REG_INTERRUPT_DRDY     0x04
#define HDC2080_REG_TEMP_MAX           0x05
#define HDC2080_REG_HUMID_MAX          0x06
#define HDC2080_REG_INTERRUPT_CONFIG   0x07
#define HDC2080_REG_TEMP_OFFSET_ADJUST 0x08
#define HDC2080_REG_HUM_OFFSET_ADJUST  0x09
#define HDC2080_REG_TEMP_THR_L         0x0A
#define HDC2080_REG_TEMP_THR_H         0x0B
#define HDC2080_REG_HUMID_THR_L        0x0C
#define HDC2080_REG_HUMID_THR_H        0x0D
#define HDC2080_REG_CONFIG             0x0E
#define HDC2080_REG_MEASUREMENT_CONFIG 0x0F
#define HDC2080_REG_MANUFACTURER_ID_LOW 0xFC
#define HDC2080_REG_MANUFACTURER_ID_HIGH 0xFD
#define HDC2080_REG_DEVICE_ID_LOW        0xFE
#define HDC2080_REG_DEVICE_ID_HIGH        0xFF

typedef enum {
    HDC2080_TEMP_AND_HUMID = 0,
    HDC2080_TEMP_ONLY      = 1,
    HDC2080_HUMID_ONLY     = 2,
} HDC2080_InterruptMode;

/* Arduino-style enums */
enum {
    HDC2080_NINE_BIT     = 0,
    HDC2080_ELEVEN_BIT   = 1,
    HDC2080_FOURTEEN_BIT = 2,
};


enum {
    HDC2080_MANUAL      = 0,
    HDC2080_TWO_MINS    = 1,
    HDC2080_ONE_MINS    = 2,
    HDC2080_TEN_SECONDS = 3,
    HDC2080_FIVE_SECONDS= 4,
    HDC2080_ONE_HZ      = 5,
    HDC2080_TWO_HZ      = 6,
    HDC2080_FIVE_HZ     = 7,
};

enum {
    HDC2080_ACTIVE_LOW  = 0,
    HDC2080_ACTIVE_HIGH = 1,
};

enum {
    HDC2080_LEVEL_MODE      = 0,
    HDC2080_COMPARATOR_MODE = 1,
};

/** Initialize the device handle on an existing I2C master bus. */
esp_err_t hdc2080_init(hdc2080_t *s, i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_speed_hz);

/** Remove device from bus (optional cleanup). */
esp_err_t hdc2080_deinit(hdc2080_t *s);

bool hdc2080_is_connected(const hdc2080_t *s);

esp_err_t hdc2080_read_reg(const hdc2080_t *s, uint8_t reg, uint8_t *out);
esp_err_t hdc2080_write_reg(const hdc2080_t *s, uint8_t reg, uint8_t data);

esp_err_t hdc2080_trigger_measurement(const hdc2080_t *s);

esp_err_t hdc2080_read_temp_c(const hdc2080_t *s, float *out_temp_c);
esp_err_t hdc2080_read_humidity_rh(const hdc2080_t *s, float *out_humidity);

esp_err_t hdc2080_read_temp_offset_adjust(const hdc2080_t *s, uint8_t *out);
esp_err_t hdc2080_set_temp_offset_adjust(const hdc2080_t *s, uint8_t value, uint8_t *out_new);

esp_err_t hdc2080_read_humidity_offset_adjust(const hdc2080_t *s, uint8_t *out);
esp_err_t hdc2080_set_humidity_offset_adjust(const hdc2080_t *s, uint8_t value, uint8_t *out_new);

esp_err_t hdc2080_enable_threshold_interrupt(const hdc2080_t *s, HDC2080_InterruptMode mode);
esp_err_t hdc2080_disable_threshold_interrupt(const hdc2080_t *s);

esp_err_t hdc2080_enable_heater(const hdc2080_t *s);
esp_err_t hdc2080_disable_heater(const hdc2080_t *s);

esp_err_t hdc2080_set_low_temp(const hdc2080_t *s, float temp_c);
esp_err_t hdc2080_set_high_temp(const hdc2080_t *s, float temp_c);
esp_err_t hdc2080_set_low_humidity(const hdc2080_t *s, float humid_rh);
esp_err_t hdc2080_set_high_humidity(const hdc2080_t *s, float humid_rh);

esp_err_t hdc2080_read_low_humidity_threshold(const hdc2080_t *s, float *out_humid);
esp_err_t hdc2080_read_high_humidity_threshold(const hdc2080_t *s, float *out_humid);
esp_err_t hdc2080_read_low_temp_threshold(const hdc2080_t *s, float *out_temp);
esp_err_t hdc2080_read_high_temp_threshold(const hdc2080_t *s, float *out_temp);

esp_err_t hdc2080_set_temp_res(const hdc2080_t *s, int resolution);
esp_err_t hdc2080_set_humid_res(const hdc2080_t *s, int resolution);
esp_err_t hdc2080_set_measurement_mode(const hdc2080_t *s, int mode);

esp_err_t hdc2080_reset(const hdc2080_t *s);

esp_err_t hdc2080_enable_interrupt(const hdc2080_t *s);
esp_err_t hdc2080_disable_interrupt(const hdc2080_t *s);

esp_err_t hdc2080_set_rate(const hdc2080_t *s, int rate);
esp_err_t hdc2080_set_interrupt_polarity(const hdc2080_t *s, int polarity);
esp_err_t hdc2080_set_interrupt_mode(const hdc2080_t *s, int mode);

esp_err_t hdc2080_read_interrupt_status(const hdc2080_t *s, uint8_t *out);

esp_err_t hdc2080_clear_max_temp(const hdc2080_t *s);
esp_err_t hdc2080_clear_max_humidity(const hdc2080_t *s);
esp_err_t hdc2080_read_max_temp(const hdc2080_t *s, float *out);
esp_err_t hdc2080_read_max_humidity(const hdc2080_t *s, float *out);

esp_err_t hdc2080_enable_drdy_interrupt(const hdc2080_t *s);
esp_err_t hdc2080_disable_drdy_interrupt(const hdc2080_t *s);

esp_err_t hdc2080_start_auto_measurement(const hdc2080_t *s);

#ifdef __cplusplus
}
#endif
