// Copied from managed component waveshare__qmi8658 and extended with WoM INT configuration.
#ifndef QMI8658_H
#define QMI8658_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define QMI8658_LIBRARY_VERSION "0.1.0"

#define QMI8658_ADDRESS_LOW  0x6A
#define QMI8658_ADDRESS_HIGH 0x6B

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif
#define ONE_G (9.807f)

#define QMI8658_DISABLE_ALL    0x00
#define QMI8658_ENABLE_ACCEL   0x01
#define QMI8658_ENABLE_GYRO    0x02
#define QMI8658_ENABLE_MAG     0x04
#define QMI8658_ENABLE_AE      0x08

// CTRL9 host command protocol (QMI8658A)
#define QMI8658_CTRL_CMD_ACK               0x00
#define QMI8658_CTRL_CMD_WRITE_WOM_SETTING 0x08

typedef enum {
    QMI8658_WHO_AM_I        = 0x00,
    QMI8658_REVISION        = 0x01,
    QMI8658_CTRL1           = 0x02,
    QMI8658_CTRL2           = 0x03,
    QMI8658_CTRL3           = 0x04,
    QMI8658_CTRL4           = 0x05,
    QMI8658_CTRL5           = 0x06,
    QMI8658_CTRL6           = 0x07,
    QMI8658_CTRL7           = 0x08,
    QMI8658_CTRL8           = 0x09,
    QMI8658_CTRL9           = 0x0A,
    QMI8658_STATUSINT       = 0x2D,
    QMI8658_STATUS0         = 0x2E,
    QMI8658_STATUS1         = 0x2F,
    QMI8658_TIMESTAMP_L     = 0x30,
    QMI8658_TIMESTAMP_M     = 0x31,
    QMI8658_TIMESTAMP_H     = 0x32,
    QMI8658_TEMP_L          = 0x33,
    QMI8658_TEMP_H          = 0x34,
    QMI8658_AX_L            = 0x35,
    QMI8658_AX_H            = 0x36,
    QMI8658_AY_L            = 0x37,
    QMI8658_AY_H            = 0x38,
    QMI8658_AZ_L            = 0x39,
    QMI8658_AZ_H            = 0x3A,
    QMI8658_GX_L            = 0x3B,
    QMI8658_GX_H            = 0x3C,
    QMI8658_GY_L            = 0x3D,
    QMI8658_GY_H            = 0x3E,
    QMI8658_GZ_L            = 0x3F,
    QMI8658_GZ_H            = 0x40
} qmi8658_register_t;

typedef enum {
    QMI8658_ACCEL_RANGE_2G  = 0x00,
    QMI8658_ACCEL_RANGE_4G  = 0x01,
    QMI8658_ACCEL_RANGE_8G  = 0x02,
    QMI8658_ACCEL_RANGE_16G = 0x03
} qmi8658_accel_range_t;

typedef enum {
    QMI8658_ACCEL_ODR_8000HZ    = 0x00,
    QMI8658_ACCEL_ODR_4000HZ    = 0x01,
    QMI8658_ACCEL_ODR_2000HZ    = 0x02,
    QMI8658_ACCEL_ODR_1000HZ    = 0x03,
    QMI8658_ACCEL_ODR_500HZ     = 0x04,
    QMI8658_ACCEL_ODR_250HZ     = 0x05,
    QMI8658_ACCEL_ODR_125HZ     = 0x06,
    QMI8658_ACCEL_ODR_62_5HZ    = 0x07,
    QMI8658_ACCEL_ODR_31_25HZ   = 0x08,
    QMI8658_ACCEL_ODR_LOWPOWER_128HZ = 0x0C,
    QMI8658_ACCEL_ODR_LOWPOWER_21HZ  = 0x0D,
    QMI8658_ACCEL_ODR_LOWPOWER_11HZ  = 0x0E,
    QMI8658_ACCEL_ODR_LOWPOWER_3HZ   = 0x0F
} qmi8658_accel_odr_t;

typedef enum {
    QMI8658_GYRO_RANGE_32DPS   = 0x00,
    QMI8658_GYRO_RANGE_64DPS   = 0x01,
    QMI8658_GYRO_RANGE_128DPS  = 0x02,
    QMI8658_GYRO_RANGE_256DPS  = 0x03,
    QMI8658_GYRO_RANGE_512DPS  = 0x04,
    QMI8658_GYRO_RANGE_1024DPS = 0x05,
    QMI8658_GYRO_RANGE_2048DPS = 0x06,
    QMI8658_GYRO_RANGE_4096DPS = 0x07
} qmi8658_gyro_range_t;

typedef enum {
    QMI8658_GYRO_ODR_8000HZ   = 0x00,
    QMI8658_GYRO_ODR_4000HZ   = 0x01,
    QMI8658_GYRO_ODR_2000HZ   = 0x02,
    QMI8658_GYRO_ODR_1000HZ   = 0x03,
    QMI8658_GYRO_ODR_500HZ    = 0x04,
    QMI8658_GYRO_ODR_250HZ    = 0x05,
    QMI8658_GYRO_ODR_125HZ    = 0x06,
    QMI8658_GYRO_ODR_62_5HZ   = 0x07,
    QMI8658_GYRO_ODR_31_25HZ  = 0x08
} qmi8658_gyro_odr_t;

typedef enum {
    QMI8658_PRECISION_2 = 2,
    QMI8658_PRECISION_4 = 4,
    QMI8658_PRECISION_6 = 6
} qmi8658_precision_t;

typedef struct {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temperature;
    uint32_t timestamp;
} qmi8658_data_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint16_t accel_lsb_div;
    uint16_t gyro_lsb_div;
    bool accel_unit_mps2;
    bool gyro_unit_rads;
    int display_precision;
    uint32_t timestamp;
} qmi8658_dev_t;

// Interrupt pins on QMI8658A can be enabled/disabled (push-pull vs High-Z) via CTRL1.
typedef enum {
    QMI8658_INT_PIN1 = 1,
    QMI8658_INT_PIN2 = 2,
} qmi8658_int_pin_t;

// Enable/disable INT pin output driver (CTRL1.bit3 INT1_EN, CTRL1.bit4 INT2_EN).
esp_err_t qmi8658_enable_int_pin(qmi8658_dev_t *dev, qmi8658_int_pin_t pin, bool enable);

esp_err_t qmi8658_init(qmi8658_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

esp_err_t qmi8658_set_accel_range(qmi8658_dev_t *dev, qmi8658_accel_range_t range);
esp_err_t qmi8658_set_accel_odr(qmi8658_dev_t *dev, qmi8658_accel_odr_t odr);
esp_err_t qmi8658_set_gyro_range(qmi8658_dev_t *dev, qmi8658_gyro_range_t range);
esp_err_t qmi8658_set_gyro_odr(qmi8658_dev_t *dev, qmi8658_gyro_odr_t odr);

esp_err_t qmi8658_enable_accel(qmi8658_dev_t *dev, bool enable);
esp_err_t qmi8658_enable_gyro(qmi8658_dev_t *dev, bool enable);
esp_err_t qmi8658_enable_sensors(qmi8658_dev_t *dev, uint8_t enable_flags);

esp_err_t qmi8658_read_accel(qmi8658_dev_t *dev, float *x, float *y, float *z);
esp_err_t qmi8658_read_gyro(qmi8658_dev_t *dev, float *x, float *y, float *z);
esp_err_t qmi8658_read_temp(qmi8658_dev_t *dev, float *temperature);
esp_err_t qmi8658_read_sensor_data(qmi8658_dev_t *dev, qmi8658_data_t *data);

esp_err_t qmi8658_is_data_ready(qmi8658_dev_t *dev, bool *ready);
esp_err_t qmi8658_get_who_am_i(qmi8658_dev_t *dev, uint8_t *who_am_i);
esp_err_t qmi8658_reset(qmi8658_dev_t *dev);

void qmi8658_set_accel_unit_mps2(qmi8658_dev_t *dev, bool use_mps2);
void qmi8658_set_accel_unit_mg(qmi8658_dev_t *dev, bool use_mg);
void qmi8658_set_gyro_unit_rads(qmi8658_dev_t *dev, bool use_rads);
void qmi8658_set_gyro_unit_dps(qmi8658_dev_t *dev, bool use_dps);

void qmi8658_set_display_precision(qmi8658_dev_t *dev, int decimals);
void qmi8658_set_display_precision_enum(qmi8658_dev_t *dev, qmi8658_precision_t precision);
int qmi8658_get_display_precision(qmi8658_dev_t *dev);

bool qmi8658_is_accel_unit_mps2(qmi8658_dev_t *dev);
bool qmi8658_is_accel_unit_mg(qmi8658_dev_t *dev);
bool qmi8658_is_gyro_unit_rads(qmi8658_dev_t *dev);
bool qmi8658_is_gyro_unit_dps(qmi8658_dev_t *dev);

// Wake-on-motion initial INT output level (as encoded in CAL1_H per datasheet WoM section).
typedef enum {
    QMI8658_WOM_INITIAL_LEVEL_LOW = 0,
    QMI8658_WOM_INITIAL_LEVEL_HIGH = 1,
} qmi8658_wom_initial_level_t;

// Wake-on-motion configuration (QMI8658A): threshold in mg (1mg/LSB), routing to INT1/INT2,
// initial output level, and blanking time in accelerometer samples (0..63).
typedef struct {
    qmi8658_int_pin_t int_pin;                 // INT1 or INT2 (CAL1_H[7:6] selection)
    qmi8658_wom_initial_level_t initial_level; // initial output level after STATUS1 clear
    uint8_t threshold_mg;                      // CAL1_L (0 disables WoM)
    uint8_t blanking_samples;                  // CAL1_H[5:0] (in accel samples)
} qmi8658_wom_config_t;

// Enable wake-on-motion using the full configuration.
esp_err_t qmi8658_enable_wake_on_motion_cfg(qmi8658_dev_t *dev, const qmi8658_wom_config_t *cfg);

// Backwards-compatible API: explicit initial level, routes to INT1, blanking default.
// Use INITIAL_LEVEL_HIGH when using ESP32 EXT1 ANY_LOW (motion toggles the line LOW).
esp_err_t qmi8658_enable_wake_on_motion_ex(qmi8658_dev_t *dev, uint8_t threshold, qmi8658_wom_initial_level_t initial_level);

// Enable wake-on-motion with default semantics.
esp_err_t qmi8658_enable_wake_on_motion(qmi8658_dev_t *dev, uint8_t threshold);
esp_err_t qmi8658_disable_wake_on_motion(qmi8658_dev_t *dev);

// Read STATUS1 to clear WoM latch and restore INT to the configured initial level.
esp_err_t qmi8658_wom_clear_status(qmi8658_dev_t *dev);

// Optional: dump key WoM-related registers for field debugging.
// (CAL1_L/H, CTRL1, CTRL2, STATUS1, STATUSINT)
esp_err_t qmi8658_wom_dump_regs(qmi8658_dev_t *dev);

esp_err_t qmi8658_write_register(qmi8658_dev_t *dev, uint8_t reg, uint8_t value);
esp_err_t qmi8658_read_register(qmi8658_dev_t *dev, uint8_t reg, uint8_t *buffer, uint8_t length);

#endif // QMI8658_H
