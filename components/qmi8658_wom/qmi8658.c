#include "qmi8658.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "QMI8658";

// CAL1_L / CAL1_H registers are used for WoM settings on QMI8658A
#define QMI8658_CAL1_L_REG  0x0B
#define QMI8658_CAL1_H_REG  0x0C

// STATUSINT.bit7 = CmdDone for CTRL9 host command protocol
#define QMI8658_STATUSINT_CMD_DONE_BIT   (1u << 7)

// Compile-time switch for WoM register dumps (keep logs quiet in production).
#ifndef QMI8658_WOM_DEBUG
#define QMI8658_WOM_DEBUG 1
#endif

// CAL1_H[7:6] selection per datasheet:
//  00 INT1 init=0
//  10 INT1 init=1
//  01 INT2 init=0
//  11 INT2 init=1
#define QMI8658_WOM_SEL_INT1_INIT0 0x00
#define QMI8658_WOM_SEL_INT1_INIT1 0x80
#define QMI8658_WOM_SEL_INT2_INIT0 0x40
#define QMI8658_WOM_SEL_INT2_INIT1 0xC0

static uint8_t qmi8658_wom_encode_cal1_h(qmi8658_int_pin_t pin,
                                        qmi8658_wom_initial_level_t initial_level,
                                        uint8_t blanking_samples)
{
    const uint8_t blank = (uint8_t)(blanking_samples & 0x3F);
    uint8_t sel = QMI8658_WOM_SEL_INT1_INIT0;

    if (pin == QMI8658_INT_PIN1) {
        sel = (initial_level == QMI8658_WOM_INITIAL_LEVEL_HIGH) ? QMI8658_WOM_SEL_INT1_INIT1
                                                                : QMI8658_WOM_SEL_INT1_INIT0;
    } else { // QMI8658_INT_PIN2
        sel = (initial_level == QMI8658_WOM_INITIAL_LEVEL_HIGH) ? QMI8658_WOM_SEL_INT2_INIT1
                                                                : QMI8658_WOM_SEL_INT2_INIT0;
    }

    return (uint8_t)(sel | blank);
}

static void qmi8658_wom_decode_cal1_h(uint8_t cal1_h,
                                     qmi8658_int_pin_t *pin_out,
                                     qmi8658_wom_initial_level_t *level_out,
                                     uint8_t *blank_out)
{
    *blank_out = (uint8_t)(cal1_h & 0x3F);

    const uint8_t sel = (uint8_t)(cal1_h & 0xC0);
    *pin_out = (sel == QMI8658_WOM_SEL_INT2_INIT0 || sel == QMI8658_WOM_SEL_INT2_INIT1) ? QMI8658_INT_PIN2 : QMI8658_INT_PIN1;

    *level_out = (sel == QMI8658_WOM_SEL_INT1_INIT1 || sel == QMI8658_WOM_SEL_INT2_INIT1)
                     ? QMI8658_WOM_INITIAL_LEVEL_HIGH
                     : QMI8658_WOM_INITIAL_LEVEL_LOW;
}

static esp_err_t qmi8658_set_int_pin_output(qmi8658_dev_t *dev, bool int1_enable, bool int2_enable)
{
    uint8_t ctrl1;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL1, &ctrl1, 1);
    if (ret != ESP_OK) return ret;

    // CTRL1.bit3 = INT1_EN, CTRL1.bit4 = INT2_EN
    ctrl1 &= (uint8_t)~((1u << 3) | (1u << 4));
    if (int1_enable) ctrl1 |= (1u << 3);
    if (int2_enable) ctrl1 |= (1u << 4);

    return qmi8658_write_register(dev, QMI8658_CTRL1, ctrl1);
}

esp_err_t qmi8658_enable_int_pin(qmi8658_dev_t *dev, qmi8658_int_pin_t pin, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Read current state and only toggle the requested pin.
    uint8_t ctrl1;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL1, &ctrl1, 1);
    if (ret != ESP_OK) return ret;

    bool int1_en = (ctrl1 & (1u << 3)) != 0;
    bool int2_en = (ctrl1 & (1u << 4)) != 0;

    if (pin == QMI8658_INT_PIN1) {
        int1_en = enable;
    } else if (pin == QMI8658_INT_PIN2) {
        int2_en = enable;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return qmi8658_set_int_pin_output(dev, int1_en, int2_en);
}

#define QMI8658_CTRL8_HOST_CMD_STATUSINT_EN_BIT (1u << 7)

static esp_err_t qmi8658_ctrl9_set_handshake_statusint(qmi8658_dev_t *dev)
{
    uint8_t ctrl8 = 0;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL8, &ctrl8, 1);
    if (ret != ESP_OK) return ret;

    if ((ctrl8 & QMI8658_CTRL8_HOST_CMD_STATUSINT_EN_BIT) == 0) {
        ctrl8 |= QMI8658_CTRL8_HOST_CMD_STATUSINT_EN_BIT;
        ret = qmi8658_write_register(dev, QMI8658_CTRL8, ctrl8);
        if (ret != ESP_OK) return ret;

        // Read back once to catch I2C issues.
        uint8_t rb = 0;
        ret = qmi8658_read_register(dev, QMI8658_CTRL8, &rb, 1);
        if (ret != ESP_OK) return ret;
#if QMI8658_WOM_DEBUG
        ESP_LOGI(TAG, "CTRL8 handshake set: 0x%02X", rb);
#endif
    }

    return ESP_OK;
}

static void qmi8658_dump_ctrl9_snapshot(qmi8658_dev_t *dev, const char *reason)
{
#if QMI8658_WOM_DEBUG
    uint8_t ctrl8 = 0, ctrl9 = 0, statusint = 0, status1 = 0, ctrl1 = 0, ctrl2 = 0, ctrl7 = 0;
    (void)qmi8658_read_register(dev, QMI8658_CTRL8, &ctrl8, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL9, &ctrl9, 1);
    (void)qmi8658_read_register(dev, QMI8658_STATUSINT, &statusint, 1);
    (void)qmi8658_read_register(dev, QMI8658_STATUS1, &status1, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL1, &ctrl1, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL2, &ctrl2, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL7, &ctrl7, 1);

    ESP_LOGI(TAG, "[CTRL9] %s: CTRL8=0x%02X CTRL9=0x%02X STATUSINT=0x%02X STATUS1=0x%02X CTRL1=0x%02X CTRL2=0x%02X CTRL7=0x%02X",
             reason ? reason : "snapshot", ctrl8, ctrl9, statusint, status1, ctrl1, ctrl2, ctrl7);
#else
    (void)dev;
    (void)reason;
#endif
}

static esp_err_t qmi8658_ctrl9_exec(qmi8658_dev_t *dev, uint8_t cmd) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Ensure we're using STATUSINT polling handshake; otherwise CmdDone may not behave as expected.
    esp_err_t ret = qmi8658_ctrl9_set_handshake_statusint(dev);
    if (ret != ESP_OK) return ret;

    // Clear any stale CmdDone first so we don't wait on an old completion state.
    (void)qmi8658_write_register(dev, QMI8658_CTRL9, QMI8658_CTRL_CMD_ACK);
    uint8_t statusint = 0;
    (void)qmi8658_read_register(dev, QMI8658_STATUSINT, &statusint, 1);

    // Issue the host command (e.g. WRITE_WOM_SETTING).
    ret = qmi8658_write_register(dev, QMI8658_CTRL9, cmd);
    if (ret != ESP_OK) {
        qmi8658_dump_ctrl9_snapshot(dev, "CTRL9 write cmd failed");
        return ret;
    }

    // Give the device a moment before polling.
    vTaskDelay(pdMS_TO_TICKS(2));

    // Poll STATUSINT.bit7 (CmdDone).
    for (int i = 0; i < 200; i++) {
        statusint = 0;
        ret = qmi8658_read_register(dev, QMI8658_STATUSINT, &statusint, 1);
        if (ret != ESP_OK) {
            qmi8658_dump_ctrl9_snapshot(dev, "CTRL9 poll read failed");
            return ret;
        }

        if (statusint & QMI8658_STATUSINT_CMD_DONE_BIT) {
            // ACK to end protocol
            return qmi8658_write_register(dev, QMI8658_CTRL9, QMI8658_CTRL_CMD_ACK);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    qmi8658_dump_ctrl9_snapshot(dev, "CTRL9 timeout");
    ESP_LOGW(TAG, "CTRL9 cmd 0x%02X timed out; STATUSINT=0x%02X", cmd, statusint);
    return ESP_ERR_TIMEOUT;
}

esp_err_t qmi8658_init(qmi8658_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr) {
    if (!dev || !bus_handle) return ESP_ERR_INVALID_ARG;

    dev->bus_handle = bus_handle;
    dev->accel_lsb_div = 4096;
    dev->gyro_lsb_div = 64;
    dev->accel_unit_mps2 = false;
    dev->gyro_unit_rads = false;
    dev->display_precision = 6;
    dev->timestamp = 0;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ret;
    }

    uint8_t who_am_i;
    ret = qmi8658_get_who_am_i(dev, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    if (who_am_i != 0x05) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value: 0x%02X, expected 0x05", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    ret = qmi8658_write_register(dev, QMI8658_CTRL1, 0x60);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        return ret;
    }

    // Default to STATUSINT handshake for CTRL9 host command protocol.
    // (Safe no-op if bit is already set.)
    ret = qmi8658_ctrl9_set_handshake_statusint(dev);
    if (ret != ESP_OK) return ret;

    ret = qmi8658_set_accel_range(dev, QMI8658_ACCEL_RANGE_8G);
    if (ret != ESP_OK) return ret;

    ret = qmi8658_set_accel_odr(dev, QMI8658_ACCEL_ODR_1000HZ);
    if (ret != ESP_OK) return ret;

    ret = qmi8658_set_gyro_range(dev, QMI8658_GYRO_RANGE_512DPS);
    if (ret != ESP_OK) return ret;

    ret = qmi8658_set_gyro_odr(dev, QMI8658_GYRO_ODR_1000HZ);
    if (ret != ESP_OK) return ret;

    ret = qmi8658_enable_sensors(dev, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);


    ESP_LOGI(TAG, "QMI8658 initialized successfully");
    return ret;
}

esp_err_t qmi8658_set_accel_range(qmi8658_dev_t *dev, qmi8658_accel_range_t range) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    switch (range) {
        case QMI8658_ACCEL_RANGE_2G:
            dev->accel_lsb_div = 16384;
            break;
        case QMI8658_ACCEL_RANGE_4G:
            dev->accel_lsb_div = 8192;
            break;
        case QMI8658_ACCEL_RANGE_8G:
            dev->accel_lsb_div = 4096;
            break;
        case QMI8658_ACCEL_RANGE_16G:
            dev->accel_lsb_div = 2048;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return qmi8658_write_register(dev, QMI8658_CTRL2, (range << 4) | 0x03);
}

esp_err_t qmi8658_set_accel_odr(qmi8658_dev_t *dev, qmi8658_accel_odr_t odr) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t current_ctrl2;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL2, &current_ctrl2, 1);
    if (ret != ESP_OK) return ret;

    uint8_t new_ctrl2 = (current_ctrl2 & 0xF0) | odr;
    return qmi8658_write_register(dev, QMI8658_CTRL2, new_ctrl2);
}

esp_err_t qmi8658_set_gyro_range(qmi8658_dev_t *dev, qmi8658_gyro_range_t range) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    switch (range) {
        case QMI8658_GYRO_RANGE_32DPS:
            dev->gyro_lsb_div = 1024;
            break;
        case QMI8658_GYRO_RANGE_64DPS:
            dev->gyro_lsb_div = 512;
            break;
        case QMI8658_GYRO_RANGE_128DPS:
            dev->gyro_lsb_div = 256;
            break;
        case QMI8658_GYRO_RANGE_256DPS:
            dev->gyro_lsb_div = 128;
            break;
        case QMI8658_GYRO_RANGE_512DPS:
            dev->gyro_lsb_div = 64;
            break;
        case QMI8658_GYRO_RANGE_1024DPS:
            dev->gyro_lsb_div = 32;
            break;
        case QMI8658_GYRO_RANGE_2048DPS:
            dev->gyro_lsb_div = 16;
            break;
        case QMI8658_GYRO_RANGE_4096DPS:
            dev->gyro_lsb_div = 8;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return qmi8658_write_register(dev, QMI8658_CTRL3, (range << 4) | 0x03);
}

esp_err_t qmi8658_set_gyro_odr(qmi8658_dev_t *dev, qmi8658_gyro_odr_t odr) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t current_ctrl3;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL3, &current_ctrl3, 1);
    if (ret != ESP_OK) return ret;

    uint8_t new_ctrl3 = (current_ctrl3 & 0xF0) | odr;
    return qmi8658_write_register(dev, QMI8658_CTRL3, new_ctrl3);
}

esp_err_t qmi8658_enable_accel(qmi8658_dev_t *dev, bool enable) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t current_ctrl7;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL7, &current_ctrl7, 1);
    if (ret != ESP_OK) return ret;

    if (enable) {
        current_ctrl7 |= QMI8658_ENABLE_ACCEL;
    } else {
        current_ctrl7 &= (uint8_t)~QMI8658_ENABLE_ACCEL;
    }

    return qmi8658_write_register(dev, QMI8658_CTRL7, current_ctrl7);
}

esp_err_t qmi8658_enable_gyro(qmi8658_dev_t *dev, bool enable) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t current_ctrl7;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL7, &current_ctrl7, 1);
    if (ret != ESP_OK) return ret;

    if (enable) {
        current_ctrl7 |= QMI8658_ENABLE_GYRO;
    } else {
        current_ctrl7 &= (uint8_t)~QMI8658_ENABLE_GYRO;
    }

    return qmi8658_write_register(dev, QMI8658_CTRL7, current_ctrl7);
}

esp_err_t qmi8658_enable_sensors(qmi8658_dev_t *dev, uint8_t enable_flags) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return qmi8658_write_register(dev, QMI8658_CTRL7, enable_flags & 0x0F);
}

esp_err_t qmi8658_read_accel(qmi8658_dev_t *dev, float *x, float *y, float *z) {
    if (!dev || !x || !y || !z) return ESP_ERR_INVALID_ARG;

    uint8_t buffer[6];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_AX_L, buffer, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    if (dev->accel_unit_mps2) {
        *x = ((float)raw_x * (float)ONE_G) / (float)dev->accel_lsb_div;
        *y = ((float)raw_y * (float)ONE_G) / (float)dev->accel_lsb_div;
        *z = ((float)raw_z * (float)ONE_G) / (float)dev->accel_lsb_div;
    } else {
        *x = ((float)raw_x * 1000.0f) / (float)dev->accel_lsb_div;
        *y = ((float)raw_y * 1000.0f) / (float)dev->accel_lsb_div;
        *z = ((float)raw_z * 1000.0f) / (float)dev->accel_lsb_div;
    }

    return ESP_OK;
}

esp_err_t qmi8658_read_gyro(qmi8658_dev_t *dev, float *x, float *y, float *z) {
    if (!dev || !x || !y || !z) return ESP_ERR_INVALID_ARG;

    uint8_t buffer[6];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_GX_L, buffer, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    if (dev->gyro_unit_rads) {
        *x = ((float)raw_x * (float)M_PI / 180.0f) / (float)dev->gyro_lsb_div;
        *y = ((float)raw_y * (float)M_PI / 180.0f) / (float)dev->gyro_lsb_div;
        *z = ((float)raw_z * (float)M_PI / 180.0f) / (float)dev->gyro_lsb_div;
    } else {
        *x = (float)raw_x / (float)dev->gyro_lsb_div;
        *y = (float)raw_y / (float)dev->gyro_lsb_div;
        *z = (float)raw_z / (float)dev->gyro_lsb_div;
    }

    return ESP_OK;
}

esp_err_t qmi8658_read_temp(qmi8658_dev_t *dev, float *temperature) {
    if (!dev || !temperature) return ESP_ERR_INVALID_ARG;

    uint8_t buffer[2];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_TEMP_L, buffer, 2);
    if (ret != ESP_OK) return ret;

    int16_t raw_temp = (int16_t)((buffer[1] << 8) | buffer[0]);
    *temperature = (float)raw_temp / 256.0f;

    return ESP_OK;
}

esp_err_t qmi8658_read_sensor_data(qmi8658_dev_t *dev, qmi8658_data_t *data) {
    if (!dev || !data) return ESP_ERR_INVALID_ARG;

    uint8_t timestamp_buffer[3];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_TIMESTAMP_L, timestamp_buffer, 3);
    if (ret == ESP_OK) {
        uint32_t timestamp = ((uint32_t)timestamp_buffer[2] << 16) |
                             ((uint32_t)timestamp_buffer[1] << 8) |
                             timestamp_buffer[0];
        if (timestamp > dev->timestamp) {
            dev->timestamp = timestamp;
        } else {
            dev->timestamp = (timestamp + 0x1000000 - dev->timestamp);
        }
        data->timestamp = dev->timestamp;
    }

    uint8_t sensor_buffer[12];
    ret = qmi8658_read_register(dev, QMI8658_AX_L, sensor_buffer, 12);
    if (ret != ESP_OK) return ret;

    int16_t raw_ax = (int16_t)((sensor_buffer[1] << 8) | sensor_buffer[0]);
    int16_t raw_ay = (int16_t)((sensor_buffer[3] << 8) | sensor_buffer[2]);
    int16_t raw_az = (int16_t)((sensor_buffer[5] << 8) | sensor_buffer[4]);

    int16_t raw_gx = (int16_t)((sensor_buffer[7] << 8) | sensor_buffer[6]);
    int16_t raw_gy = (int16_t)((sensor_buffer[9] << 8) | sensor_buffer[8]);
    int16_t raw_gz = (int16_t)((sensor_buffer[11] << 8) | sensor_buffer[10]);

    if (dev->accel_unit_mps2) {
        data->accelX = ((float)raw_ax * (float)ONE_G) / (float)dev->accel_lsb_div;
        data->accelY = ((float)raw_ay * (float)ONE_G) / (float)dev->accel_lsb_div;
        data->accelZ = ((float)raw_az * (float)ONE_G) / (float)dev->accel_lsb_div;
    } else {
        data->accelX = ((float)raw_ax * 1000.0f) / (float)dev->accel_lsb_div;
        data->accelY = ((float)raw_ay * 1000.0f) / (float)dev->accel_lsb_div;
        data->accelZ = ((float)raw_az * 1000.0f) / (float)dev->accel_lsb_div;
    }

    if (dev->gyro_unit_rads) {
        data->gyroX = ((float)raw_gx * (float)M_PI / 180.0f) / (float)dev->gyro_lsb_div;
        data->gyroY = ((float)raw_gy * (float)M_PI / 180.0f) / (float)dev->gyro_lsb_div;
        data->gyroZ = ((float)raw_gz * (float)M_PI / 180.0f) / (float)dev->gyro_lsb_div;
    } else {
        data->gyroX = (float)raw_gx / (float)dev->gyro_lsb_div;
        data->gyroY = (float)raw_gy / (float)dev->gyro_lsb_div;
        data->gyroZ = (float)raw_gz / (float)dev->gyro_lsb_div;
    }

    return qmi8658_read_temp(dev, &data->temperature);
}

esp_err_t qmi8658_is_data_ready(qmi8658_dev_t *dev, bool *ready) {
    if (!dev || !ready) return ESP_ERR_INVALID_ARG;

    uint8_t status;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_STATUS0, &status, 1);
    if (ret != ESP_OK) return ret;

    *ready = (status & 0x03) != 0;
    return ESP_OK;
}

esp_err_t qmi8658_get_who_am_i(qmi8658_dev_t *dev, uint8_t *who_am_i) {
    if (!dev || !who_am_i) return ESP_ERR_INVALID_ARG;
    return qmi8658_read_register(dev, QMI8658_WHO_AM_I, who_am_i, 1);
}

esp_err_t qmi8658_reset(qmi8658_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return qmi8658_write_register(dev, QMI8658_CTRL1, 0x80);
}

void qmi8658_set_accel_unit_mps2(qmi8658_dev_t *dev, bool use_mps2) {
    if (dev) {
        dev->accel_unit_mps2 = use_mps2;
    }
}

void qmi8658_set_accel_unit_mg(qmi8658_dev_t *dev, bool use_mg) {
    if (dev) {
        dev->accel_unit_mps2 = !use_mg;
    }
}

void qmi8658_set_gyro_unit_rads(qmi8658_dev_t *dev, bool use_rads) {
    if (dev) {
        dev->gyro_unit_rads = use_rads;
    }
}

void qmi8658_set_gyro_unit_dps(qmi8658_dev_t *dev, bool use_dps) {
    if (dev) {
        dev->gyro_unit_rads = !use_dps;
    }
}

void qmi8658_set_display_precision(qmi8658_dev_t *dev, int decimals) {
    if (dev && decimals >= 0 && decimals <= 10) {
        dev->display_precision = decimals;
    }
}

void qmi8658_set_display_precision_enum(qmi8658_dev_t *dev, qmi8658_precision_t precision) {
    if (dev) {
        dev->display_precision = (int)precision;
    }
}

int qmi8658_get_display_precision(qmi8658_dev_t *dev) {
    return dev ? dev->display_precision : 0;
}

bool qmi8658_is_accel_unit_mps2(qmi8658_dev_t *dev) {
    return dev ? dev->accel_unit_mps2 : false;
}

bool qmi8658_is_accel_unit_mg(qmi8658_dev_t *dev) {
    return dev ? !dev->accel_unit_mps2 : false;
}

bool qmi8658_is_gyro_unit_rads(qmi8658_dev_t *dev) {
    return dev ? dev->gyro_unit_rads : false;
}

bool qmi8658_is_gyro_unit_dps(qmi8658_dev_t *dev) {
    return dev ? !dev->gyro_unit_rads : false;
}

static esp_err_t qmi8658_clear_wom_status(qmi8658_dev_t *dev)
{
    // Reading STATUS1 clears the WoM flag and returns the INT pin to its initial level.
    uint8_t status1 = 0;
    return qmi8658_read_register(dev, QMI8658_STATUS1, &status1, 1);
}

esp_err_t qmi8658_wom_clear_status(qmi8658_dev_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return qmi8658_clear_wom_status(dev);
}

esp_err_t qmi8658_wom_dump_regs(qmi8658_dev_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t cal1_l = 0, cal1_h = 0, ctrl1 = 0, ctrl2 = 0, status1 = 0, statusint = 0;
    uint8_t ctrl8 = 0, ctrl9 = 0;
    (void)qmi8658_read_register(dev, QMI8658_CAL1_L_REG, &cal1_l, 1);
    (void)qmi8658_read_register(dev, QMI8658_CAL1_H_REG, &cal1_h, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL1, &ctrl1, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL2, &ctrl2, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL8, &ctrl8, 1);
    (void)qmi8658_read_register(dev, QMI8658_CTRL9, &ctrl9, 1);
    (void)qmi8658_read_register(dev, QMI8658_STATUSINT, &statusint, 1);
    (void)qmi8658_read_register(dev, QMI8658_STATUS1, &status1, 1);

    qmi8658_int_pin_t pin = QMI8658_INT_PIN1;
    qmi8658_wom_initial_level_t lvl = QMI8658_WOM_INITIAL_LEVEL_LOW;
    uint8_t blank = 0;
    qmi8658_wom_decode_cal1_h(cal1_h, &pin, &lvl, &blank);

    ESP_LOGI(TAG, "[WoM] CAL1_L(thr) = %u mg", (unsigned)cal1_l);
    ESP_LOGI(TAG, "[WoM] CAL1_H      = 0x%02X (pin=INT%u init=%s blank=%u samples)",
             cal1_h, (unsigned)pin, (lvl == QMI8658_WOM_INITIAL_LEVEL_HIGH) ? "HIGH" : "LOW",
             (unsigned)blank);
    ESP_LOGI(TAG, "[WoM] CTRL1=0x%02X (INT1_EN=%d INT2_EN=%d)", ctrl1,
             (int)((ctrl1 >> 3) & 1), (int)((ctrl1 >> 4) & 1));
    ESP_LOGI(TAG, "[WoM] CTRL2=0x%02X (ACC_RANGE=%u ACC_ODR=0x%X)", ctrl2,
             (unsigned)((ctrl2 >> 4) & 0x0F), (unsigned)(ctrl2 & 0x0F));
    ESP_LOGI(TAG, "[WoM] CTRL8=0x%02X CTRL9=0x%02X", ctrl8, ctrl9);
    ESP_LOGI(TAG, "[WoM] STATUSINT=0x%02X STATUS1=0x%02X (WoM=%d)", statusint, status1,
             (int)((status1 >> 2) & 1));

    return ESP_OK;
}

static esp_err_t qmi8658_write_wom_cal1_h(qmi8658_dev_t *dev,
                                         qmi8658_int_pin_t int_pin,
                                         qmi8658_wom_initial_level_t initial_level,
                                         uint8_t blanking_samples)
{
    if (int_pin != QMI8658_INT_PIN1 && int_pin != QMI8658_INT_PIN2) return ESP_ERR_INVALID_ARG;

    const uint8_t cal1_h = qmi8658_wom_encode_cal1_h(int_pin, initial_level, blanking_samples);
    return qmi8658_write_register(dev, QMI8658_CAL1_H_REG, cal1_h);
}

esp_err_t qmi8658_enable_wake_on_motion_cfg(qmi8658_dev_t *dev,
                                           const qmi8658_wom_config_t *cfg)
{
    if (!dev || !cfg) return ESP_ERR_INVALID_ARG;

    // Sanitize per datasheet: blanking is 6-bit (0..63)
    uint8_t blank = (cfg->blanking_samples > 63) ? 63 : cfg->blanking_samples;

    // Threshold 0 disables WoM (datasheet). Refuse silently or treat as disable.
    if (cfg->threshold_mg == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Ensure CTRL9 handshake uses STATUSINT polling.
    ret = qmi8658_ctrl9_set_handshake_statusint(dev);
    if (ret != ESP_OK) return ret;

    // -----------------------------
    // Figure 25 sequence (master)
    // -----------------------------

    // 1) Disable sensors: CTRL7[1:0] = 0 (gyro off, accel off while programming)
    ret = qmi8658_enable_sensors(dev, QMI8658_DISABLE_ALL);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(2));

    // 2) Set accelerometer ODR and full-scale (CTRL2)
    // WoM prerequisite: accel low-power ODR must be 11xx.
    ret = qmi8658_set_accel_range(dev, QMI8658_ACCEL_RANGE_8G);
    if (ret != ESP_OK) return ret;

    // Use a low-power ODR (11xx). 21Hz is a good default for WoM.
    ret = qmi8658_set_accel_odr(dev, QMI8658_ACCEL_ODR_LOWPOWER_21HZ);
    if (ret != ESP_OK) return ret;

    // 3) Program WoM threshold + interrupt select/polarity + blanking time (CAL1_L/H)
    // Note (datasheet): writing non-zero threshold clears STATUS1.WoM.
    ret = qmi8658_write_register(dev, QMI8658_CAL1_L_REG, cfg->threshold_mg);
    if (ret != ESP_OK) return ret;

    ret = qmi8658_write_wom_cal1_h(dev, cfg->int_pin, cfg->initial_level, blank);
    if (ret != ESP_OK) return ret;

    // Enable only the selected INT pin output (CTRL1.INTx_EN); otherwise pin may be Hi-Z.
    ret = qmi8658_set_int_pin_output(dev,
                                     cfg->int_pin == QMI8658_INT_PIN1,
                                     cfg->int_pin == QMI8658_INT_PIN2);
    if (ret != ESP_OK) return ret;

    // 4) Execute CTRL9 command to configure WoM mode (WRITE_WOM_SETTING)
    ret = qmi8658_ctrl9_exec(dev, QMI8658_CTRL_CMD_WRITE_WOM_SETTING);
    if (ret != ESP_OK) return ret;

    // 5) Enable accelerometer only (gyro must remain off in WoM)
    ret = qmi8658_enable_sensors(dev, QMI8658_ENABLE_ACCEL);
    if (ret != ESP_OK) return ret;

    // -----------------------------
    // Post-arming hardening
    // -----------------------------
    // Datasheet: Reading STATUS1 clears WoM and resets INT line to initial level.
    // This makes the line deterministic *at this instant*, but a WoM event can still occur afterward.
    (void)qmi8658_clear_wom_status(dev);

#if QMI8658_WOM_DEBUG
    (void)qmi8658_wom_dump_regs(dev);

    // Readback CAL1_H to catch encoding mistakes
    uint8_t cal1_h_rb = 0;
    (void)qmi8658_read_register(dev, QMI8658_CAL1_H_REG, &cal1_h_rb, 1);
    const uint8_t exp = qmi8658_wom_encode_cal1_h(cfg->int_pin, cfg->initial_level, blank);
    if (cal1_h_rb != exp) {
        ESP_LOGW(TAG, "[WoM] CAL1_H mismatch: got 0x%02X expected 0x%02X", cal1_h_rb, exp);
    }
#endif

    return ESP_OK;
}


esp_err_t qmi8658_enable_wake_on_motion_ex(qmi8658_dev_t *dev, uint8_t threshold,
                                          qmi8658_wom_initial_level_t initial_level)
{
    // Backwards-compatible wrapper: INT1, blanking default.
    const qmi8658_wom_config_t cfg = {
        .int_pin = QMI8658_INT_PIN1,
        .initial_level = initial_level,
        .threshold_mg = threshold,
        .blanking_samples = 3, // ~140ms at 21Hz; reduces false wake during arming.
    };
    return qmi8658_enable_wake_on_motion_cfg(dev, &cfg);
}

esp_err_t qmi8658_enable_wake_on_motion(qmi8658_dev_t *dev, uint8_t threshold)
{
    // Preserve legacy behavior: initial LOW (motion toggles HIGH).
    return qmi8658_enable_wake_on_motion_ex(dev, threshold, QMI8658_WOM_INITIAL_LEVEL_LOW);
}

esp_err_t qmi8658_write_register(qmi8658_dev_t *dev, uint8_t reg, uint8_t value) {
    if (!dev || !dev->dev_handle) return ESP_ERR_INVALID_ARG;

    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(dev->dev_handle, data, 2, 1000);
}

esp_err_t qmi8658_read_register(qmi8658_dev_t *dev, uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (!dev || !buffer || length == 0 || !dev->dev_handle) return ESP_ERR_INVALID_ARG;

    return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, buffer, length, 1000);
}
