#include "hdc2080.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HDC2080_ADDR_MIN 0x40
#define HDC2080_ADDR_MAX 0x41

static inline bool hdc2080_addr_valid(uint8_t addr)
{
    return (addr == HDC2080_ADDR_MIN) || (addr == HDC2080_ADDR_MAX);
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

esp_err_t hdc2080_init(hdc2080_t *s, i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_speed_hz)
{
    if (!s || !bus || !hdc2080_addr_valid(addr)) {
        return ESP_ERR_INVALID_ARG;
    }

    s->addr = addr;
    s->bus = bus;
    s->dev = NULL;

    const i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz = scl_speed_hz,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, &s->dev);
}

esp_err_t hdc2080_deinit(hdc2080_t *s)
{
    if (!s || !s->dev) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_master_bus_rm_device(s->dev);
    s->dev = NULL;
    return err;
}

bool hdc2080_is_connected(const hdc2080_t *s)
{
    if (!s || !s->bus) {
        return false;
    }

    if (!hdc2080_addr_valid(s->addr)) {
        return false;
    }

    return i2c_master_probe(s->bus, s->addr, 50) == ESP_OK;
}

esp_err_t hdc2080_read_reg(const hdc2080_t *s, uint8_t reg, uint8_t *out)
{
    if (!s || !s->dev || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(s->dev, &reg, 1, out, 1, 1000);
}

esp_err_t hdc2080_write_reg(const hdc2080_t *s, uint8_t reg, uint8_t data)
{
    if (!s || !s->dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[2] = { reg, data };
    return i2c_master_transmit(s->dev, buf, sizeof(buf), 1000);
}

static esp_err_t hdc2080_read_u16_le(const hdc2080_t *s, uint8_t reg_low, uint16_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t lo, hi;
    esp_err_t err = hdc2080_read_reg(s, reg_low, &lo);
    if (err != ESP_OK) return err;
    err = hdc2080_read_reg(s, (uint8_t)(reg_low + 1), &hi);
    if (err != ESP_OK) return err;

    *out = ((uint16_t)hi << 8) | (uint16_t)lo;
    return ESP_OK;
}

esp_err_t hdc2080_trigger_measurement(const hdc2080_t *s)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    cfg |= 0x01;
    return hdc2080_write_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, cfg);
}

esp_err_t hdc2080_read_temp_c(const hdc2080_t *s, float *out_temp_c)
{
    if (!out_temp_c) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw;
    esp_err_t err = hdc2080_read_u16_le(s, HDC2080_REG_TEMP_LOW, &raw);
    if (err != ESP_OK) {
        return err;
    }

    *out_temp_c = ((float)raw * 165.0f / 65536.0f) - 40.5f;
    return ESP_OK;
}

esp_err_t hdc2080_read_humidity_rh(const hdc2080_t *s, float *out_humidity)
{
    if (!out_humidity) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw;
    esp_err_t err = hdc2080_read_u16_le(s, HDC2080_REG_HUMID_LOW, &raw);
    if (err != ESP_OK) {
        return err;
    }

    *out_humidity = ((float)raw / 65536.0f) * 100.0f;
    return ESP_OK;
}

esp_err_t hdc2080_read_temp_offset_adjust(const hdc2080_t *s, uint8_t *out)
{
    return hdc2080_read_reg(s, HDC2080_REG_TEMP_OFFSET_ADJUST, out);
}

esp_err_t hdc2080_set_temp_offset_adjust(const hdc2080_t *s, uint8_t value, uint8_t *out_new)
{
    esp_err_t err = hdc2080_write_reg(s, HDC2080_REG_TEMP_OFFSET_ADJUST, value);
    if (err != ESP_OK) return err;
    return out_new ? hdc2080_read_temp_offset_adjust(s, out_new) : ESP_OK;
}

esp_err_t hdc2080_read_humidity_offset_adjust(const hdc2080_t *s, uint8_t *out)
{
    return hdc2080_read_reg(s, HDC2080_REG_HUM_OFFSET_ADJUST, out);
}

esp_err_t hdc2080_set_humidity_offset_adjust(const hdc2080_t *s, uint8_t value, uint8_t *out_new)
{
    esp_err_t err = hdc2080_write_reg(s, HDC2080_REG_HUM_OFFSET_ADJUST, value);
    if (err != ESP_OK) return err;
    return out_new ? hdc2080_read_humidity_offset_adjust(s, out_new) : ESP_OK;
}

esp_err_t hdc2080_enable_threshold_interrupt(const hdc2080_t *s, HDC2080_InterruptMode mode)
{
    uint8_t v = 0;  // Start clean

    switch (mode) {
    case HDC2080_TEMP_ONLY:
        v = 0x60;  // TH_EN(0x40) + TL_EN(0x20)
        break;
    case HDC2080_HUMID_ONLY:
        v = 0x18;  // HH_EN(0x10) + HL_EN(0x08)
        break;
    case HDC2080_TEMP_AND_HUMID:
        v = 0x78;  // All four
        break;
    }

    return hdc2080_write_reg(s, 0x07, v);  // HDC2080_INT_CONFIG
}

// After config, kickstart AMM:
esp_err_t hdc2080_start_auto_measurement(const hdc2080_t *s)
{
    uint8_t meas;
    hdc2080_read_reg(s, 0x0F, &meas);
    meas |= 0x01;  // MEAS_TRIG_START
    return hdc2080_write_reg(s, 0x0F, meas);
}


esp_err_t hdc2080_disable_threshold_interrupt(const hdc2080_t *s)
{
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_INTERRUPT_CONFIG, &v);
    if (err != ESP_OK) return err;
    v &= ~((1 << 6) | (1 << 5) | (1 << 4) | (1 << 3));
    return hdc2080_write_reg(s, HDC2080_REG_INTERRUPT_CONFIG, v);
}

esp_err_t hdc2080_enable_heater(const hdc2080_t *s)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;
    cfg |= 0x08;
    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_disable_heater(const hdc2080_t *s)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;
    cfg &= (uint8_t)~0x08;
    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_set_low_temp(const hdc2080_t *s, float temp_c)
{
    temp_c = clampf(temp_c, -40.0f, 125.0f);
    uint8_t reg = (uint8_t)(256.0f * (temp_c + 40.0f) / 165.0f);
    return hdc2080_write_reg(s, HDC2080_REG_TEMP_THR_L, reg);
}

esp_err_t hdc2080_set_high_temp(const hdc2080_t *s, float temp_c)
{
    temp_c = clampf(temp_c, -40.0f, 125.0f);
    uint8_t reg = (uint8_t)(256.0f * (temp_c + 40.0f) / 165.0f);
    return hdc2080_write_reg(s, HDC2080_REG_TEMP_THR_H, reg);
}

esp_err_t hdc2080_set_high_humidity(const hdc2080_t *s, float humid_rh)
{
    humid_rh = clampf(humid_rh, 0.0f, 100.0f);
    uint8_t reg = (uint8_t)(256.0f * humid_rh / 100.0f);
    return hdc2080_write_reg(s, HDC2080_REG_HUMID_THR_H, reg);
}

esp_err_t hdc2080_set_low_humidity(const hdc2080_t *s, float humid_rh)
{
    humid_rh = clampf(humid_rh, 0.0f, 100.0f);
    uint8_t reg = (uint8_t)(256.0f * humid_rh / 100.0f);
    return hdc2080_write_reg(s, HDC2080_REG_HUMID_THR_L, reg);
}

esp_err_t hdc2080_read_low_humidity_threshold(const hdc2080_t *s, float *out_humid)
{
    if (!out_humid) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_HUMID_THR_L, &v);
    if (err != ESP_OK) return err;
    *out_humid = ((float)v) * 100.0f / 256.0f;
    return ESP_OK;
}

esp_err_t hdc2080_read_high_humidity_threshold(const hdc2080_t *s, float *out_humid)
{
    if (!out_humid) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_HUMID_THR_H, &v);
    if (err != ESP_OK) return err;
    *out_humid = ((float)v) * 100.0f / 256.0f;
    return ESP_OK;
}

esp_err_t hdc2080_read_low_temp_threshold(const hdc2080_t *s, float *out_temp)
{
    if (!out_temp) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_TEMP_THR_L, &v);
    if (err != ESP_OK) return err;
    *out_temp = ((float)v * 165.0f / 256.0f) - 40.0f;
    return ESP_OK;
}

esp_err_t hdc2080_read_high_temp_threshold(const hdc2080_t *s, float *out_temp)
{
    if (!out_temp) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_TEMP_THR_H, &v);
    if (err != ESP_OK) return err;
    *out_temp = ((float)v * 165.0f / 256.0f) - 40.0f;
    return ESP_OK;
}

esp_err_t hdc2080_set_temp_res(const hdc2080_t *s, int resolution)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    switch (resolution) {
        case HDC2080_FOURTEEN_BIT: cfg &= 0x3F; break;
        case HDC2080_ELEVEN_BIT:   cfg = (uint8_t)((cfg & 0x7F) | 0x40); break;
        case HDC2080_NINE_BIT:     cfg = (uint8_t)((cfg & 0xBF) | 0x80); break;
        default:                   cfg &= 0x3F; break;
    }

    return hdc2080_write_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, cfg);
}

esp_err_t hdc2080_set_humid_res(const hdc2080_t *s, int resolution)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    switch (resolution) {
        case HDC2080_FOURTEEN_BIT: cfg &= 0xCF; break;
        case HDC2080_ELEVEN_BIT:   cfg = (uint8_t)((cfg & 0xDF) | 0x10); break;
        case HDC2080_NINE_BIT:     cfg = (uint8_t)((cfg & 0xEF) | 0x20); break;
        default:                   cfg &= 0xCF; break;
    }

    return hdc2080_write_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, cfg);
}

esp_err_t hdc2080_set_measurement_mode(const hdc2080_t *s, int mode)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    switch (mode) {
        case HDC2080_TEMP_AND_HUMID: cfg &= 0xF9; break;
        case HDC2080_TEMP_ONLY:      cfg = (uint8_t)((cfg & 0xFC) | 0x02); break;
        case HDC2080_HUMID_ONLY:     cfg = (uint8_t)((cfg & 0xFD) | 0x04); break;
        default:                     cfg &= 0xF9; break;
    }

    return hdc2080_write_reg(s, HDC2080_REG_MEASUREMENT_CONFIG, cfg);
}

esp_err_t hdc2080_reset(const hdc2080_t *s)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    cfg |= 0x80;
    err = hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

esp_err_t hdc2080_enable_interrupt(const hdc2080_t *s)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    cfg |= 0x04;
    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_disable_interrupt(const hdc2080_t *s)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    cfg &= (uint8_t)~0x04;
    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_set_rate(const hdc2080_t *s, int rate)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    switch (rate) {
        case HDC2080_MANUAL:       cfg &= 0x8F; break;
        case HDC2080_TWO_MINS:     cfg = (uint8_t)((cfg & 0x9F) | 0x10); break;
        case HDC2080_ONE_MINS:     cfg = (uint8_t)((cfg & 0xAF) | 0x20); break;
        case HDC2080_TEN_SECONDS:  cfg = (uint8_t)((cfg & 0xBF) | 0x30); break;
        case HDC2080_FIVE_SECONDS: cfg = (uint8_t)((cfg & 0xCF) | 0x40); break;
        case HDC2080_ONE_HZ:       cfg = (uint8_t)((cfg & 0xDF) | 0x50); break;
        case HDC2080_TWO_HZ:       cfg = (uint8_t)((cfg & 0xEF) | 0x60); break;
        case HDC2080_FIVE_HZ:      cfg |= 0x70; break;
        default:                   cfg &= 0x8F; break;
    }

    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_set_interrupt_polarity(const hdc2080_t *s, int polarity)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    switch (polarity) {
        case HDC2080_ACTIVE_LOW:  cfg &= (uint8_t)~0x02; break;
        case HDC2080_ACTIVE_HIGH: cfg |= 0x02; break;
        default:                  cfg &= (uint8_t)~0x02; break;
    }

    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_set_interrupt_mode(const hdc2080_t *s, int mode)
{
    uint8_t cfg;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    switch (mode) {
        case HDC2080_LEVEL_MODE:      cfg &= (uint8_t)~0x01; break;
        case HDC2080_COMPARATOR_MODE: cfg |= 0x01; break;
        default:                      cfg &= (uint8_t)~0x01; break;
    }

    return hdc2080_write_reg(s, HDC2080_REG_CONFIG, cfg);
}

esp_err_t hdc2080_read_interrupt_status(const hdc2080_t *s, uint8_t *out)
{
    return hdc2080_read_reg(s, HDC2080_REG_INTERRUPT_DRDY, out);
}

esp_err_t hdc2080_clear_max_temp(const hdc2080_t *s)
{
    return hdc2080_write_reg(s, HDC2080_REG_TEMP_MAX, 0x00);
}

esp_err_t hdc2080_clear_max_humidity(const hdc2080_t *s)
{
    return hdc2080_write_reg(s, HDC2080_REG_HUMID_MAX, 0x00);
}

esp_err_t hdc2080_read_max_temp(const hdc2080_t *s, float *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_TEMP_MAX, &v);
    if (err != ESP_OK) return err;
    *out = ((float)v * 165.0f / 256.0f) - 40.0f;
    return ESP_OK;
}

esp_err_t hdc2080_read_max_humidity(const hdc2080_t *s, float *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t v;
    esp_err_t err = hdc2080_read_reg(s, HDC2080_REG_HUMID_MAX, &v);
    if (err != ESP_OK) return err;
    *out = ((float)v / 256.0f) * 100.0f;
    return ESP_OK;
}
