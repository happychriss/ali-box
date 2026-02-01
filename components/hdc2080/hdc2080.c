#include "hdc2080.h"

#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hdc2080";

// -----------------------------
// Register map (matches Arduino sketch)
// -----------------------------
static const uint8_t REG_TEMP_L        = 0x00;
// static const uint8_t REG_TEMP_H        = 0x01;
static const uint8_t REG_HUM_L         = 0x02;
// static const uint8_t REG_HUM_H         = 0x03;
static const uint8_t REG_INT_DRDY      = 0x04;
static const uint8_t REG_INT_ENABLE    = 0x07;
static const uint8_t REG_TEMP_THR_L    = 0x0A;
static const uint8_t REG_TEMP_THR_H    = 0x0B;
static const uint8_t REG_CONFIG        = 0x0E;
static const uint8_t REG_MEAS_CFG      = 0x0F;

// REG_CONFIG bit fields (0x0E)
static const uint8_t CFG_AMM_SHIFT     = 4;          // bits 6:4
static const uint8_t CFG_INT_EN        = (1u << 2);  // DRDY/INT_EN
static const uint8_t CFG_INT_POL       = (1u << 1);  // INT_POL
// static const uint8_t CFG_INT_MODE   = (1u << 0);  // INT_MODE (0 latched/level-sensitive, 1 comparator)

// REG_INT_ENABLE (0x07)
static const uint8_t EN_TH             = (1u << 6);
static const uint8_t EN_TL             = (1u << 5);

// REG_MEAS_CFG (0x0F)
static const uint8_t MEAS_TEMP_ONLY    = (0b01u << 1); // MEAS_CONF
static const uint8_t MEAS_RH_AND_TEMP  = (0b00u << 1);
static const uint8_t MEAS_TRIG         = (1u << 0);

struct hdc2080_t {
    i2c_master_dev_handle_t dev;

    gpio_num_t int_gpio;
    hdc2080_int_polarity_t int_polarity;

    uint32_t i2c_timeout_ms;
    uint32_t meas_wait_ms;
    uint32_t deassert_wait_ms;
};

static inline uint32_t ms_to_ticks(uint32_t ms) {
    return (ms == 0) ? 0 : pdMS_TO_TICKS(ms);
}

static esp_err_t hdc_write_u8(hdc2080_handle_t h, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(h->dev, buf, sizeof(buf), ms_to_ticks(h->i2c_timeout_ms));
}

static esp_err_t hdc_read_u8(hdc2080_handle_t h, uint8_t reg, uint8_t *out) {
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out is NULL");
    return i2c_master_transmit_receive(h->dev, &reg, 1, out, 1, ms_to_ticks(h->i2c_timeout_ms));
}

static esp_err_t hdc_read_u16_le(hdc2080_handle_t h, uint8_t reg_l, uint16_t *out) {
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out is NULL");

    // Preserve Arduino sketch behavior: read L then H via separate single-byte register reads.
    // (This avoids any subtle behavior differences from multi-byte burst reads.)
    uint8_t l = 0, hh = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, reg_l, &l), TAG, "read L failed");
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, (uint8_t)(reg_l + 1), &hh), TAG, "read H failed");

    *out = ((uint16_t)hh << 8) | l;
    return ESP_OK;
}

// -----------------------------
// Conversions (match Arduino sketch)
// -----------------------------
static float raw_to_temp_c(uint16_t raw) {
    return ((float)raw * 165.0f / 65536.0f) - 40.5f;
}

static float raw_to_rh_pct(uint16_t raw) {
    // HDC2080 RH% = raw/2^16 * 100
    return ((float)raw * 100.0f / 65536.0f);
}

static uint8_t temp_c_to_thr_code(float t_c) {
    // practical clamp (same as Arduino)
    if (t_c < -40.0f) t_c = -40.0f;
    if (t_c > 125.0f) t_c = 125.0f;

    float code = (t_c + 40.5f) * 256.0f / 165.0f;
    int ci = (int)lroundf(code);
    if (ci < 0) ci = 0;
    if (ci > 255) ci = 255;
    return (uint8_t)ci;
}

static esp_err_t read_and_clear_status_raw(hdc2080_handle_t h, uint8_t *out_raw) {
    // Reading REG_INT_DRDY clears bits on the device (latched mode relies on this).
    return hdc_read_u8(h, REG_INT_DRDY, out_raw);
}

// same warning behavior as Arduino sketch
static void wait_for_int_deassert_or_warn(hdc2080_handle_t h) {
    int64_t start_us = esp_timer_get_time();
    const int64_t timeout_us = (int64_t)h->deassert_wait_ms * 1000LL;

    while (hdc2080_int_is_asserted(h)) {
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            ESP_LOGW(TAG,
                     "INT did not deassert after clearing status. "
                     "Check INT_POL/ACTIVE_LOW, wiring, or whether board requires pull-up/pull-down.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static esp_err_t set_measurement_mode(hdc2080_handle_t h, bool temp_only) {
    uint8_t meas = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_MEAS_CFG, &meas), TAG, "read MEAS_CFG failed");

    // clear MEAS_CONF (bits2:1) and MEAS_TRIG (bit0)
    meas &= (uint8_t)~((0b11u << 1) | MEAS_TRIG);
    meas |= temp_only ? MEAS_TEMP_ONLY : MEAS_RH_AND_TEMP;

    return hdc_write_u8(h, REG_MEAS_CFG, meas);
}

static esp_err_t measure_temp_once_c(hdc2080_handle_t h, float *out_temp_c) {
    ESP_RETURN_ON_FALSE(out_temp_c, ESP_ERR_INVALID_ARG, TAG, "out_temp_c is NULL");

    uint8_t meas = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_MEAS_CFG, &meas), TAG, "read MEAS_CFG failed");

    // Trigger on demand by setting MEAS_TRIG bit (self-clearing)
    ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_MEAS_CFG, (uint8_t)(meas | MEAS_TRIG)), TAG, "write MEAS_TRIG failed");

    // At default resolution, 30ms is safe margin (Arduino uses 30ms)
    vTaskDelay(pdMS_TO_TICKS(h->meas_wait_ms));

    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u16_le(h, REG_TEMP_L, &raw), TAG, "read TEMP failed");
    *out_temp_c = raw_to_temp_c(raw);
    return ESP_OK;
}

static esp_err_t configure_for_temp_irq_detect(hdc2080_handle_t h, hdc2080_amm_rate_t amm_rate, bool active_low) {
    // 0) Set TEMP_THR_L to minimum (-40.5C, code 0x00) so TL won't randomly trigger.
    ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_TEMP_THR_L, 0x00), TAG, "write TEMP_THR_L failed");

    // 1) Measurement config: temp-only (Arduino sets temp-only on init)
    ESP_RETURN_ON_ERROR(set_measurement_mode(h, true), TAG, "set measurement mode failed");

    // 2) Enable only TH interrupt generator (Arduino enables only TH)
    ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_INT_ENABLE, EN_TH), TAG, "write INT_ENABLE failed");

    // 3) Configure AMM + interrupt pin. INT_MODE must be 0 (latched / level-sensitive).
    uint8_t cfg = 0;
    cfg |= (uint8_t)((uint8_t)amm_rate << CFG_AMM_SHIFT);
    cfg |= CFG_INT_EN;

    // polarity: Arduino sets CFG_INT_POL only if ACTIVE_LOW is false
    if (!active_low) cfg |= CFG_INT_POL;

    ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_CONFIG, cfg), TAG, "write CONFIG failed");

    // 4) Clear stale latched flags
    uint8_t st = 0;
    ESP_RETURN_ON_ERROR(read_and_clear_status_raw(h, &st), TAG, "read/clear status failed");
    (void)st;

    return ESP_OK;
}

static esp_err_t set_int_enable_only(hdc2080_handle_t h, hdc2080_irq_mode_t mode) {
    // Preserve the sketch’s "only TH enabled" idea when arming TH.
    // For TL mode, enable only TL.
    uint8_t mask = (mode == HDC2080_IRQ_TL) ? EN_TL : EN_TH;
    return hdc_write_u8(h, REG_INT_ENABLE, mask);
}

esp_err_t hdc2080_init(const hdc2080_config_t *cfg, hdc2080_handle_t *out_handle) {
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(cfg && out_handle, ESP_ERR_INVALID_ARG, TAG, "cfg/out_handle is NULL");
    ESP_RETURN_ON_FALSE(cfg->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "i2c_bus is NULL");

    hdc2080_handle_t h = (hdc2080_handle_t)calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "no mem");

    const uint8_t addr = (cfg->i2c_addr == 0) ? 0x40 : cfg->i2c_addr;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(cfg->i2c_bus, &dev_cfg, &h->dev);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    h->int_gpio = cfg->int_gpio;
    h->int_polarity = cfg->int_polarity;
    h->i2c_timeout_ms = (cfg->i2c_timeout_ms == 0) ? 1000 : cfg->i2c_timeout_ms;
    h->meas_wait_ms = (cfg->meas_wait_ms == 0) ? 30 : cfg->meas_wait_ms;
    h->deassert_wait_ms = (cfg->deassert_wait_ms == 0) ? 200 : cfg->deassert_wait_ms;

    // Configure INT GPIO similar to Arduino options
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << (uint64_t)h->int_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (cfg->int_pull == HDC2080_INT_PULLUP) io.pull_up_en = GPIO_PULLUP_ENABLE;
    if (cfg->int_pull == HDC2080_INT_PULLDOWN) io.pull_down_en = GPIO_PULLDOWN_ENABLE;

    ESP_GOTO_ON_ERROR(gpio_config(&io), fail, TAG, "gpio_config failed");

    // Configure sensor registers (preserve Arduino sketch sequence)
    const bool active_low = (cfg->int_polarity == HDC2080_INT_ACTIVE_LOW);
    ESP_GOTO_ON_ERROR(configure_for_temp_irq_detect(h, cfg->amm_rate, active_low), fail, TAG, "configure failed");

    *out_handle = h;
    return ESP_OK;

fail:
    hdc2080_deinit(h);
    return err;
}

esp_err_t hdc2080_measure_temp_humidity(hdc2080_handle_t h, bool temp_only, hdc2080_measurement_t *out) {
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "h/out NULL");
    memset(out, 0, sizeof(*out));

    // Ensure chip measurement mode matches the requested mode (no caching; write/read mirrors sketch style).
    ESP_RETURN_ON_ERROR(set_measurement_mode(h, temp_only), TAG, "set measurement mode failed");

    uint8_t meas = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_MEAS_CFG, &meas), TAG, "read MEAS_CFG failed");

    // Trigger
    ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_MEAS_CFG, (uint8_t)(meas | MEAS_TRIG)), TAG, "write MEAS_TRIG failed");

    vTaskDelay(pdMS_TO_TICKS(h->meas_wait_ms));

    // Temp
    uint16_t raw_t = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u16_le(h, REG_TEMP_L, &raw_t), TAG, "read TEMP failed");
    out->temp_c = raw_to_temp_c(raw_t);
    out->temp_valid = true;

    // Humidity optional
    if (!temp_only) {
        uint16_t raw_rh = 0;
        ESP_RETURN_ON_ERROR(hdc_read_u16_le(h, REG_HUM_L, &raw_rh), TAG, "read RH failed");
        out->rh_pct = raw_to_rh_pct(raw_rh);
        out->rh_valid = true;
    }

    return ESP_OK;
}

esp_err_t hdc2080_arm_irq(hdc2080_handle_t h, hdc2080_irq_mode_t mode, float baseline_temp_c, float delta_c) {
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h NULL");

    // Ensure only selected generator is enabled (preserves sketch’s intent to enable only one source)
    ESP_RETURN_ON_ERROR(set_int_enable_only(h, mode), TAG, "set INT_ENABLE failed");

    // Robust arming (preserve Arduino logic):
    //  1) Put threshold to extreme safe side so we are guaranteed on the non-triggered side
    //  2) Clear status
    //  3) short wait
    //  4) write real threshold
    if (mode == HDC2080_IRQ_TH) {
        ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_TEMP_THR_H, temp_c_to_thr_code(80.0f)), TAG, "write TEMP_THR_H high failed");
        uint8_t st = 0;
        ESP_RETURN_ON_ERROR(read_and_clear_status_raw(h, &st), TAG, "clear status failed");
        (void)st;
        vTaskDelay(pdMS_TO_TICKS(5));

        const float th_c = baseline_temp_c + delta_c;
        ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_TEMP_THR_H, temp_c_to_thr_code(th_c)), TAG, "write TEMP_THR_H failed");
    } else {
        // Symmetric for TL support (sketch default disables TL by setting it very low).
        ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_TEMP_THR_L, 0x00), TAG, "write TEMP_THR_L low failed");
        uint8_t st = 0;
        ESP_RETURN_ON_ERROR(read_and_clear_status_raw(h, &st), TAG, "clear status failed");
        (void)st;
        vTaskDelay(pdMS_TO_TICKS(5));

        const float tl_c = baseline_temp_c - delta_c;
        ESP_RETURN_ON_ERROR(hdc_write_u8(h, REG_TEMP_THR_L, temp_c_to_thr_code(tl_c)), TAG, "write TEMP_THR_L failed");
    }

    return ESP_OK;
}

esp_err_t hdc2080_read_and_clear_status(hdc2080_handle_t h, hdc2080_status_t *out) {
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "h/out NULL");

    uint8_t raw = 0;
    ESP_RETURN_ON_ERROR(read_and_clear_status_raw(h, &raw), TAG, "read status failed");

    out->raw = raw;
    out->drdy = (raw & (1u << 7)) != 0;
    out->th   = (raw & (1u << 6)) != 0;
    out->tl   = (raw & (1u << 5)) != 0;
    out->hh   = (raw & (1u << 4)) != 0;
    out->hl   = (raw & (1u << 3)) != 0;

    return ESP_OK;
}

bool hdc2080_int_is_asserted(hdc2080_handle_t h) {
    if (!h) return false;
    const int level = gpio_get_level(h->int_gpio);
    return (h->int_polarity == HDC2080_INT_ACTIVE_LOW) ? (level == 0) : (level == 1);
}

esp_err_t hdc2080_dump_config(hdc2080_handle_t h) {
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h NULL");

    uint8_t cfg = 0, ien = 0, th = 0, tl = 0, meas = 0;
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_CONFIG, &cfg), TAG, "read CONFIG failed");
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_INT_ENABLE, &ien), TAG, "read INT_ENABLE failed");
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_TEMP_THR_H, &th), TAG, "read THR_H failed");
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_TEMP_THR_L, &tl), TAG, "read THR_L failed");
    ESP_RETURN_ON_ERROR(hdc_read_u8(h, REG_MEAS_CFG, &meas), TAG, "read MEAS_CFG failed");

    const uint8_t amm = (cfg >> CFG_AMM_SHIFT) & 0x7;
    const uint8_t int_en = (cfg >> 2) & 0x1;
    const uint8_t pol = (cfg >> 1) & 0x1;
    const uint8_t mode = (cfg >> 0) & 0x1;

    ESP_LOGI(TAG, "REG_CONFIG (0x0E)=0x%02X AMM=%u INT_EN=%u POL=%u MODE=%u", cfg, amm, int_en, pol, mode);
    ESP_LOGI(TAG, "REG_INT_ENABLE (0x07)=0x%02X (TH=%u TL=%u)", ien, (ien >> 6) & 0x1, (ien >> 5) & 0x1);
    ESP_LOGI(TAG, "REG_TEMP_THR_H (0x0B)=0x%02X  REG_TEMP_THR_L (0x0A)=0x%02X", th, tl);
    ESP_LOGI(TAG, "REG_MEAS_CFG (0x0F)=0x%02X", meas);

    return ESP_OK;
}

void hdc2080_deinit(hdc2080_handle_t h) {
    if (!h) return;
    if (h->dev) {
        // i2c_master_bus_rm_device exists in ESP-IDF v5.x
        i2c_master_bus_rm_device(h->dev);
    }
    free(h);
}
