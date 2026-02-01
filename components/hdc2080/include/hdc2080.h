#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * HDC2080 driver (ESP-IDF v5.5 style).
 *
 * Goal: preserve the exact register-level logic/ordering from the existing Arduino sketch:
 *  - INT_STATUS (0x04) read clears bits
 *  - CONFIG (0x0E) INT_MODE=0 (latched/level-sensitive)
 *  - Robust TH arming: set TH very high, clear status, short wait, then set real threshold
 *  - Manual MEAS_TRIG + wait before reads
 */

typedef struct hdc2080_t* hdc2080_handle_t;

typedef enum {
    HDC2080_AMM_DISABLED = 0b000,
    HDC2080_AMM_2MIN     = 0b001,
    HDC2080_AMM_1MIN     = 0b010,
    HDC2080_AMM_10SEC    = 0b011,
    HDC2080_AMM_5SEC     = 0b100,
    HDC2080_AMM_1HZ      = 0b101,
    HDC2080_AMM_2HZ      = 0b110,
    HDC2080_AMM_5HZ      = 0b111,
} hdc2080_amm_rate_t;

typedef enum {
    HDC2080_INT_ACTIVE_LOW = 0,
    HDC2080_INT_ACTIVE_HIGH = 1,
} hdc2080_int_polarity_t;

typedef enum {
    HDC2080_INT_PULL_NONE = 0,
    HDC2080_INT_PULLUP,
    HDC2080_INT_PULLDOWN,
} hdc2080_int_pull_t;

typedef enum {
    HDC2080_IRQ_TH = 0,  // temperature high
    HDC2080_IRQ_TL = 1,  // temperature low
} hdc2080_irq_mode_t;

typedef struct {
    float temp_c;
    float rh_pct;
    bool temp_valid;
    bool rh_valid;
} hdc2080_measurement_t;

typedef struct {
    uint8_t raw;
    bool drdy;
    bool th;
    bool tl;
    bool hh;
    bool hl;
} hdc2080_status_t;

typedef struct {
    // I2C
    i2c_master_bus_handle_t i2c_bus;
    uint8_t i2c_addr;               // default 0x40
    uint32_t i2c_timeout_ms;        // applied to each transaction

    // Pins
    gpio_num_t int_gpio;
    hdc2080_int_polarity_t int_polarity;
    hdc2080_int_pull_t int_pull;

    // Behavior knobs (defaults keep Arduino timing/logic)
    hdc2080_amm_rate_t amm_rate;
    uint32_t meas_wait_ms;          // default 30
    uint32_t deassert_wait_ms;      // default 200

    // Poll/debounce (if you use helper polling in example)
    uint32_t debounce_ms;           // default 30
} hdc2080_config_t;

/**
 * Initialize the driver and configure the sensor for latched TH/TL interrupt behavior.
 *
 * Important: this calls the same configuration sequence as the Arduino sketch:
 *  - TEMP-only measurement config
 *  - enables only TH generator by default (see NOTE below)
 *  - configures CONFIG register (AMM, INT_EN, INT_POL, INT_MODE=0)
 *  - clears stale latched flags by reading INT_STATUS (0x04)
 *
 * NOTE about "only TH generator": the original sketch deliberately enables only TH to avoid
 * priority masking/ambiguity. We preserve that default behavior.
 */
esp_err_t hdc2080_init(const hdc2080_config_t *cfg, hdc2080_handle_t *out_handle);

/**
 * Measure temperature and optionally humidity.
 *
 * This preserves the sketch’s behavior:
 *  - set MEAS_TRIG
 *  - wait cfg->meas_wait_ms
 *  - read TEMP (and optionally RH)
 */
esp_err_t hdc2080_measure_temp_humidity(hdc2080_handle_t h, bool temp_only, hdc2080_measurement_t *out);

/**
 * Arm the temperature IRQ generator.
 *
 * mode:
 *  - HDC2080_IRQ_TH: arms Temperature High threshold relative to baseline_temp_c (+delta)
 *  - HDC2080_IRQ_TL: arms Temperature Low threshold relative to baseline_temp_c (-delta)
 *
 * This keeps the "robust arm" step: write a safe extreme threshold first, clear status,
 * short wait, then write the real threshold.
 */
esp_err_t hdc2080_arm_irq(hdc2080_handle_t h, hdc2080_irq_mode_t mode, float baseline_temp_c, float delta_c);

/** Read INT_STATUS (0x04) and clear latched bits (chip behavior). */
esp_err_t hdc2080_read_and_clear_status(hdc2080_handle_t h, hdc2080_status_t *out);

/** Read the INT GPIO level and interpret asserted state based on configured polarity. */
bool hdc2080_int_is_asserted(hdc2080_handle_t h);

/** Debug: read CONFIG/INT_ENABLE/THR/MEAS_CFG and format to log. */
esp_err_t hdc2080_dump_config(hdc2080_handle_t h);

/** Free resources. */
void hdc2080_deinit(hdc2080_handle_t h);

#ifdef __cplusplus
}
#endif
