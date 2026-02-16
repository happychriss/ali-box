#include "hdc2080_helper.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "HDC2080";

esp_err_t hdc2080_helper_init(const hdc2080_helper_config_t *cfg, hdc2080_helper_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "cfg/out NULL");
    ESP_RETURN_ON_FALSE(cfg->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "i2c_bus NULL");

    out->gpio_irq = cfg->gpio_irq;
    out->initialized = false;

    hdc2080_config_t hdc_cfg = {
        .i2c_bus = cfg->i2c_bus,
        .i2c_addr = cfg->i2c_addr ? cfg->i2c_addr : 0x40,
        .i2c_timeout_ms = 1000,
        .int_gpio = cfg->gpio_irq,
        .int_polarity = HDC2080_INT_ACTIVE_LOW,
        .int_pull = HDC2080_INT_PULLUP,
        .amm_rate = HDC2080_AMM_1HZ,
        .meas_wait_ms = 30,
        .deassert_wait_ms = 200,
        .debounce_ms = 30,
    };

    esp_err_t ret = hdc2080_init(&hdc_cfg, &out->handle);
    if (ret == ESP_OK) {
        out->initialized = true;
        ESP_LOGI(TAG, "Initialized, IRQ=GPIO%d", (int)cfg->gpio_irq);
    } else {
        ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t hdc2080_helper_read_temp(hdc2080_helper_handle_t *h, float *temp_c, float *humidity_rh)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    hdc2080_measurement_t m = {0};
    bool temp_only = (humidity_rh == NULL);
    esp_err_t ret = hdc2080_measure_temp_humidity(h->handle, temp_only, &m);

    if (ret == ESP_OK) {
        if (temp_c) *temp_c = m.temp_c;
        if (humidity_rh) *humidity_rh = m.rh_pct;
    }

    return ret;
}

esp_err_t hdc2080_helper_arm_temp_irq(hdc2080_helper_handle_t *h, float baseline_c, float delta_c)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    /* Arm for rising temperature threshold (TH) */
    esp_err_t ret = hdc2080_arm_irq(h->handle, HDC2080_IRQ_TH, baseline_c, delta_c);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Temp IRQ armed: baseline=%.2fC, threshold=%.2fC",
                 (double)baseline_c, (double)(baseline_c + delta_c));
    }

    /* Clear status after arming */
    hdc2080_status_t st = {0};
    (void)hdc2080_read_and_clear_status(h->handle, &st);

    return ret;
}

esp_err_t hdc2080_helper_clear_status(hdc2080_helper_handle_t *h)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    hdc2080_status_t st = {0};
    return hdc2080_read_and_clear_status(h->handle, &st);
}

bool hdc2080_helper_irq_asserted(hdc2080_helper_handle_t *h)
{
    if (!h || !h->initialized) return false;
    return hdc2080_int_is_asserted(h->handle);
}

int hdc2080_helper_get_irq_level(hdc2080_helper_handle_t *h)
{
    if (!h) return -1;
    return gpio_get_level(h->gpio_irq);
}

