#include "imu_helper.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "IMU";

esp_err_t imu_helper_init(const imu_helper_config_t *cfg, imu_helper_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "cfg/out NULL");
    ESP_RETURN_ON_FALSE(cfg->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "i2c_bus NULL");

    out->gpio_int1 = cfg->gpio_int1;
    out->wom_threshold_mg = cfg->wom_threshold_mg ? cfg->wom_threshold_mg : 20;
    out->wom_blanking_samples = cfg->wom_blanking_samples ? cfg->wom_blanking_samples : 3;
    out->initialized = false;

    /* Configure INT1 as input with pull-up (EXT1 ANY_LOW inactive = HIGH) */
    const gpio_config_t imu_int_cfg = {
        .pin_bit_mask = 1ULL << (int)cfg->gpio_int1,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&imu_int_cfg));

    /* Try both common I2C addresses */
    esp_err_t ret = qmi8658_init(&out->dev, cfg->i2c_bus, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK) {
        ret = qmi8658_init(&out->dev, cfg->i2c_bus, QMI8658_ADDRESS_LOW);
    }

    if (ret == ESP_OK) {
        out->initialized = true;
        ESP_LOGI(TAG, "QMI8658 initialized, INT1=GPIO%d", (int)cfg->gpio_int1);
    } else {
        ESP_LOGW(TAG, "QMI8658 not detected: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t imu_helper_arm_wom(imu_helper_handle_t *h)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    /* Configure WoM: INT1 starts HIGH, toggles LOW on motion (for EXT1 ANY_LOW) */
    const qmi8658_wom_config_t wom_cfg = {
        .int_pin = QMI8658_INT_PIN1,
        .initial_level = QMI8658_WOM_INITIAL_LEVEL_HIGH,
        .threshold_mg = h->wom_threshold_mg,
        .blanking_samples = h->wom_blanking_samples,
    };

    esp_err_t ret = qmi8658_enable_wake_on_motion_cfg(&h->dev, &wom_cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WoM armed: INT1 HIGH->LOW on motion, thr=%u, blank=%u",
                 (unsigned)wom_cfg.threshold_mg, (unsigned)wom_cfg.blanking_samples);
    } else {
        ESP_LOGW(TAG, "WoM arm failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t imu_helper_clear_wom_status(imu_helper_handle_t *h)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;
    return qmi8658_wom_clear_status(&h->dev);
}

int imu_helper_get_int1_level(imu_helper_handle_t *h)
{
    if (!h) return -1;
    return gpio_get_level(h->gpio_int1);
}

