#include "touch_helper.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_sleep.h"
#include "freertos/task.h"

static const char *TAG = "TOUCH";

static SemaphoreHandle_t s_irq_sem = NULL;

static void touch_isr_callback(esp_lcd_touch_handle_t tp)
{
    (void)tp;
    if (s_irq_sem) {
        BaseType_t high_task_wake = pdFALSE;
        xSemaphoreGiveFromISR(s_irq_sem, &high_task_wake);
        if (high_task_wake) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t touch_helper_init(const touch_helper_config_t *cfg, touch_helper_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "cfg/out NULL");
    ESP_RETURN_ON_FALSE(cfg->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "i2c_bus NULL");

    ESP_LOGI(TAG, "CST816S init");

    /* Create IRQ semaphore */
    out->irq_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(out->irq_sem, ESP_ERR_NO_MEM, TAG, "semaphore create failed");
    s_irq_sem = out->irq_sem;

    out->gpio_int = cfg->gpio_int;
    out->gpio_rst = cfg->gpio_rst;

    /* Configure INT pin */
    const gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << (int)cfg->gpio_int,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_cfg));

    /* Perform hardware reset sequence if not already done */
    if (!cfg->skip_hw_reset) {
        ESP_LOGI(TAG, "Hardware reset (GPIO%d)", cfg->gpio_rst);

        /* Configure RST pin */
        const gpio_config_t rst_cfg = {
            .pin_bit_mask = 1ULL << (int)cfg->gpio_rst,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&rst_cfg));

        /* Release GPIO hold on pins (in case waking from deep sleep) */
        gpio_hold_dis(cfg->gpio_rst);
        gpio_hold_dis(cfg->gpio_int);

        gpio_set_level(cfg->gpio_rst, cfg->rst_active_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(cfg->gpio_rst, !cfg->rst_active_level);
        vTaskDelay(pdMS_TO_TICKS(120));

        /* Wait for touch controller to boot */
        vTaskDelay(pdMS_TO_TICKS(150));
    } else {
        ESP_LOGI(TAG, "Hardware reset skipped (already done)");
    }

    /* Touch panel IO config */
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = cfg->i2c_clk_hz;

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(cfg->i2c_bus, &tp_io_config, &tp_io_handle));

    /* Touch config (RAW panel-native coords, LVGL rotates automatically) */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = cfg->x_max,
        .y_max = cfg->y_max,
        .rst_gpio_num = cfg->gpio_rst,
        .int_gpio_num = cfg->gpio_int,
        .interrupt_callback = touch_isr_callback,
        .levels = {.reset = cfg->rst_active_level, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &out->handle));
    ESP_LOGI(TAG, "CST816S initialized");

    return ESP_OK;
}

bool touch_helper_read(touch_helper_handle_t *h, int32_t *x, int32_t *y)
{
    if (!h || !h->handle || !h->irq_sem) return false;

    /* Only read I2C after IRQ */
    if (xSemaphoreTake(h->irq_sem, 0) != pdTRUE) return false;

    esp_lcd_touch_read_data(h->handle);

    esp_lcd_touch_point_data_t points[1];
    uint8_t point_cnt = 0;

    if (esp_lcd_touch_get_data(h->handle, points, &point_cnt, 1) == ESP_OK && point_cnt > 0) {
        if (x) *x = points[0].x;
        if (y) *y = points[0].y;
        return true;
    }
    return false;
}

esp_err_t touch_helper_arm_wakeup(touch_helper_handle_t *h)
{
    if (!h) return ESP_ERR_INVALID_ARG;

    /* Configure as input with pull-up for EXT1 ANY_LOW wake */
    const gpio_config_t wake_cfg = {
        .pin_bit_mask = 1ULL << (int)h->gpio_int,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&wake_cfg));

    ESP_LOGI(TAG, "Wakeup armed on GPIO%d", (int)h->gpio_int);
    return ESP_OK;
}

esp_err_t touch_helper_enter_sleep(touch_helper_handle_t *h)
{
    if (!h || !h->handle) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = esp_lcd_touch_cst816s_enter_sleep(h->handle);
    ESP_LOGI(TAG, "Enter sleep -> %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(100));
    return ret;
}

void touch_helper_disable_irq(touch_helper_handle_t *h)
{
    if (!h) return;
    (void)gpio_intr_disable(h->gpio_int);
    if (h->irq_sem) {
        (void)xSemaphoreTake(h->irq_sem, 0);
    }
}

