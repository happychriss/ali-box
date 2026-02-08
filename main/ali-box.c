/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Cleaned for:
 *  - LCD: 1.9", 170x320, ST7789V2, SPI
 *  - Touch: CST816S, I2C
 *  - LVGL 9.4: automatic input rotation (no manual transform needed)
 *  - Touch dot follows finger in all rotations
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include "qmi8658.h"

#include "hdc2080.h"

#include <stdio.h>  /* snprintf */
#include <inttypes.h>

#include "freertos/semphr.h"  /* SemaphoreHandle_t */

/* Image asset */
LV_IMG_DECLARE(esp_logo);

/* =========================================================
 * CST816S TOUCH (IRQ-driven read, per component README)
 * ========================================================= */
#include "esp_lcd_touch_cst816s.h"

static const char* TAG = "MAIN";

/* === LCD native resolution (portrait) === */
#define LCD_H_RES   (170)
#define LCD_V_RES   (320)

/* === PANEL GAP (RAM window offset) ===
 * This 170x320 ST7789 panel needs a RAM window offset ("gap").
 * Apply a different gap depending on orientation.
 */
#define LCD_PORTRAIT_X_OFFSET  (35)
#define LCD_PORTRAIT_Y_OFFSET  (0)
#define LCD_LANDSCAPE_X_OFFSET (0)
#define LCD_LANDSCAPE_Y_OFFSET (35)

/* === LCD SPI settings === */
#define LCD_SPI_NUM          (SPI3_HOST)
#define LCD_PIXEL_CLK_HZ     (26 * 1000 * 1000)
#define LCD_CMD_BITS         (8)
#define LCD_PARAM_BITS       (8)
#define LCD_BITS_PER_PIXEL   (16)
#define LCD_DRAW_BUFF_DOUBLE (1)
#define LCD_DRAW_BUFF_HEIGHT (40)
#define LCD_BL_ON_LEVEL      (0)

/* === LCD pins === */
#define LCD_GPIO_RST     (GPIO_NUM_9)
#define LCD_GPIO_SCLK    (GPIO_NUM_10)
#define LCD_GPIO_DC      (GPIO_NUM_11)
#define LCD_GPIO_CS      (GPIO_NUM_12)
#define LCD_GPIO_MOSI    (GPIO_NUM_13)
#define LCD_GPIO_BL      (GPIO_NUM_14)

/* === Touch I2C settings === */
#define TOUCH_I2C_NUM        (0)
#define TOUCH_I2C_CLK_HZ     (400000)
#define TOUCH_I2C_SDA        (GPIO_NUM_47)
#define TOUCH_I2C_SCL        (GPIO_NUM_48)
#define TOUCH_GPIO_INT       (GPIO_NUM_21)
#define TOUCH_RST_PIN        (GPIO_NUM_17)

// CST816S reset is active-low for this board
#define TOUCH_RST_ACTIVE_LEVEL (0)

/* === HDC2080 settings === */
#define HDC2080_ADDR 0x40
#define HDC2080_IRQ  (GPIO_NUM_15)
#define TEMP_DELTA_WAKE_C    (2.f)
#define DISPLAY_ON_MS        (3000)

/* === IMU (QMI8658) interrupt pins ===
 * IMU INT1 = GPIO8, INT2 = GPIO7 (per user wiring).
 * Assumption for EXT1: interrupt is active-low (common open-drain).
 * If your wiring/config is active-high, switch EXT1 mode to ANY_HIGH.
 */
#define IMU_INT1_GPIO        (GPIO_NUM_8)


/* Wake-on-motion sensitivity (library uses register 0x0B). Tune as needed. */
#define QMI8658_WOM_THRESHOLD_DEFAULT (20)

/* Wake sources
 * Touch IRQ is on GPIO21 (TOUCH_GPIO_INT).
 * HDC2080 IRQ is on GPIO15 (HDC2080_IRQ).
 * Both can be configured as GPIO deep-sleep wake sources.
 */

typedef enum
{
    APP_WAKE_UNKNOWN = 0,
    APP_WAKE_TOUCH,
    APP_WAKE_TEMP_THRESHOLD,
    APP_WAKE_IMU,
} app_wake_cause_t;

static lv_obj_t* wakeup_label = NULL;

static void app_wakeup_cause_ui_init(void);
static void app_wakeup_cause_ui_set(app_wake_cause_t cause);
static esp_err_t app_i2c_init_shared(void);
static void app_note_activity(app_wake_cause_t cause, TickType_t* last_activity_ticks);
static void app_configure_gpio_wakeup_sources(void);
static void app_prepare_for_deep_sleep(void);
static void app_sleep_debug_dump_pins(void);

/* Touch visualizer (green dot) */
static lv_obj_t* touch_dot = NULL;
static lv_timer_t* touch_dot_timer = NULL;

/* Temp/humidity demo UI (kept separate from touch demo) */
static lv_obj_t* temp_label = NULL;
static lv_obj_t* hum_label = NULL;

static void app_touch_dot_update(int32_t x, int32_t y, bool pressed);
static void app_touch_dot_timer_cb(lv_timer_t* t);

/* Temperature display demo helpers */
static void app_temp_display_ui_init(void);
static void app_temp_display_update(float temp_c, float humidity_rh, bool sensor_ok);
static void app_display_turn_on(void);
static void app_display_turn_off(void);

/* Handles */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;
static lv_display_t* lvgl_disp = NULL;
static lv_indev_t* lvgl_touch_indev = NULL;

/* Shared I2C bus (created once; reused by touch + sensors) */
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

static qmi8658_dev_t s_qmi8658 = {0};

/* IRQ semaphore for CST816S (per component README) */
static SemaphoreHandle_t touch_irq_sem = NULL;

/* Forward declarations */
static esp_err_t app_lcd_init(void);
static esp_err_t app_touch_init(void);
static esp_err_t app_lvgl_init(void);
static esp_err_t app_hdc2080_init(void);
static void app_main_display(void);
static void app_runtime_diag_task(void* arg);
static esp_err_t app_lcd_apply_gap_for_rotation(lv_display_rotation_t rot);
static void app_apply_rotation(lv_display_rotation_t rot);
static void app_power_save_shutdown_and_sleep_ext1(bool keep_touch_wake, bool keep_hdc_wake);


/* =========================================================
 * TOUCH ISR CALLBACK (CST816S README)
 * ========================================================= */
static void touch_isr_callback(esp_lcd_touch_handle_t tp)
{
    (void)tp;
    if (touch_irq_sem)
    {
        BaseType_t high_task_wake = pdFALSE;
        xSemaphoreGiveFromISR(touch_irq_sem, &high_task_wake);
        if (high_task_wake)
        {
            portYIELD_FROM_ISR();
        }
    }
}

/* =========================================================
 * LVGL TOUCH CALLBACK (raw panel-native coords)
 * ========================================================= */
/* LVGL 9.4: report RAW panel coordinates (rotation 0).
 * LVGL transforms automatically based on lv_display_set_rotation().
 */
static void app_lvgl_touch_read_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
    (void)indev;
    data->state = LV_INDEV_STATE_RELEASED;

    if (!touch_handle || !touch_irq_sem)
    {
        return;
    }

    /* Only read I2C after IRQ (CST816S responds briefly post-touch) */
    if (xSemaphoreTake(touch_irq_sem, 0) == pdTRUE)
    {
        esp_lcd_touch_read_data(touch_handle);

        esp_lcd_touch_point_data_t points[1];
        uint8_t point_cnt = 0;

        if (esp_lcd_touch_get_data(touch_handle, points, &point_cnt, 1) == ESP_OK &&
            point_cnt > 0)
        {
            /*
             * IMPORTANT:
             * Report RAW panel-native coordinates here (rotation 0).
             * LVGL will rotate/map them for widgets based on lv_display_set_rotation().
             */
            data->point.x = points[0].x;
            data->point.y = points[0].y;
            data->state = LV_INDEV_STATE_PRESSED;
        }
    }
}

/* =========================================================
 * LCD INIT
 * ========================================================= */
static esp_err_t app_lcd_init(void)
{

    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(LCD_GPIO_BL);

    ESP_LOGI(TAG, "[LCD] Init!! %dx%d ST7789", LCD_H_RES, LCD_V_RES);

    /* Backlight GPIO */
    gpio_config_t bk_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_GPIO_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_config));
    gpio_set_level(LCD_GPIO_BL, !LCD_BL_ON_LEVEL);

    /* SPI bus */
    const spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_GPIO_SCLK,
        .mosi_io_num = LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    /* Panel IO */
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_GPIO_DC,
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io));

    /* ST7789 panel */
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));

    /* Backlight ON */
    gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL);

    return ESP_OK;
}

/* =========================================================
 * TOUCH INIT
 * ========================================================= */
static esp_err_t app_touch_init(void)
{
    ESP_LOGI(TAG, "[TOUCH] CST816S init");

    // Configure RST pin FIRST (before I2C init) to wake touch controller from deep sleep
    const gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << (int)TOUCH_RST_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));

    const gpio_config_t touch_int_cfg = {
        .pin_bit_mask = 1ULL << (int)TOUCH_GPIO_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&touch_int_cfg));

    // Reset touch controller BEFORE I2C initialization (required when waking from deep sleep)
    gpio_set_level(TOUCH_RST_PIN, !TOUCH_RST_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(TOUCH_RST_PIN, TOUCH_RST_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(TOUCH_RST_PIN, !TOUCH_RST_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(150));  // Allow touch controller to wake up before I2C init

    ESP_RETURN_ON_ERROR(app_i2c_init_shared(), TAG, "[I2C] init failed");
    vTaskDelay(pdMS_TO_TICKS(150));  // Allow touch controller to wake up before I2C init

    // If we're reinitializing (e.g., after a failed attempt), clean up the old handle.
    if (touch_handle) {
        esp_lcd_touch_del(touch_handle);
        touch_handle = NULL;
    }

    /* Touch config (RAW panel-native, LVGL rotates automatically) */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_RST_PIN,
        .int_gpio_num = TOUCH_GPIO_INT,
        .interrupt_callback = touch_isr_callback,
        .levels = {.reset = TOUCH_RST_ACTIVE_LEVEL, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0}, /* No manual rotation */
    };

    // Create panel IO once per init call; reuse across attempts in this call.
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = TOUCH_I2C_CLK_HZ;

    esp_err_t err = esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config, &tp_io_handle);
    ESP_RETURN_ON_ERROR(err, TAG, "[TOUCH] create I2C panel IO failed");

    // Retry init a couple of times (reset already done before I2C init)
    for (int attempt = 1; attempt <= 3; attempt++) {
        // Additional delay for touch controller boot (reset already done above)
        if (attempt > 1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        err = esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "[TOUCH] CST816S init OK (attempt %d)", attempt);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "[TOUCH] CST816S init failed (attempt %d): %s", attempt, esp_err_to_name(err));

        // Best-effort I2C scan for diagnostics on first failure only.
        if (attempt == 1) {
            uint8_t found = 0;
            for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
                if (i2c_master_probe(i2c_bus_handle, addr, 20) == ESP_OK) {
                    ESP_LOGW(TAG, "[TOUCH] I2C device ACK at 0x%02X", addr);
                    found++;
                }
            }
            if (!found) {
                ESP_LOGW(TAG, "[TOUCH] I2C scan: no devices ACKed (check power, SDA/SCL, pull-ups)");
            }
        }

        // Clean up any partially-created handle before retrying.
        if (touch_handle) {
            esp_lcd_touch_del(touch_handle);
            touch_handle = NULL;
        }
    }

    // All attempts failed; free IO handle so we don't leak devices across reboots/retries.
    if (tp_io_handle) {
        (void)esp_lcd_panel_io_del(tp_io_handle);
        tp_io_handle = NULL;
    }

    ESP_LOGE(TAG, "[TOUCH] CST816S init failed after retries: %s", esp_err_to_name(err));
    return err;
}

/* =========================================================
 * LVGL INIT
 * ========================================================= */
static esp_err_t app_lvgl_init(void)

{
    ESP_LOGI(TAG, "[LVGL] Init v%d.%d.%d", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    // return with error if not version 9.4.x
#if LVGL_VERSION_MAJOR != 9 || LVGL_VERSION_MINOR != 4
    ESP_LOGE(TAG, "LVGL version 9.4.x required");
    return ESP_ERR_INVALID_VERSION;
#endif

    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 8192,
        .timer_period_ms = 5
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));
    ESP_LOGI(TAG, "[LVGL] Port initialized");

    /* Display */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = LCD_DRAW_BUFF_DOUBLE,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags.buff_dma = true,
        .flags.swap_bytes = true,
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    ESP_LOGI(TAG, "[LVGL] Display added");

    /* Default: 90° landscape (your preference) */
    app_apply_rotation(LV_DISPLAY_ROTATION_90);

    /* Touch input device */
    lvgl_touch_indev = lv_indev_create();
    lv_indev_set_type(lvgl_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(lvgl_touch_indev, lvgl_disp);
    lv_indev_set_read_cb(lvgl_touch_indev, app_lvgl_touch_read_cb);
    ESP_LOGI(TAG, "[LVGL] Touch input device added");

    /*
     * Touch-dot updater:
     * The indev read callback receives/stores RAW coordinates.
     * To draw in the rotated UI coordinate system, query LVGL for the processed
     * pointer point and draw from that.
     */
    touch_dot_timer = lv_timer_create(app_touch_dot_timer_cb, 16, NULL);

    ESP_LOGI(TAG, "[LVGL] Init done");
    return ESP_OK;
}

/* =========================================================
 * Temperature-display-only UI helpers
 * (kept separate from TOUCH_DISPLAY_DEMO)
 * ========================================================= */
static void app_display_turn_on(void)
{
    if (lcd_panel)
    {
        (void)esp_lcd_panel_disp_on_off(lcd_panel, true);
    }
    gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL);
}

static void app_display_turn_off(void)
{
    gpio_set_level(LCD_GPIO_BL, !LCD_BL_ON_LEVEL);
    if (lcd_panel)
    {
        (void)esp_lcd_panel_disp_on_off(lcd_panel, false);
    }
}

static void app_temp_display_ui_init(void)
{
    lvgl_port_lock(0);

    lv_obj_t* scr = lv_scr_act();
    lv_obj_clean(scr);

    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, "HDC2080");
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    /* Wakeup cause line (small) */
    wakeup_label = lv_label_create(scr);
    lv_label_set_text(wakeup_label, "Wake: --");
    lv_obj_set_style_text_align(wakeup_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(wakeup_label, LV_ALIGN_TOP_MID, 0, 36);

    temp_label = lv_label_create(scr);
    lv_label_set_text(temp_label, "Temp: --.-- C");
    lv_obj_align(temp_label, LV_ALIGN_CENTER, 0, -10);

    hum_label = lv_label_create(scr);
    lv_label_set_text(hum_label, "Humidity: --.-- %RH");
    lv_obj_align(hum_label, LV_ALIGN_CENTER, 0, 20);

    lvgl_port_unlock();
}

static void app_temp_display_update(float temp_c, float humidity_rh, bool sensor_ok)
{
    if (!temp_label || !hum_label)
    {
        return;
    }

    char buf1[48];
    char buf2[48];

    if (!sensor_ok)
    {
        snprintf(buf1, sizeof(buf1), "Temp: (sensor missing)");
        snprintf(buf2, sizeof(buf2), "Humidity: (sensor missing)");
    }
    else
    {
        snprintf(buf1, sizeof(buf1), "Temp: %.2f C", (double)temp_c);
        snprintf(buf2, sizeof(buf2), "Humidity: %.2f %%RH", (double)humidity_rh);
    }

    lvgl_port_lock(0);
    lv_label_set_text(temp_label, buf1);
    lv_label_set_text(hum_label, buf2);
    lvgl_port_unlock();
}

/* =========================================================
 * Wakeup cause helpers
 * ========================================================= */
static void app_wakeup_cause_ui_init(void)
{
    /* Created by app_temp_display_ui_init(); kept for symmetry if the UI changes later. */
}

static void app_wakeup_cause_ui_set(app_wake_cause_t cause)
{
    if (!wakeup_label) return;

    const char* txt = "Wake: ?";
    switch (cause)
    {
    case APP_WAKE_TOUCH: txt = "Wake: touch";
        break;
    case APP_WAKE_TEMP_THRESHOLD: txt = "Wake: temp";
        break;
    case APP_WAKE_IMU: txt = "Wake: motion";
        break;
    default: txt = "Wake: unknown";
        break;
    }

    lvgl_port_lock(0);
    lv_label_set_text(wakeup_label, txt);
    lvgl_port_unlock();
}

static void app_configure_gpio_wakeup_sources(void)
{
    /* Configure only the HDC2080 IRQ pin here to keep init simple.
     * HDC2080 INT is typically open-drain active-low, so ensure a pull-up.
     */
    const gpio_config_t hdc_irq_cfg = {
        .pin_bit_mask = 1ULL << HDC2080_IRQ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&hdc_irq_cfg));

    /* IMPORTANT:
     * gpio_wakeup_enable() touches GPIO interrupt configuration.
     * If an IRQ line is already active (e.g. touch INT stuck low), doing this while
     * the ISR service is running can lead to ISR contention and interrupt WDT.
     *
     * So we only call this as part of the deep-sleep transition, after disabling
     * runtime touch IRQ handling.
     */
    ESP_ERROR_CHECK(gpio_wakeup_enable(TOUCH_GPIO_INT, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(HDC2080_IRQ, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    ESP_LOGI(TAG, "[SLEEP] GPIO wake armed: touch=%d, hdc2080=%d (low-level)",
             (int)TOUCH_GPIO_INT, (int)HDC2080_IRQ);
}

static void app_prepare_for_deep_sleep(void)
{
    /* Stop runtime IRQ-driven touch reads before reconfiguring GPIOs for wake. */
    (void)gpio_intr_disable(TOUCH_GPIO_INT);

    if (touch_irq_sem)
    {
        xSemaphoreTake(touch_irq_sem, 0);
    }

    app_configure_gpio_wakeup_sources();
}

static esp_err_t app_i2c_init_shared(void)
{
    if (i2c_bus_handle != NULL)
    {
        return ESP_OK;
    }

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = TOUCH_I2C_NUM,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_config, &i2c_bus_handle), TAG, "[I2C] new bus failed");
    ESP_LOGI(TAG, "[I2C] Master bus ready on I2C%d (SDA=%d SCL=%d)",
             TOUCH_I2C_NUM, (int)TOUCH_I2C_SDA, (int)TOUCH_I2C_SCL);

    // Scan I2C bus for devices
    ESP_LOGI(TAG, "[I2C] Scanning bus...");
    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(i2c_bus_handle, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "[I2C] Device found at 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "[I2C] Scan complete: %d device(s) found", found);

    return ESP_OK;
}

static esp_err_t app_qmi8658_init_optional(void)
{


    /* Reuse the existing shared I2C bus (same as touch + HDC2080). */
    ESP_RETURN_ON_ERROR(app_i2c_init_shared(), TAG, "[IMU] I2C init failed");

    /* Configure IMU INT1 as input with PULL-UP.
     * We use EXT1 ANY_LOW for wake, so the inactive level must be HIGH.
     */

    app_sleep_debug_dump_pins();
    const gpio_config_t imu_int_cfg = {
        .pin_bit_mask = (1ULL << (int)IMU_INT1_GPIO) ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&imu_int_cfg));

    /* Try both common I2C addresses (0x6B then 0x6A). */
    esp_err_t ret = qmi8658_init(&s_qmi8658, i2c_bus_handle, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK) {
        ret = qmi8658_init(&s_qmi8658, i2c_bus_handle, QMI8658_ADDRESS_LOW);
        ESP_LOGW(TAG, "[IMU] QMI8658 init failed (using alternate address 0x6A): %s", esp_err_to_name(ret));
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[IMU] QMI8658 initialized INT1=GPIO%d", (int)IMU_INT1_GPIO);
        app_sleep_debug_dump_pins();
    } else {
        ESP_LOGW(TAG, "[IMU] QMI8658 not detected/init failed: %s (IMU wake disabled)", esp_err_to_name(ret));
    }

    return ret;
}

static void app_note_activity(app_wake_cause_t cause, TickType_t* last_activity_ticks)
{
    if (last_activity_ticks)
    {
        *last_activity_ticks = xTaskGetTickCount();
    }

    app_display_turn_on();
    app_wakeup_cause_ui_set(cause);

    if (cause == APP_WAKE_TOUCH)
    {
        ESP_LOGI(TAG, "[UI] Touch activity -> display on for %d ms", DISPLAY_ON_MS);
    }
    else if (cause == APP_WAKE_TEMP_THRESHOLD)
    {
        ESP_LOGI(TAG, "[UI] Temp threshold activity -> display on for %d ms", DISPLAY_ON_MS);
    }
}

/* =========================================================
 * UI
 * ========================================================= */
static void app_button_cb(lv_event_t* e)
{
    (void)e;
    lv_display_rotation_t rot = lv_display_get_rotation(lvgl_disp);
    rot = (rot + 1) % 4; /* 0→1→2→3→0 */
    app_apply_rotation(rot);
    ESP_LOGI(TAG, "[UI] Rotation → %d", (int)rot);
}

static void app_main_display(void)
{
    lvgl_port_lock(0);

    lv_obj_t* scr = lv_scr_act();

    /* Logo */
    lv_obj_t* logo = lv_img_create(scr);
    lv_img_set_src(logo, &esp_logo);
    lv_obj_align(logo, LV_ALIGN_TOP_MID, 0, 20);

    /* Status */
    lv_obj_t* label = lv_label_create(scr);
    lv_label_set_text(label, "ST7789 + CST816S + LVGL 9.4\nTouch anywhere (green dot)\nRotate button");
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    /* Rotate button */
    lv_obj_t* btn = lv_btn_create(scr);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn, app_button_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* btn_lbl = lv_label_create(btn);
    lv_label_set_text(btn_lbl, "Rotate");

    /* Touch dot visualizer */
    touch_dot = lv_obj_create(scr);
    lv_obj_set_size(touch_dot, 8, 8);
    lv_obj_set_style_radius(touch_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(touch_dot, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_bg_opa(touch_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(touch_dot, 0, 0);
    lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);

    lvgl_port_unlock();
}


/* =========================================================
 * RUNTIME DIAGNOSTICS
 * ========================================================= */
/* Optional: call this right before deep sleep to log the levels you are about to arm with */
static void app_sleep_debug_dump_pins(void)
{
    const int touch_lvl = gpio_get_level(TOUCH_GPIO_INT);
    const int hdc_lvl   = gpio_get_level(HDC2080_IRQ);
    const int imu1_lvl  = gpio_get_level(IMU_INT1_GPIO);
    ESP_LOGI(TAG, "[SLEEP] Pin levels: touch GPIO%d=%d, hdc GPIO%d=%d, imu1 GPIO%d=%d",
             (int)TOUCH_GPIO_INT, touch_lvl, (int)HDC2080_IRQ, hdc_lvl, (int)IMU_INT1_GPIO, imu1_lvl);
}

static void app_wake_debug_dump_ext1(void)
{
    const esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "[WAKE] cause=%d", (int)wc);

    const int touch_lvl = gpio_get_level(TOUCH_GPIO_INT);
    const int hdc_lvl   = gpio_get_level(HDC2080_IRQ);
    const int imu1_lvl  = gpio_get_level(IMU_INT1_GPIO);

    ESP_LOGI(TAG, "[WAKE] boot pin levels: touch GPIO%d=%d, hdc GPIO%d=%d, imu1 GPIO%d=%d",
             (int)TOUCH_GPIO_INT, touch_lvl, (int)HDC2080_IRQ, hdc_lvl,
             (int)IMU_INT1_GPIO, imu1_lvl);

    // WoM: reading STATUS1 clears STATUS1.WoM and restores the INT line to its configured initial level.
    (void)qmi8658_wom_clear_status(&s_qmi8658);

    if (wc == ESP_SLEEP_WAKEUP_EXT1) {
        const uint64_t m = esp_sleep_get_ext1_wakeup_status();
        ESP_LOGI(TAG, "[WAKE] ext1 wake mask=0x%016" PRIx64, m);

        const bool touch_hit = (m & (1ULL << (int)TOUCH_GPIO_INT)) != 0;
        const bool hdc_hit   = (m & (1ULL << (int)HDC2080_IRQ)) != 0;
        const bool imu1_hit  = (m & (1ULL << (int)IMU_INT1_GPIO)) != 0;
        ESP_LOGI(TAG, "[WAKE] ext1 bits: touch=%d, hdc=%d, imu1=%d",
                 (int)touch_hit, (int)hdc_hit, (int)imu1_hit);
    }
}

static void app_configure_wakeup_sources_ext1_lowlevel(void)
{
    /* Deep sleep GPIO wake on ESP32-S3 must use EXT0/EXT1 (ESP_SLEEP_WAKEUP_GPIO is light sleep only).
     * Both CST816S INT and HDC2080 INT are active-low (open-drain) => wake on ANY_LOW.
     * QMI8658 WoM is a toggle output. We configure it to start LOW (CAL1_H) and then
     * toggle HIGH on motion, so wake on ANY_HIGH for the IMU pin.
     */

    /* Make sure both wake pins are inputs with pull-ups right before arming EXT1.
     * (We do this here so it also applies after wake, regardless of earlier init order.)
     */
    const gpio_config_t wake_gpio_cfg = {
        .pin_bit_mask = (1ULL << (int)HDC2080_IRQ) | (1ULL << (int)TOUCH_GPIO_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&wake_gpio_cfg));

    /* If touch INT is already asserted (low) right now, arming it as an ANY_LOW wake source
     * will either cause an immediate wake-loop or make it look like touch never wakes.
     * In that case, fall back to only HDC2080 for this sleep cycle.
     */
    uint64_t mask = (1ULL << (int)HDC2080_IRQ) | (1ULL << (int)TOUCH_GPIO_INT);
    const int touch_lvl = gpio_get_level(TOUCH_GPIO_INT);
    if (touch_lvl == 0) {
        ESP_LOGW(TAG, "[SLEEP] Touch INT already LOW; excluding touch from EXT1 mask for this cycle");
        mask &= ~(1ULL << (int)TOUCH_GPIO_INT);
    }

    ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(mask, ESP_EXT1_WAKEUP_ANY_LOW));
#else
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW));
#endif

    ESP_LOGI(TAG, "[SLEEP] EXT1 wake armed mask=0x%016" PRIx64 " ANY_LOW", mask);
}


static void app_runtime_diag_task(void* arg)
{
    (void)arg;
    while (1)
    {
        ESP_LOGI(TAG, ".");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static esp_err_t app_lcd_apply_gap_for_rotation(lv_display_rotation_t rot)
{
    int x_off;
    int y_off;

    /* Gap is a panel RAM-window quirk; we tie it to portrait vs landscape orientation. */
    if (rot == LV_DISPLAY_ROTATION_90 || rot == LV_DISPLAY_ROTATION_270) {
        x_off = LCD_LANDSCAPE_X_OFFSET;
        y_off = LCD_LANDSCAPE_Y_OFFSET;
    } else {
        x_off = LCD_PORTRAIT_X_OFFSET;
        y_off = LCD_PORTRAIT_Y_OFFSET;
    }

    ESP_LOGI(TAG, "[LCD] set_gap for rotation=%d -> x=%d y=%d", (int)rot, x_off, y_off);
    return esp_lcd_panel_set_gap(lcd_panel, x_off, y_off);
}

static void app_apply_rotation(lv_display_rotation_t rot)
{
    if (lvgl_disp == NULL || lcd_panel == NULL) {
        return;
    }

    if (app_lcd_apply_gap_for_rotation(rot) != ESP_OK) {
        ESP_LOGW(TAG, "[LCD] set_gap failed for rotation=%d", (int)rot);
    }

    /* LVGL will rotate rendering + input mapping automatically. */
    lv_display_set_rotation(lvgl_disp, rot);
}

static void app_touch_dot_timer_cb(lv_timer_t *t)
{
    (void)t;

    if (touch_dot == NULL || lvgl_touch_indev == NULL) {
        return;
    }

    /* This is LVGL's final (rotation-aware) pointer point */
    lv_point_t p;
    lv_indev_get_point(lvgl_touch_indev, &p);

    const lv_indev_state_t st = lv_indev_get_state(lvgl_touch_indev);
    const bool pressed = (st == LV_INDEV_STATE_PRESSED);

    app_touch_dot_update((int32_t)p.x, (int32_t)p.y, pressed);
}

static void app_touch_dot_update(int32_t x, int32_t y, bool pressed)
{
    if (touch_dot == NULL || lvgl_disp == NULL) {
        return;
    }

    lvgl_port_lock(0);

    if (!pressed) {
        lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
        lvgl_port_unlock();
        return;
    }

    /*
     * x/y must be in the CURRENT UI coordinate space.
     * (i.e. already rotation-aware, as returned by lv_indev_get_point())
     */
    int32_t w = lv_display_get_horizontal_resolution(lvgl_disp);
    int32_t h = lv_display_get_vertical_resolution(lvgl_disp);

    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > w - 1) x = w - 1;
    if (y > h - 1) y = h - 1;

    int32_t dot_w = (int32_t)lv_obj_get_width(touch_dot);
    int32_t dot_h = (int32_t)lv_obj_get_height(touch_dot);

    lv_obj_clear_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(touch_dot, x - dot_w / 2, y - dot_h / 2);

    lvgl_port_unlock();
}

// HDC 2080 TEMP SENSOR
// #define HDC2080_ADDR 0x40


/* =========================================================
 * MAIN
 * ========================================================= */

/* =========================================================
 * HDC2080 TEMP SENSOR
 * ========================================================= */
static esp_err_t app_hdc2080_init(void)
{
    hdc2080_handle_t h = NULL;
    hdc2080_config_t cfg = {
        .i2c_bus = i2c_bus_handle,
        .i2c_addr = 0x40,
        .i2c_timeout_ms = 1000,

        .int_gpio = HDC2080_IRQ,
        .int_polarity = HDC2080_INT_ACTIVE_LOW,  // same as ACTIVE_LOW=true
        .int_pull = HDC2080_INT_PULLUP,          // same as USE_INTERNAL_PULL=true + ACTIVE_LOW

        .amm_rate = HDC2080_AMM_1HZ,
        .meas_wait_ms = 30,
        .deassert_wait_ms = 200,

        .debounce_ms = 30,
    };

    ESP_ERROR_CHECK(hdc2080_init(&cfg, &h));

    // Optional: dump configuration for verification
    ESP_ERROR_CHECK(hdc2080_dump_config(h));

    // Clear any stale status after reset/wake
    hdc2080_status_t st0 = {0};
    ESP_ERROR_CHECK(hdc2080_read_and_clear_status(h, &st0));
    ESP_LOGI(TAG, "HDC2080 INT_STATUS(0x04) after wake/init: 0x%02X (TH=%d TL=%d)",
             st0.raw, (int)st0.th, (int)st0.tl);

    // 2) Baseline measure
    hdc2080_measurement_t m = {0};
    ESP_ERROR_CHECK(hdc2080_measure_temp_humidity(h, true /*temp_only*/, &m));
    ESP_LOGI(TAG, "Baseline temp: %.2f C", (double)m.temp_c);

    // Update UI once before sleep (humidity not measured in temp_only mode)
    app_temp_display_update(m.temp_c, 0.0f, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 3) Decide and arm. Example chooses rising threshold (TH): baseline + delta
    const bool want_rising = true;


    if (want_rising) {
        ESP_ERROR_CHECK(hdc2080_arm_irq(h, HDC2080_IRQ_TH, m.temp_c, TEMP_DELTA_WAKE_C));
        ESP_LOGI(TAG,"RAISING - Baseline: %.2f C, new target temp %.2f",
         (double)m.temp_c,
         (double)(m.temp_c + TEMP_DELTA_WAKE_C));

    } else {
        ESP_LOGI(TAG,"FALLING - Baseline: %.2f C, new target temp %.2f",
                 (double)m.temp_c,
                 (double)(m.temp_c - TEMP_DELTA_WAKE_C));

        ESP_ERROR_CHECK(hdc2080_arm_irq(h, HDC2080_IRQ_TL, m.temp_c, TEMP_DELTA_WAKE_C));
    }

    // Clear again so we don't immediately wake from a latched flag
    hdc2080_status_t st1 = {0};
    ESP_ERROR_CHECK(hdc2080_read_and_clear_status(h, &st1));
    ESP_LOGI(TAG, "HDC2080 INT_STATUS(0x04) after arming: 0x%02X (TH=%d TL=%d)",
             st1.raw, (int)st1.th, (int)st1.tl);

    // Confirm INT pin is inactive before enabling GPIO wake + deep sleep.
    // For ACTIVE_LOW, inactive means GPIO reads HIGH.
    if (hdc2080_int_is_asserted(h)) {
        ESP_LOGW(TAG, "HDC2080 INT is still asserted before sleep (GPIO%d level=%d). "
                     "Either still above TH, status not cleared, or polarity/wiring issue.",
                     (int)HDC2080_IRQ, gpio_get_level(HDC2080_IRQ));
    } else {
        ESP_LOGI(TAG, "HDC2080 INT inactive before sleep (GPIO%d level=%d)",
                 (int)HDC2080_IRQ, gpio_get_level(HDC2080_IRQ));
    }

    // One more read-to-clear right before sleeping to avoid getting stuck asserted.
    hdc2080_status_t st2 = {0};
    ESP_ERROR_CHECK(hdc2080_read_and_clear_status(h, &st2));
    ESP_LOGI(TAG, "HDC2080 INT_STATUS(0x04) before deep sleep: 0x%02X (TH=%d TL=%d)",
             st2.raw, (int)st2.th, (int)st2.tl);

    return ESP_OK;
}

/* =========================================================
 * POWER SAVING (ALL-IN-ONE) – keep at end of file
 *
 * Goal:
 *  - Minimize board leakage before deep sleep
 *  - Preserve wake from CST816S INT (GPIO21) and HDC2080 INT (GPIO15)
 *  - Avoid ISR contention / wake loops if a wake pin is already asserted
 *
 * Notes:
 *  - Board-level 3V3 peripherals (buck IQ, LCD VDD rail, sensors) can dominate current.
 *  - We can still reduce current by putting peripherals into sleep/suspend and
 *    ensuring GPIOs aren’t sourcing/sinking current.
 *
 * This is a single function on purpose (per request).
 * ========================================================= */
static void app_power_save_shutdown_and_sleep_ext1(bool keep_touch_wake, bool keep_hdc_wake)
{
    // /* --- 0) Quiesce runtime IRQ-driven touch reads (prevents WDT issues) --- */
     (void)gpio_intr_disable(TOUCH_GPIO_INT);
     if (touch_irq_sem) {
         (void)xSemaphoreTake(touch_irq_sem, 0);
     }

    vTaskDelay(pdMS_TO_TICKS(20));


    /* Touch controller: enter deep sleep / low power mode.
     * The INT pin stays wired to the ESP32 for wake (EXT1 ANY_LOW).
     */
    if (touch_handle) {
        esp_err_t tr = esp_lcd_touch_cst816s_enter_sleep(touch_handle);
        if (tr != ESP_OK) {
            ESP_LOGW(TAG, "[SLEEP] CST816S enter sleep failed: %s", esp_err_to_name(tr));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 2) LCD off (backlight first, then controller) --- #1#
    gpio_set_level(LCD_GPIO_BL, !LCD_BL_ON_LEVEL);
    if (lcd_panel) {
        (void)esp_lcd_panel_disp_on_off(lcd_panel, false);
        (void)esp_lcd_panel_disp_sleep(lcd_panel, true);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    /* QMI8658 WoM:
     * ESP32-S3 EXT1 uses a SINGLE wake polarity for all pins. Touch + HDC are active-low => ANY_LOW.
     * Therefore configure WoM initial level HIGH, so motion toggles INT LOW (ANY_LOW wake).
     */
    const qmi8658_wom_config_t wom_cfg = {
        .int_pin = QMI8658_INT_PIN1,
        .initial_level = QMI8658_WOM_INITIAL_LEVEL_HIGH,
        .threshold_mg = QMI8658_WOM_THRESHOLD_DEFAULT,
        .blanking_samples = 3,
    };

    ESP_LOGI(TAG,"Before WOM irq enabled");
    app_sleep_debug_dump_pins();

    esp_err_t wr = qmi8658_enable_wake_on_motion_cfg(&s_qmi8658, &wom_cfg);
    if (wr != ESP_OK) {
        ESP_LOGW(TAG, "[SLEEP] QMI8658 enable WoM failed: %s", esp_err_to_name(wr));
    } else {
        ESP_LOGI(TAG, "[SLEEP] QMI8658 WoM enabled (INT%u initial HIGH -> toggles LOW) thr=%u blank=%u",
                 (unsigned)wom_cfg.int_pin, (unsigned)wom_cfg.threshold_mg, (unsigned)wom_cfg.blanking_samples);
    }
    ESP_LOGI(TAG,"Before  AFTER irq enabled");

    /* --- 3) Configure EXT1 wake ---
     * Touch + HDC2080 are active-low (open-drain) => wake on ANY_LOW.
     * QMI8658 WoM is configured above to toggle LOW on motion.
     */
    uint64_t mask_low  = 0;

    //if (keep_touch_wake) mask_low |= (1ULL << (int)TOUCH_GPIO_INT);
    if (keep_hdc_wake)   mask_low |= (1ULL << (int)HDC2080_IRQ);
    mask_low |= (1ULL << (int)IMU_INT1_GPIO);

    // Ensure wake pins are inputs with pull-ups (ANY_LOW inactive level = HIGH).
    gpio_config_t wake_cfg = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    wake_cfg.pin_bit_mask = mask_low;
    if (wake_cfg.pin_bit_mask) {
        ESP_ERROR_CHECK(gpio_config(&wake_cfg));
    }

    // Clear STATUS1 right before arming EXT1 so the IMU INT returns to initial level.
    // Reading STATUS1 is the ONLY datasheet-defined way to reset the WoM toggle output.
    (void)qmi8658_wom_clear_status(&s_qmi8658);

    // If a pin is already LOW, arming ANY_LOW can cause an immediate wake loop.
    // Exclude it for this sleep cycle (preferred over sleeping into a loop).
    if ((mask_low & (1ULL << (int)IMU_INT1_GPIO)) && gpio_get_level(IMU_INT1_GPIO) == 0) {
        ESP_LOGW(TAG, "[SLEEP] IMU INT1 already LOW before sleep; clearing STATUS1 + delaying, then excluding if still LOW");
        (void)qmi8658_wom_clear_status(&s_qmi8658);
        vTaskDelay(pdMS_TO_TICKS(20));
        if (gpio_get_level(IMU_INT1_GPIO) == 0) {
            mask_low &= ~(1ULL << (int)IMU_INT1_GPIO);
            ESP_LOGW(TAG, "[SLEEP] IMU INT1 still LOW; excluded from EXT1 mask for this cycle");
        }
    }

    if ((mask_low & (1ULL << (int)TOUCH_GPIO_INT)) && gpio_get_level(TOUCH_GPIO_INT) == 0) {
        ESP_LOGW(TAG, "[SLEEP] Touch INT already LOW; excluding touch from EXT1 mask for this cycle");
        mask_low &= ~(1ULL << (int)TOUCH_GPIO_INT);
    }
    if ((mask_low & (1ULL << (int)HDC2080_IRQ)) && gpio_get_level(HDC2080_IRQ) == 0) {
        ESP_LOGW(TAG, "[SLEEP] HDC2080 INT already LOW; excluding HDC2080 from EXT1 mask for this cycle");
        mask_low &= ~(1ULL << (int)HDC2080_IRQ);
    }

    ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));

    if (mask_low) {
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(mask_low, ESP_EXT1_WAKEUP_ANY_LOW));
    } else {
        ESP_LOGE(TAG, "[SLEEP] EXT1 mask is empty after sanitizing; refusing to enter deep sleep");
        return;
    }

    ESP_LOGI(TAG, "[SLEEP] Entering deep sleep; EXT1 mask=0x%016" PRIx64 " ANY_LOW", mask_low);
    app_sleep_debug_dump_pins();
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_deep_sleep_start();
}


//---------------------------------------------------------------------------------------------------------------
// APP MAIN
//---------------------------------------------------------------------------------------------------------------

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "[APP] Starting...");

    //#define TOUCH_DISPLAY_DEMO
#ifdef TOUCH_DISPLAY_DEMO
    ESP_LOGI(TAG, "[APP] Touch display demo");
    touch_irq_sem = xSemaphoreCreateBinary();
    if (!touch_irq_sem)
    {
        ESP_LOGE(TAG, "Failed to create touch semaphore");
        return;
    }

    if (app_lcd_init() != ESP_OK) return;
    if (app_touch_init() != ESP_OK) return;
    if (app_lvgl_init() != ESP_OK) return;

    xTaskCreate(app_runtime_diag_task, "diag", 3072, NULL, 1, NULL);
    app_main_display();
#else
    /* Temperature + humidity demo */
    ESP_LOGI(TAG, "Welcome - Ali-Box Sensor ");
    touch_irq_sem = xSemaphoreCreateBinary();
    if (!touch_irq_sem)
    {
        ESP_LOGE(TAG, "Failed to create touch semaphore");
        return;
    }


    if (app_touch_init() != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(100));  // small delay to ensure LCD is ready

    if (app_lcd_init() != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(100));  // small delay to ensure LCD is ready
    if (app_lvgl_init() != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(100));  // small delay to ensure LCD is ready
    if (app_qmi8658_init_optional()!= ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(100));  // small delay to ensure LCD is ready
    if (app_hdc2080_init() != ESP_OK) return;

    /* Create the temp/humidity UI */
    app_temp_display_ui_init();
    app_wakeup_cause_ui_init();
    ESP_LOGI(TAG, "=== HDC2080 TH/TL IRQ example (ESP-IDF v5.5 component) ===");

    app_wake_debug_dump_ext1();

    /* Determine wakeup cause (best-effort) */
    app_wake_cause_t wake_ui = APP_WAKE_UNKNOWN;
    const esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
    if (wc == ESP_SLEEP_WAKEUP_EXT1)
    {
        const uint64_t m = esp_sleep_get_ext1_wakeup_status();
        if (m & (1ULL << (int)TOUCH_GPIO_INT)) {
            wake_ui = APP_WAKE_TOUCH;
        } else if (m & (1ULL << (int)HDC2080_IRQ)) {
            wake_ui = APP_WAKE_TEMP_THRESHOLD;
        } else if (m & (1ULL << (int)IMU_INT1_GPIO)) {
            wake_ui = APP_WAKE_IMU;
        } else {
            wake_ui = APP_WAKE_UNKNOWN;
        }
    } else {
        ESP_LOGI(TAG, "Not woke by EXT1 (cause=%d)", (int)wc);
    }

    /* Fallback heuristic: if EXT1 did not indicate the source (e.g. IMU wasn't armed for EXT1),
     * infer from current pin levels. IMU WoM is configured as active-low with pull-up.
     */
    if (wake_ui == APP_WAKE_UNKNOWN) {
        if (gpio_get_level(IMU_INT1_GPIO) == 0) {
            wake_ui = APP_WAKE_IMU;
        }
    }

    // print wakeup in clear text
    ESP_LOGI(TAG, "Wakeup cause: %s",
             (wake_ui == APP_WAKE_TOUCH) ? "Touch" :
             (wake_ui == APP_WAKE_TEMP_THRESHOLD) ? "Temp threshold" :
             (wake_ui == APP_WAKE_IMU) ? "Motion (IMU)" :
             "Unknown");

    app_wakeup_cause_ui_set(wake_ui);



    /* Optional: init IMU so it can be used as an additional deep-sleep wake source.
     * If the IMU isn't present, we just log a warning and continue.
     */




#define USE_UNIFIED_POWER_SAVE_ROUTINE
#ifndef USE_UNIFIED_POWER_SAVE_ROUTINE
    ESP_LOGI(TAG, "Entering deep sleep; wake on GPIO: touch GPIO%d low, hdc2080 GPIO%d low",
         (int)TOUCH_GPIO_INT, (int)HDC2080_IRQ);

    app_display_turn_off();
    vTaskDelay(pdMS_TO_TICKS(20)); // let the SPI transaction / GPIO settle

    // Give logs time to flush
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_deep_sleep_start();
#else


    ESP_LOGI(TAG, "Entering deep sleep via unified power-save routine");
    app_power_save_shutdown_and_sleep_ext1(true /*touch*/, true /*hdc2080*/);

    // On wake, execution restarts from app_main().
    // Typical pattern: check wake cause, read/clear status, measure, re-arm, sleep again.

#endif
#endif
}

