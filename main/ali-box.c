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

#include "hdc2080.h"

#include <stdio.h>  /* snprintf */
#include <inttypes.h>

/* Image asset */
LV_IMG_DECLARE(esp_logo);

/* =========================================================
 * CST816S TOUCH (IRQ-driven read, per component README)
 * ========================================================= */
#include "esp_lcd_touch_cst816s.h"

static const char* TAG = "MAIN";

/* === LCD native resolution (portrait) === */
#define EXAMPLE_LCD_H_RES   (170)
#define EXAMPLE_LCD_V_RES   (320)

/* === PANEL GAP (RAM window offset) ===
 * This 170x320 ST7789 panel needs a RAM window offset ("gap").
 * Apply a different gap depending on orientation.
 */
#define EXAMPLE_LCD_PORTRAIT_X_OFFSET  (35)
#define EXAMPLE_LCD_PORTRAIT_Y_OFFSET  (0)
#define EXAMPLE_LCD_LANDSCAPE_X_OFFSET (0)
#define EXAMPLE_LCD_LANDSCAPE_Y_OFFSET (35)

/* === LCD SPI settings === */
#define EXAMPLE_LCD_SPI_NUM          (SPI3_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ     (26 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS         (8)
#define EXAMPLE_LCD_PARAM_BITS       (8)
#define EXAMPLE_LCD_BITS_PER_PIXEL   (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (40)
#define EXAMPLE_LCD_BL_ON_LEVEL      (0)

/* === LCD pins === */
#define EXAMPLE_LCD_GPIO_RST     (GPIO_NUM_9)
#define EXAMPLE_LCD_GPIO_SCLK    (GPIO_NUM_10)
#define EXAMPLE_LCD_GPIO_DC      (GPIO_NUM_11)
#define EXAMPLE_LCD_GPIO_CS      (GPIO_NUM_12)
#define EXAMPLE_LCD_GPIO_MOSI    (GPIO_NUM_13)
#define EXAMPLE_LCD_GPIO_BL      (GPIO_NUM_14)

/* === Touch I2C settings === */
#define EXAMPLE_TOUCH_I2C_NUM        (0)
#define EXAMPLE_TOUCH_I2C_CLK_HZ     (400000)
#define EXAMPLE_TOUCH_I2C_SDA        (GPIO_NUM_47)
#define EXAMPLE_TOUCH_I2C_SCL        (GPIO_NUM_48)
#define EXAMPLE_TOUCH_GPIO_INT       (GPIO_NUM_21)

/* === HDC2080 settings === */
#define HDC2080_ADDR 0x40
#define EXAMPLE_HDC2080_IRQ  (GPIO_NUM_15)
#define EXAMPLE_TEMP_DELTA_WAKE_C    (0.5f)
#define EXAMPLE_DISPLAY_ON_MS        (3000)

/* Wake sources
 * Touch IRQ is on GPIO21 (EXAMPLE_TOUCH_GPIO_INT).
 * HDC2080 IRQ is on GPIO18 (EXAMPLE_HDC2080_IRQ).
 * Both can be configured as GPIO deep-sleep wake sources.
 */

typedef enum
{
    APP_WAKE_UNKNOWN = 0,
    APP_WAKE_TOUCH,
    APP_WAKE_TEMP_THRESHOLD,
} app_wake_cause_t;

static lv_obj_t* wakeup_label = NULL;

static void app_wakeup_cause_ui_init(void);
static void app_wakeup_cause_ui_set(app_wake_cause_t cause);
static esp_err_t app_hdc2080_prepare_temp_threshold_wakeup(const hdc2080_t* s, float delta_c);
static esp_err_t app_i2c_init_shared(void);
static esp_err_t app_hdc2080_init(hdc2080_t* s);
static bool app_hdc2080_irq_gpio_asserted(void);
static void app_note_activity(app_wake_cause_t cause, TickType_t* last_activity_ticks);
static void app_configure_gpio_wakeup_sources(void);
static void app_prepare_for_deep_sleep(void);

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

/* IRQ semaphore for CST816S (per component README) */
static SemaphoreHandle_t touch_irq_sem = NULL;

/* Forward declarations */
static esp_err_t app_lcd_init(void);
static esp_err_t app_touch_init(void);
static esp_err_t app_lvgl_init(void);
static void app_main_display(void);
static void app_runtime_diag_task(void* arg);
static esp_err_t app_lcd_apply_gap_for_rotation(lv_display_rotation_t rot);
static void app_apply_rotation(lv_display_rotation_t rot);


/* =========================================================
 * HDC2080 IRQ (runtime ISR -> task)
 * =========================================================
 * Keep this isolated from the CST816S touch IRQ path.
 * We do not perform I2C transactions in ISR context.
 */
static TaskHandle_t hdc2080_irq_task_handle = NULL;
static volatile bool hdc2080_irq_pending = false;

static void IRAM_ATTR hdc2080_gpio_isr_handler(void* arg)
{
    (void)arg;
    BaseType_t higher_woken = pdFALSE;
    hdc2080_irq_pending = true;
    if (hdc2080_irq_task_handle)
    {
        vTaskNotifyGiveFromISR(hdc2080_irq_task_handle, &higher_woken);
    }
    if (higher_woken)
    {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t app_hdc2080_irq_runtime_init(void)
{


    /* Configure the dedicated IRQ pin (HDC2080 INT is typically open-drain active-low). */
    gpio_config_t irq_cfg = {
        .pin_bit_mask = 1ULL << EXAMPLE_HDC2080_IRQ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&irq_cfg));

    ESP_LOGI(TAG, "[HDC2080] Init addr=0x%02X irq_gpio=%d", HDC2080_ADDR, (int)EXAMPLE_HDC2080_IRQ);

    /* Only configure runtime edge-triggered ISR here.
     * Deep-sleep wake configuration is handled separately.
     */
    ESP_RETURN_ON_ERROR(gpio_intr_disable(EXAMPLE_HDC2080_IRQ), TAG, "[HDC2080][IRQ] intr disable failed");

    /* Install ISR service if not already installed by other code.
     * Ignore 'already installed' error to stay compatible with touch demo code.
     */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        return isr_err;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(EXAMPLE_HDC2080_IRQ, hdc2080_gpio_isr_handler, NULL));

    return ESP_OK;
}

static void app_hdc2080_irq_runtime_deinit(void)
{
    (void)gpio_intr_disable(EXAMPLE_HDC2080_IRQ);
    (void)gpio_isr_handler_remove(EXAMPLE_HDC2080_IRQ);
}


/* =========================================================
 * HDC2080 IRQ task (runs in normal task context; safe for I2C)
 * ========================================================= */
typedef enum
{
    SLEEP_STATE_SLEEPING = 0,
    SLEEP_STATE_ACTIVE,
} sleep_state_t;

static volatile sleep_state_t sleep_state = SLEEP_STATE_SLEEPING;
static volatile TickType_t active_until = 0;

static void app_hdc2080_irq_task(void* arg)
{
    hdc2080_t* s = (hdc2080_t*)arg;
    float temp_c = 0.0f;

    while (1)
    {
        /* Coalesce multiple IRQ edges into one handling pass. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        hdc2080_irq_pending = false;

        if (!s || !hdc2080_is_connected(s))
        {
            continue;
        }

        /* Snapshot + clear interrupt status (device deasserts INT after this). */
        uint8_t status = 0;
        esp_err_t st_err = hdc2080_read_interrupt_status(s, &status);
        if (st_err != ESP_OK)
        {
            ESP_LOGW(TAG, "[HDC2080][IRQ] Failed to read interrupt status: %s", esp_err_to_name(st_err));
            continue;
        }

        /* Document threshold hits as simply as possible. */
        if (status & 0x40) ESP_LOGW(TAG, "[HDC2080][IRQ] TEMP HIGH!");
        if (status & 0x20) ESP_LOGW(TAG, "[HDC2080][IRQ] TEMP LOW!");

        /* Fresh data ready. */
        esp_err_t t_err = hdc2080_read_temp_c(s, &temp_c);
        if (t_err == ESP_OK)
        {
            ESP_LOGI(TAG, "[HDC2080] Temp: %.2f C (status=0x%02X)", (double)temp_c, status);
        }
        else
        {
            ESP_LOGW(TAG, "[HDC2080] Temp read failed after IRQ: %s", esp_err_to_name(t_err));
        }

        app_note_activity(APP_WAKE_TEMP_THRESHOLD, NULL);
        active_until = xTaskGetTickCount() + pdMS_TO_TICKS(EXAMPLE_DISPLAY_ON_MS);
        sleep_state = SLEEP_STATE_ACTIVE;
    }
}

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
    ESP_LOGI(TAG, "[LCD] Init %dx%d ST7789", EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

    /* Backlight GPIO */
    gpio_config_t bk_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_config));
    gpio_set_level(EXAMPLE_LCD_GPIO_BL, !EXAMPLE_LCD_BL_ON_LEVEL);

    /* SPI bus */
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    /* Panel IO */
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io));

    /* ST7789 panel */
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));

    /* Backlight ON */
    gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL);

    return ESP_OK;
}

/* =========================================================
 * TOUCH INIT
 * ========================================================= */
static esp_err_t app_touch_init(void)
{
    ESP_LOGI(TAG, "[TOUCH] CST816S init");

    ESP_RETURN_ON_ERROR(app_i2c_init_shared(), TAG, "[I2C] init failed");

    /* Touch config (RAW panel-native, LVGL rotates automatically) */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = EXAMPLE_TOUCH_GPIO_INT,
        .interrupt_callback = touch_isr_callback,
        .levels = {.reset = 0, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0}, /* No manual rotation */
    };

    esp_lcd_panel_io_handle_t tp_io_handle;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = EXAMPLE_TOUCH_I2C_CLK_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle));

    return ESP_OK;
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
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
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
    gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL);
}

static void app_display_turn_off(void)
{
    gpio_set_level(EXAMPLE_LCD_GPIO_BL, !EXAMPLE_LCD_BL_ON_LEVEL);
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
    default: txt = "Wake: unknown";
        break;
    }

    lvgl_port_lock(0);
    lv_label_set_text(wakeup_label, txt);
    lvgl_port_unlock();
}

static void app_configure_gpio_wakeup_sources(void)
{
    /* IMPORTANT:
     * gpio_wakeup_enable() touches GPIO interrupt configuration.
     * If an IRQ line is already active (e.g. touch INT stuck low), doing this while
     * the ISR service is running can lead to ISR contention and interrupt WDT.
     *
     * So we only call this as part of the deep-sleep transition, after disabling
     * runtime touch IRQ handling.
     */
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_TOUCH_GPIO_INT, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_HDC2080_IRQ, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    ESP_LOGI(TAG, "[SLEEP] GPIO wake armed: touch=%d, hdc2080=%d (low-level)",
             (int)EXAMPLE_TOUCH_GPIO_INT, (int)EXAMPLE_HDC2080_IRQ);
}

static void app_prepare_for_deep_sleep(void)
{
    /* Stop runtime IRQ-driven touch reads before reconfiguring GPIOs for wake. */
    (void)gpio_intr_disable(EXAMPLE_TOUCH_GPIO_INT);

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
        .i2c_port = EXAMPLE_TOUCH_I2C_NUM,
        .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
        .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_config, &i2c_bus_handle), TAG, "[I2C] new bus failed");
    ESP_LOGI(TAG, "[I2C] Master bus ready on I2C%d (SDA=%d SCL=%d)",
             EXAMPLE_TOUCH_I2C_NUM, (int)EXAMPLE_TOUCH_I2C_SDA, (int)EXAMPLE_TOUCH_I2C_SCL);
    return ESP_OK;
}

static esp_err_t app_hdc2080_init(hdc2080_t* s)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(app_i2c_init_shared(), TAG, "[I2C] init failed");


    ESP_LOGI(TAG, "[HDC2080] Initializing...");
    ESP_RETURN_ON_ERROR(hdc2080_init(s, i2c_bus_handle, HDC2080_ADDR, 100000), TAG, "HDC init failed");
    vTaskDelay(pdMS_TO_TICKS(50)); // Stabilize
    if (!hdc2080_is_connected(s))
    {
        ESP_LOGW(TAG, "[HDC2080] Sensor not found at addr=0x%02X", HDC2080_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    // 1. HARD RESET (clears everything)
    ESP_RETURN_ON_ERROR(hdc2080_reset(s), TAG, "Reset failed");
    vTaskDelay(pdMS_TO_TICKS(50)); // Stabilize

    // 2. MEASUREMENT CONFIG (0x0F): 14-bit Temp-only
    uint8_t meas = 0x00; // TRES14=00 HRES14=00 MEAS_TEMP=10
    ESP_RETURN_ON_ERROR(hdc2080_write_reg(s, 0x0F, meas), TAG, "Meas config failed");

    // 3. THRESHOLD ENABLE (0x07): TH+TL only (0x60)
    uint8_t int_en = 0x60; // TH_EN=1 TL_EN=1 HH/HL=0
    ESP_RETURN_ON_ERROR(hdc2080_write_reg(s, 0x07, int_en), TAG, "Int enable failed");

    // 4. MAIN CONFIG (0x0E): 1Hz AMM + Comparator + Active Low + IRQ enabled
    uint8_t config = 0x55; // RST=0 AMM1Hz=101 HEAT=0 IRQ_EN=1 POL=0 MODE=1
    ESP_RETURN_ON_ERROR(hdc2080_write_reg(s, 0x0E, config), TAG, "Config failed");

    // 5. KICKSTART AUTO MEASUREMENT (triggers first conversion)
    meas |= 0x01; // MEAS_TRIG=1
    ESP_RETURN_ON_ERROR(hdc2080_write_reg(s, 0x0F, meas), TAG, "Kickstart failed");
    vTaskDelay(pdMS_TO_TICKS(10)); // ~6ms conversion

    // 6. Verify
    uint8_t status;
    hdc2080_read_interrupt_status(s, &status);
    ESP_LOGI(TAG, "[HDC2080] Init complete: status=0x%02X", status);

    // 7. ARM first thresholds (±2°C example)
    ESP_RETURN_ON_ERROR(app_hdc2080_prepare_temp_threshold_wakeup(s, 0.2f), TAG, "Prepare thr wake failed");

    return ESP_OK;
}

static esp_err_t app_hdc2080_prepare_temp_threshold_wakeup(const hdc2080_t *s, float delta_c)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    if (!hdc2080_is_connected(s)) return ESP_ERR_NOT_FOUND;

    float t = 0.0f;

    // 1. Fresh baseline temp (AMM already running)
    ESP_RETURN_ON_ERROR(hdc2080_read_temp_c(s, &t), TAG, "HDC read temp failed");

    // 2. Clear stale status (unlatches IRQ)
    uint8_t status;
    ESP_RETURN_ON_ERROR(hdc2080_read_interrupt_status(s, &status), TAG, "Clear status failed");
    ESP_LOGD(TAG, "[HDC2080] Cleared status=0x%02X", status);

    // 3. Arm thresholds around baseline
    const float low  = t - delta_c;
    const float high = t + delta_c;
    ESP_RETURN_ON_ERROR(hdc2080_set_low_temp(s, low), TAG, "Set low thr failed");
    ESP_RETURN_ON_ERROR(hdc2080_set_high_temp(s, high), TAG, "Set high thr failed");

    // 4. Enable TH+TL only (0x07 = 0x60)
    ESP_RETURN_ON_ERROR(hdc2080_enable_threshold_interrupt(s, HDC2080_TEMP_ONLY),
                        TAG, "Enable TH/TL failed");

    // 5. Static config (set once at boot, not every re-arm)
    //    - Comparator + 1Hz AMM + Active Low + DRDY_INT_EN
    //    - Move these to init() if not already

    ESP_LOGI(TAG, "[HDC2080] Armed: %.2f°C ±%.2f°C (low=%.2f high=%.2f)",
             (double)t, (double)delta_c, (double)low, (double)high);

    return ESP_OK;
}


static bool app_hdc2080_irq_gpio_asserted(void)
{
    /* HDC2080 INT is typically active-low. */
    int lvl = gpio_get_level(EXAMPLE_HDC2080_IRQ);
    return (lvl == 0);
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
        ESP_LOGI(TAG, "[UI] Touch activity -> display on for %d ms", EXAMPLE_DISPLAY_ON_MS);
    }
    else if (cause == APP_WAKE_TEMP_THRESHOLD)
    {
        ESP_LOGI(TAG, "[UI] Temp threshold activity -> display on for %d ms", EXAMPLE_DISPLAY_ON_MS);
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


// HDC 2080 TEMP SENSOR
// #define HDC2080_ADDR 0x40

static hdc2080_t hdc;

/* =========================================================
 * MAIN
 * ========================================================= */
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
    ESP_LOGI(TAG, "[APP] HD2080 demo");
    touch_irq_sem = xSemaphoreCreateBinary();
    if (!touch_irq_sem)
    {
        ESP_LOGE(TAG, "Failed to create touch semaphore");
        return;
    }

    if (app_lcd_init() != ESP_OK) return;
    if (app_touch_init() != ESP_OK) return;
    if (app_lvgl_init() != ESP_OK) return;

    /* Create the temp/humidity UI */
    app_temp_display_ui_init();
    app_wakeup_cause_ui_init();

    /* HDC2080 init kept separate from touch init (shared I2C bus under the hood) */
    const bool sensor_connected = (app_hdc2080_init(&hdc) == ESP_OK);
    if (!sensor_connected)
    {
        ESP_LOGW(TAG, "[HDC2080] Not detected at 0x%02X", HDC2080_ADDR);
    }
    uint8_t st = 0;
    esp_err_t err = hdc2080_read_interrupt_status(&hdc, &st);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "[HDC2080] Interrupt status reg: 0x%02X", st);
    }

    /* After boot/wake, arm the temperature threshold interrupt around current temperature */
     esp_err_t thr_err = app_hdc2080_prepare_temp_threshold_wakeup(&hdc, EXAMPLE_TEMP_DELTA_WAKE_C);
     if (thr_err != ESP_OK)
     {
         ESP_LOGW(TAG, "[HDC2080] Threshold wake prep failed: %s", esp_err_to_name(thr_err));
     }

    /* Start runtime IRQ handling (edge-triggered ISR -> task).
     * This is separate from deep-sleep GPIO wake config.
     */

    xTaskCreate(app_hdc2080_irq_task, "hdc_irq", 4096, &hdc, 5, &hdc2080_irq_task_handle);

    esp_err_t irq_init_err = app_hdc2080_irq_runtime_init();
    if (irq_init_err != ESP_OK)
    {
        ESP_LOGW(TAG, "[HDC2080][IRQ] Runtime IRQ init failed: %s", esp_err_to_name(irq_init_err));
    }

    // Enable last
    gpio_intr_enable(EXAMPLE_HDC2080_IRQ);

    /* Start with display off until first activity */
    app_display_turn_off();
    ESP_LOGI(TAG, "[UI] Display off until activity");

    //------------------------- MAIN LOOP -------------------------

    while (1)
    {
        const TickType_t now = xTaskGetTickCount();

        // =========================================================
        // SLEEP: display off; wait for HDC2080 threshold IRQ edge
        // =========================================================
        if (sleep_state == SLEEP_STATE_SLEEPING)
        {
             vTaskDelay(pdMS_TO_TICKS(20));
             continue;
        }

        // =========================================================
        // ACTIVE: keep display on for 3 seconds, then re-arm + sleep
        // =========================================================
        if (sleep_state == SLEEP_STATE_ACTIVE)
        {
            if (now >= active_until)
            {
                /* Re-arm threshold ONCE when leaving ACTIVE back to SLEEP. */
                if (sensor_connected)
                {
                    (void)app_hdc2080_prepare_temp_threshold_wakeup(&hdc, EXAMPLE_TEMP_DELTA_WAKE_C);
                }

                app_display_turn_off();
                sleep_state = SLEEP_STATE_SLEEPING;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Should never happen, but keeps us robust if state gets corrupted. */
        sleep_state = SLEEP_STATE_SLEEPING;
    }
#endif
}


/* =========================================================
 * RUNTIME DIAGNOSTICS
 * ========================================================= */
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
        x_off = EXAMPLE_LCD_LANDSCAPE_X_OFFSET;
        y_off = EXAMPLE_LCD_LANDSCAPE_Y_OFFSET;
    } else {
        x_off = EXAMPLE_LCD_PORTRAIT_X_OFFSET;
        y_off = EXAMPLE_LCD_PORTRAIT_Y_OFFSET;
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

/* =========================================================
 * HDC2080 TEMP SENSOR
 * ========================================================= */

