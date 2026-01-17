/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Adjusted for:
 *  - LCD: 1.9", 170x320, ST7789V2, SPI
 *  - Touch: CST816, I2C
 *
 * IMPORTANT:
 *  - Pixel clock reduced for stability
 *  - Resolution corrected to portrait native
 *  - Touch driver migrated from TT21100 -> CST816
 *  - Touch axis mapping set to a sane default, may need fine-tuning
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
#include "indev/lv_indev_gesture.h"

/* =========================================================
 * CST816S TOUCH (IRQ-driven read, per component README)
 * ========================================================= */
#include "lvgl.h"

/* =========================================================
 * FIXED ROTATION (debugging touch alignment)
 * ========================================================= */
#define EXAMPLE_DEFAULT_ROTATION   (LV_DISPLAY_ROTATION_90)

/* =========================================================
 * PANEL GAP (RAM OFFSET)
 * =========================================================
 * This 170x320 ST7789 panel needs a RAM window offset ("gap").
 * Verified working configuration:
 *  - Landscape (rotation 90/270): x=0, y=35 (clean image)
 *
 * Note: Some panels need different values for portrait (0/180). If you
 * see a shifted image in portrait, adjust the portrait values below.
 */
#define EXAMPLE_LCD_PORTRAIT_X_OFFSET  (35)
#define EXAMPLE_LCD_PORTRAIT_Y_OFFSET  (0)
#define EXAMPLE_LCD_LANDSCAPE_X_OFFSET (0)
#define EXAMPLE_LCD_LANDSCAPE_Y_OFFSET (35)

/* === TOUCH DRIVER CHANGE === */
#include "esp_lcd_touch_cst816s.h"

/* === LCD native resolution (portrait) === */
#define EXAMPLE_LCD_H_RES   (170)
#define EXAMPLE_LCD_V_RES   (320)

/* === LCD SPI settings === */
/*
 * CONCERN:
 * 40 MHz is often too high for small ST7789 panels.
 * 26 MHz is a conservative, proven value.
 * If you see flicker or corrupted lines -> try 20 MHz.
 */
#define EXAMPLE_LCD_SPI_NUM          (SPI3_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ     (26 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS         (8)
#define EXAMPLE_LCD_PARAM_BITS       (8)
#define EXAMPLE_LCD_BITS_PER_PIXEL   (16)

/* Double buffer is fine */
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)

/*
 * Buffer height chosen to divide 320 cleanly.
 * This avoids partial flush edge cases.
 */
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (40)

/* Backlight logic level */
/*
 * NOTE:
 * Your logs show bl_level=0 while the panel is visibly on.
 * That strongly suggests this panel's backlight is ACTIVE-LOW.
 */
#undef EXAMPLE_LCD_BL_ON_LEVEL
#define EXAMPLE_LCD_BL_ON_LEVEL      (0)

/* === LCD pins (unchanged, assumed correct) === */
/* LCD control pins */
#define EXAMPLE_LCD_GPIO_RST     (GPIO_NUM_9)    // LCD_RST
#define EXAMPLE_LCD_GPIO_SCLK    (GPIO_NUM_10)   // LCD_CLK (SPI SCK)
#define EXAMPLE_LCD_GPIO_DC      (GPIO_NUM_11)   // LCD_DC
#define EXAMPLE_LCD_GPIO_CS      (GPIO_NUM_12)   // LCD_CS
#define EXAMPLE_LCD_GPIO_MOSI    (GPIO_NUM_13)   // LCD_DIN (SPI MOSI)
#define EXAMPLE_LCD_GPIO_BL      (GPIO_NUM_14)   // LCD_BL (backlight)

/* === Touch I2C settings === */
#define EXAMPLE_TOUCH_I2C_NUM        (0)
#define EXAMPLE_TOUCH_I2C_CLK_HZ     (400000)

/* Touch pins */
/* I2C pins */
#define EXAMPLE_TOUCH_I2C_SDA    (GPIO_NUM_47)   // TP_SDA
#define EXAMPLE_TOUCH_I2C_SCL    (GPIO_NUM_48)   // TP_SCL

/* Touch control pins */
#define EXAMPLE_TOUCH_GPIO_INT   (GPIO_NUM_21)   // TP_INT

/*
 * TP_RST is defined as NC in your table.
 * Correct handling: do NOT connect / do NOT drive.
 */
#define EXAMPLE_TOUCH_GPIO_RST   (GPIO_NUM_17)   // TP_RST

static const char *TAG = "MAIN";

/* Forward declarations (C requires prototypes before first use) */
static void app_runtime_diag_task(void *arg);
static esp_err_t app_lcd_apply_gap_for_rotation(lv_disp_rotation_t rot);

/* LVGL image */
LV_IMG_DECLARE(esp_logo)

static bool     s_touch_active = false;
static int32_t  s_touch_x = 0;
static int32_t  s_touch_y = 0;
static uint32_t s_touch_last_tick = 0;

/* How long we keep a touch "alive" without new IRQ (ms) */
#define TOUCH_HOLD_TIME_MS  40


/* Handles */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

/* Current UI/display rotation */
static lv_disp_rotation_t g_disp_rot = EXAMPLE_DEFAULT_ROTATION;

/* Touch visualizer (green dot) */
static lv_obj_t *touch_dot = NULL;
static volatile int32_t g_touch_x = 0;
static volatile int32_t g_touch_y = 0;
static volatile bool g_touch_pressed = false;

static void app_set_rotation(lv_disp_rotation_t rot);
static void app_cycle_rotation(void);
static void app_button_cb(lv_event_t *e);

/* =========================================================
 * DEBUG HELPERS
 * ========================================================= */
static void app_log_heap(const char *where)
{
    ESP_LOGI(TAG, "[HEAP] %s: free=%u, min_free=%u, largest=%u", where,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

static esp_err_t app_check_gpio_valid(const char *name, gpio_num_t gpio)
{
    if (gpio == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "GPIO %s is NC", name);
        return ESP_OK;
    }
    if (!GPIO_IS_VALID_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO %s=%d is not a valid GPIO on this target", name, (int)gpio);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/* Optional: low-level ST7789 command probes. These help distinguish:
 *  - SPI bus not toggling / wrong pins
 *  - CS/DC wiring wrong
 *  - reset/backlight issues
 */
#ifndef EXAMPLE_LCD_DEBUG_PANEL_PROBE
#define EXAMPLE_LCD_DEBUG_PANEL_PROBE  (1)
#endif

#if EXAMPLE_LCD_DEBUG_PANEL_PROBE
static esp_err_t app_st7789_basic_probe(void)
{
    ESP_LOGI(TAG, "[LCD][PROBE] Sending basic ST7789 commands (SWRESET/SLEEPOUT/DISPON)");

    /* SWRESET */
    esp_err_t err = esp_lcd_panel_io_tx_param(lcd_io, 0x01, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD][PROBE] SWRESET failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(120));

    /* SLEEPOUT */
    err = esp_lcd_panel_io_tx_param(lcd_io, 0x11, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD][PROBE] SLEEPOUT failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(120));

    /* DISPON */
    err = esp_lcd_panel_io_tx_param(lcd_io, 0x29, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD][PROBE] DISPON failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "[LCD][PROBE] Basic command probe done");
    return ESP_OK;
}
#endif

/* =========================================================
 * LCD INIT
 * ========================================================= */
static esp_err_t app_lcd_init(void)
{
    ESP_LOGI(TAG, "[LCD] ===== app_lcd_init() =====");
    app_log_heap("before lcd init");

    ESP_RETURN_ON_ERROR(app_check_gpio_valid("LCD_RST", EXAMPLE_LCD_GPIO_RST), TAG, "Invalid LCD_RST GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("LCD_SCLK", EXAMPLE_LCD_GPIO_SCLK), TAG, "Invalid LCD_SCLK GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("LCD_DC", EXAMPLE_LCD_GPIO_DC), TAG, "Invalid LCD_DC GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("LCD_CS", EXAMPLE_LCD_GPIO_CS), TAG, "Invalid LCD_CS GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("LCD_MOSI", EXAMPLE_LCD_GPIO_MOSI), TAG, "Invalid LCD_MOSI GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("LCD_BL", EXAMPLE_LCD_GPIO_BL), TAG, "Invalid LCD_BL GPIO");

    ESP_LOGI(TAG, "[LCD] Config: %dx%d, SPI host=%d, pclk=%u Hz, bpp=%d, buf_h=%d, double_buf=%d",
             EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, EXAMPLE_LCD_SPI_NUM,
             (unsigned)EXAMPLE_LCD_PIXEL_CLK_HZ, EXAMPLE_LCD_BITS_PER_PIXEL,
             EXAMPLE_LCD_DRAW_BUFF_HEIGHT, EXAMPLE_LCD_DRAW_BUFF_DOUBLE);
    ESP_LOGI(TAG, "[LCD] Pins: RST=%d DC=%d CS=%d SCLK=%d MOSI=%d BL=%d",
             EXAMPLE_LCD_GPIO_RST, EXAMPLE_LCD_GPIO_DC, EXAMPLE_LCD_GPIO_CS,
             EXAMPLE_LCD_GPIO_SCLK, EXAMPLE_LCD_GPIO_MOSI, EXAMPLE_LCD_GPIO_BL);

    /* Backlight GPIO */
    ESP_LOGI(TAG, "[LCD] Configuring backlight GPIO");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bk_gpio_config), TAG, "Backlight gpio_config failed");

    /* Keep BL off until panel init succeeds */
    gpio_set_level(EXAMPLE_LCD_GPIO_BL, !EXAMPLE_LCD_BL_ON_LEVEL);

    ESP_LOGI(TAG, "[LCD] Initializing SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES *
                           EXAMPLE_LCD_DRAW_BUFF_HEIGHT *
                           sizeof(uint16_t),
    };
    esp_err_t err = spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "[LCD] Installing panel IO (SPI)");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(err));
        goto err;
    }

    ESP_LOGI(TAG, "[LCD] Installing ST7789 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
#else
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
#endif
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };

    err = esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] esp_lcd_new_panel_st7789 failed: %s", esp_err_to_name(err));
        goto err;
    }

    ESP_LOGI(TAG, "[LCD] Resetting panel");
    err = esp_lcd_panel_reset(lcd_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] esp_lcd_panel_reset failed: %s", esp_err_to_name(err));
        goto err;
    }

    ESP_LOGI(TAG, "[LCD] Initializing panel (vendor init sequence)");
    err = esp_lcd_panel_init(lcd_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] esp_lcd_panel_init failed: %s", esp_err_to_name(err));
        goto err;
    }

#if EXAMPLE_LCD_DEBUG_PANEL_PROBE
    /* If this fails, it often indicates SPI/DC/CS wiring issues. */
    err = app_st7789_basic_probe();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[LCD] Panel command probe failed after init: %s", esp_err_to_name(err));
        /* Continue anyway; probe is diagnostic */
    }
#endif

    ESP_LOGI(TAG, "[LCD] Setting mirror/disp on");
    err = esp_lcd_panel_mirror(lcd_panel, false, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] esp_lcd_panel_mirror failed: %s", esp_err_to_name(err));
        goto err;
    }

    err = esp_lcd_panel_disp_on_off(lcd_panel, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD] esp_lcd_panel_disp_on_off failed: %s", esp_err_to_name(err));
        goto err;
    }

    /* Backlight on (only after panel init succeeds) */
    gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL);
    ESP_LOGI(TAG, "[LCD] Backlight ON");



    app_log_heap("after lcd init");
    return ESP_OK;

err:
    ESP_LOGE(TAG, "[LCD] Init failed, cleaning up");
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
        lcd_panel = NULL;
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
        lcd_io = NULL;
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    app_log_heap("after lcd cleanup");
    return err;
}

/* =========================================================
 * TOUCH INIT (CST816)
 * ========================================================= */

/*** Touch CST816 SETTINGS interupt - I2C will only response after interrupt ***/
static SemaphoreHandle_t touch_mux = NULL;

static void touch_isr_callback(esp_lcd_touch_handle_t tp)
{

    (void)tp;
    if (touch_mux == NULL) {
        return;
    }
    BaseType_t high_task_wake = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &high_task_wake);
    if (high_task_wake) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t app_touch_init(void)
{
    ESP_LOGI(TAG, "[TOUCH] ===== app_touch_init() =====");
    app_log_heap("before touch init");

    ESP_RETURN_ON_ERROR(app_check_gpio_valid("TP_SDA", EXAMPLE_TOUCH_I2C_SDA), TAG, "Invalid TP_SDA GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("TP_SCL", EXAMPLE_TOUCH_I2C_SCL), TAG, "Invalid TP_SCL GPIO");
    ESP_RETURN_ON_ERROR(app_check_gpio_valid("TP_INT", EXAMPLE_TOUCH_GPIO_INT), TAG, "Invalid TP_INT GPIO");

    ESP_LOGI(TAG, "[TOUCH] I2C port=%d clk=%u SDA=%d SCL=%d INT=%d RST=%d",
             EXAMPLE_TOUCH_I2C_NUM, (unsigned)EXAMPLE_TOUCH_I2C_CLK_HZ,
             EXAMPLE_TOUCH_I2C_SDA, EXAMPLE_TOUCH_I2C_SCL,
             EXAMPLE_TOUCH_GPIO_INT, EXAMPLE_TOUCH_GPIO_RST);

    i2c_master_bus_handle_t i2c_handle = NULL;

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = EXAMPLE_TOUCH_I2C_NUM,
        .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
        .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
        .glitch_ignore_cnt = 7,
#endif
        .flags = {
            .enable_internal_pullup = 1,
        },
    };

    esp_err_t err = i2c_new_master_bus(&i2c_config, &i2c_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[TOUCH] i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = EXAMPLE_TOUCH_GPIO_INT,
        .interrupt_callback = touch_isr_callback,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "[TOUCH] Mapping flags: swap_xy=%d mirror_x=%d mirror_y=%d",
             tp_cfg.flags.swap_xy, tp_cfg.flags.mirror_x, tp_cfg.flags.mirror_y);

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = EXAMPLE_TOUCH_I2C_CLK_HZ;

    ESP_LOGI(TAG, "[TOUCH] Creating esp_lcd I2C IO (speed=%u)", (unsigned)tp_io_config.scl_speed_hz);
    err = esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[TOUCH] esp_lcd_new_panel_io_i2c failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "[TOUCH] Creating CST816S touch driver");
    err = esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[TOUCH] esp_lcd_touch_new_i2c_cst816s failed: %s", esp_err_to_name(err));
        return err;
    }

    app_log_heap("after touch init");
    return ESP_OK;
}

/* LVGL input device read callback (IRQ-gated, per CST816S README)
 * - LVGL will call this periodically, but we only read I2C after an interrupt.
 */
static void app_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;

    data->state = LV_INDEV_STATE_RELEASED;

    if (touch_handle == NULL || touch_mux == NULL) {
        return;
    }

    bool got_irq = (xSemaphoreTake(touch_mux, 0) == pdTRUE);
    uint32_t now = lv_tick_get();

    if (got_irq) {
        /* Controller signaled new data is ready */
        if (esp_lcd_touch_read_data(touch_handle) == ESP_OK) {

            esp_lcd_touch_point_data_t points[1];
            uint8_t point_cnt = 0;

            if (esp_lcd_touch_get_data(touch_handle, points, &point_cnt, 1) == ESP_OK &&
                point_cnt > 0) {

                int32_t x = points[0].x;
                int32_t y = points[0].y;

                int32_t tx = x;
                int32_t ty = y;
                /* Cache */
                s_touch_active    = true;
                s_touch_x         = tx;
                s_touch_y         = ty;
                s_touch_last_tick = now;
            } else {
                /* Controller explicitly reports no touch */
                s_touch_active = false;
            }
        }
    }

    /* Sustain touch across LVGL ticks */
    if (s_touch_active &&
        (now - s_touch_last_tick) <= TOUCH_HOLD_TIME_MS) {

        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = s_touch_x;
        data->point.y = s_touch_y;

        g_touch_pressed = true;
        g_touch_x = data->point.x;
        g_touch_y = data->point.y;

        return;
    }

    /* Final release */
    s_touch_active  = false;
    g_touch_pressed = false;
}


static lv_indev_t *app_lvgl_register_touch_manual(lv_display_t *disp)
{
    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        return NULL;
    }

    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, disp);
    lv_indev_set_read_cb(indev, app_lvgl_touch_read_cb);
    return indev;
}

/* =========================================================
 * LVGL INIT
 * ========================================================= */
static esp_err_t app_lvgl_init(void)
{
    ESP_LOGI(TAG, "[LVGL] ===== app_lvgl_init() =====");
    app_log_heap("before lvgl init");

    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 8192,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5
    };
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LVGL] lvgl_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    const size_t buf_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT;
    ESP_LOGI(TAG, "[LVGL] Display cfg: hres=%d vres=%d buf_pixels=%u (buf_h=%d) double_buf=%d",
             EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, (unsigned)buf_pixels,
             EXAMPLE_LCD_DRAW_BUFF_HEIGHT, EXAMPLE_LCD_DRAW_BUFF_DOUBLE);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = buf_pixels,
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };

    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    if (lvgl_disp == NULL) {
        ESP_LOGE(TAG, "[LVGL] lvgl_port_add_disp returned NULL");
        return ESP_FAIL;
    }

    /* Default rotation at boot */
    g_disp_rot = EXAMPLE_DEFAULT_ROTATION;
    ESP_RETURN_ON_ERROR(app_lcd_apply_gap_for_rotation(g_disp_rot), TAG, "set_gap failed");
    lv_disp_set_rotation(lvgl_disp, g_disp_rot);

    ESP_LOGI(TAG, "[LVGL] Display registered: %p", (void *)lvgl_disp);

    /* Register touch manually to avoid lvgl_port_add_touch() re-installing GPIO ISR service */
    lvgl_touch_indev = app_lvgl_register_touch_manual(lvgl_disp);
    if (lvgl_touch_indev == NULL) {
        ESP_LOGE(TAG, "[LVGL] manual touch registration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "[LVGL] Touch input registered: %p", (void *)lvgl_touch_indev);
    app_log_heap("after lvgl init");

    return ESP_OK;
}

/* =========================================================
 * UI
 * ========================================================= */

static void app_touch_dot_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (touch_dot == NULL) {
        return;
    }

    /* Read shared state produced by input callback */
    const bool pressed = g_touch_pressed;

    const int32_t x = g_touch_x;
    const int32_t y = g_touch_y;

    if (!pressed) {
        lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    /* Clamp to display area */
    int32_t h = lv_display_get_horizontal_resolution(lvgl_disp);
    int32_t v = lv_display_get_vertical_resolution(lvgl_disp);

    const int32_t r = 4;
    int32_t cx = x;
    int32_t cy = y;

    if (cx < 0) cx = 0;
    if (cy < 0) cy = 0;
    if (cx > h - 1) cx = h - 1;
    if (cy > v - 1) cy = v - 1;

    lv_obj_clear_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(touch_dot, cx - r, cy - r);
}

static void app_main_display(void)
{
    ESP_LOGI(TAG, "[UI] Building UI");

    lvgl_port_lock(0);

    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *img_logo = lv_img_create(scr);
    lv_img_set_src(img_logo, &esp_logo);
    lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_width(label, EXAMPLE_LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(
        label,
        LV_SYMBOL_BELL "Hello ST7789 + CST816\n"
        LV_SYMBOL_WARNING " Touch dot debug + Rotate "
    );
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    /* Rotate button (cycles 0/90/180/270) */
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn, app_button_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Rotate");
    lv_obj_center(btn_label);

    /* Touch visualization dot */
    touch_dot = lv_obj_create(scr);
    lv_obj_set_size(touch_dot, 8, 8);
    lv_obj_set_style_radius(touch_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(touch_dot, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_bg_opa(touch_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(touch_dot, 0, 0);
    lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);

    /* Update dot ~60 Hz inside LVGL task context */
    lv_timer_create(app_touch_dot_timer_cb, 16, NULL);

    lvgl_port_unlock();
    ESP_LOGI(TAG, "[UI] UI build done");
}

/* =========================================================
 * MAIN
 * ========================================================= */
void app_main(void)
{
    /* Make sure we see our logs immediately */
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "=== app_main start ===");
    app_log_heap("boot");

    /* CST816S README: create semaphore used by interrupt callback */
    touch_mux = xSemaphoreCreateBinary();
    if (touch_mux == NULL) {
        ESP_LOGE(TAG, "[TOUCH] xSemaphoreCreateBinary failed");
        return;
    }

    esp_err_t err = app_lcd_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Runtime diag task helps catch: watchdog resets, heap leaks, or LVGL stalls */
    xTaskCreate(app_runtime_diag_task, "rt_diag", 3072, NULL, 1, NULL);

    err = app_touch_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Touch init failed: %s", esp_err_to_name(err));
        /* Touch isn’t required for pixels. Keep going. */
    }

    ESP_LOGI("LVGL",
         "LVGL version: %d.%d.%d (hex: 0x%06X)",
         LVGL_VERSION_MAJOR,
         LVGL_VERSION_MINOR,
         LVGL_VERSION_PATCH
         );


    err = app_lvgl_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LVGL init failed: %s", esp_err_to_name(err));
        return;
    }



    app_main_display();
    ESP_LOGI(TAG, "=== app_main done (UI created) ===");
}

/* =========================================================
 * RUNTIME DIAGNOSTICS
 * ========================================================= */
static void app_runtime_diag_task(void *arg)
{
    (void)arg;
    while (1) {

        ESP_LOGI(TAG, ".");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static esp_err_t app_lcd_apply_gap_for_rotation(lv_disp_rotation_t rot)
{
    int x_off;
    int y_off;

    if (rot == LV_DISPLAY_ROTATION_90 || rot == LV_DISPLAY_ROTATION_270) {
        x_off = EXAMPLE_LCD_LANDSCAPE_X_OFFSET;
        y_off = EXAMPLE_LCD_LANDSCAPE_Y_OFFSET;
    } else {
        /* LVGL uses 0/180 for portrait on a portrait-native panel */
        x_off = EXAMPLE_LCD_PORTRAIT_X_OFFSET;
        y_off = EXAMPLE_LCD_PORTRAIT_Y_OFFSET;
    }

    ESP_LOGI(TAG, "[LCD] set_gap for rotation=%d -> x=%d y=%d", (int)rot, x_off, y_off);
    return esp_lcd_panel_set_gap(lcd_panel, x_off, y_off);
}

/* =========================================================
 * ROTATION HELPERS
 * ========================================================= */
static void app_set_rotation(lv_disp_rotation_t rot)
{
    /* Defensive: display not ready yet */
    if (lvgl_disp == NULL) {
        return;
    }

    /* Normalize rotation enum (safety for callers) */
    if (rot > LV_DISPLAY_ROTATION_270) {
        rot = LV_DISPLAY_ROTATION_0;
    }

    /*
     * 1) Apply ST7789 RAM window offset ("gap")
     *
     * This is a *panel hardware requirement*, not an LVGL feature.
     * It must be updated whenever orientation changes, otherwise the
     * image will appear shifted or clipped.
     */
    if (app_lcd_apply_gap_for_rotation(rot) != ESP_OK) {
        ESP_LOGW(TAG, "[LCD] set_gap failed for rotation=%d", (int)rot);
    }

    /*
     * 2) Update LVGL display rotation
     *
     * This rotates:
     *  - rendering
     *  - widget layout
     *  - hit-testing
     *
     * LVGL APIs must be serialized with the LVGL task.
     */
    lvgl_port_lock(0);
    lv_disp_set_rotation(lvgl_disp, rot);

    /*
     * 3) Update LVGL input-device rotation
     *
     * CRITICAL:
     * Without this, pointer direction will feel inverted
     * ("plus/minus swapped") after rotation.
     *
     * This tells LVGL how to interpret incoming touch coordinates
     * relative to the rotated display.
     */

    lvgl_port_unlock();

    /*
     * 4) Store current rotation for application-level state
     *
     * Note:
     * Touch code does NOT use this for math anymore.
     * It is only used for cycling / logging.
     */
    g_disp_rot = rot;

    ESP_LOGI(TAG, "[UI] Rotation set to %d", (int)rot);
}

static void app_cycle_rotation(void)
{
    lv_disp_rotation_t next = g_disp_rot;
    switch (g_disp_rot) {
        case LV_DISPLAY_ROTATION_0:
            next = LV_DISPLAY_ROTATION_90;
            break;
        case LV_DISPLAY_ROTATION_90:
            next = LV_DISPLAY_ROTATION_180;
            break;
        case LV_DISPLAY_ROTATION_180:
            next = LV_DISPLAY_ROTATION_270;
            break;
        case LV_DISPLAY_ROTATION_270:
        default:
            next = LV_DISPLAY_ROTATION_0;
            break;
    }
    app_set_rotation(next);
}

static void app_button_cb(lv_event_t *e)
{
    (void)e;
    app_cycle_rotation();
}
