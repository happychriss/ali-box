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

/* Image asset */
LV_IMG_DECLARE(esp_logo);

/* =========================================================
 * CST816S TOUCH (IRQ-driven read, per component README)
 * ========================================================= */
#include "esp_lcd_touch_cst816s.h"

static const char *TAG = "MAIN";

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

/* Touch visualizer (green dot) */
static lv_obj_t *touch_dot = NULL;
static lv_timer_t *touch_dot_timer = NULL;

static void app_touch_dot_update(int32_t x, int32_t y, bool pressed);
static void app_touch_dot_timer_cb(lv_timer_t *t);

/* Handles */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

/* IRQ semaphore for CST816S (per component README) */
static SemaphoreHandle_t touch_irq_sem = NULL;

/* Forward declarations */
static esp_err_t app_lcd_init(void);
static esp_err_t app_touch_init(void);
static esp_err_t app_lvgl_init(void);
static void app_main_display(void);
static void app_runtime_diag_task(void *arg);
static esp_err_t app_lcd_apply_gap_for_rotation(lv_display_rotation_t rot);
static void app_apply_rotation(lv_display_rotation_t rot);

/* =========================================================
 * TOUCH ISR CALLBACK (CST816S README)
 * ========================================================= */
static void touch_isr_callback(esp_lcd_touch_handle_t tp)
{
    (void)tp;
    if (touch_irq_sem) {
        BaseType_t high_task_wake = pdFALSE;
        xSemaphoreGiveFromISR(touch_irq_sem, &high_task_wake);
        if (high_task_wake) {
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
static void app_lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    data->state = LV_INDEV_STATE_RELEASED;

    if (!touch_handle || !touch_irq_sem) {
        return;
    }

    /* Only read I2C after IRQ (CST816S responds briefly post-touch) */
    if (xSemaphoreTake(touch_irq_sem, 0) == pdTRUE) {
        esp_lcd_touch_read_data(touch_handle);

        esp_lcd_touch_point_data_t points[1];
        uint8_t point_cnt = 0;

        if (esp_lcd_touch_get_data(touch_handle, points, &point_cnt, 1) == ESP_OK &&
            point_cnt > 0) {
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

    /* I2C bus */
    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = EXAMPLE_TOUCH_I2C_NUM,
        .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
        .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &i2c_handle));

    /* Touch config (RAW panel-native, LVGL rotates automatically) */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = EXAMPLE_TOUCH_GPIO_INT,
        .interrupt_callback = touch_isr_callback,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },  /* No manual rotation */
    };

    esp_lcd_panel_io_handle_t tp_io_handle;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = EXAMPLE_TOUCH_I2C_CLK_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle));
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

    /* Default: 90° landscape (your preference) */
    app_apply_rotation(LV_DISPLAY_ROTATION_90);

    /* Touch input device */
    lvgl_touch_indev = lv_indev_create();
    lv_indev_set_type(lvgl_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(lvgl_touch_indev, lvgl_disp);
    lv_indev_set_read_cb(lvgl_touch_indev, app_lvgl_touch_read_cb);

    /*
     * Touch-dot updater:
     * The indev read callback receives/stores RAW coordinates.
     * To draw in the rotated UI coordinate system, query LVGL for the processed
     * pointer point and draw from that.
     */
    touch_dot_timer = lv_timer_create(app_touch_dot_timer_cb, 16, NULL);

    return ESP_OK;
}

/* =========================================================
 * UI
 * ========================================================= */
static void app_button_cb(lv_event_t *e)
{
    (void)e;
    lv_display_rotation_t rot = lv_display_get_rotation(lvgl_disp);
    rot = (rot + 1) % 4;  /* 0→1→2→3→0 */
    app_apply_rotation(rot);
    ESP_LOGI(TAG, "[UI] Rotation → %d", (int)rot);
}

static void app_main_display(void)
{
    lvgl_port_lock(0);

    lv_obj_t *scr = lv_scr_act();

    /* Logo */
    lv_obj_t *logo = lv_img_create(scr);
    lv_img_set_src(logo, &esp_logo);
    lv_obj_align(logo, LV_ALIGN_TOP_MID, 0, 20);

    /* Status */
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "ST7789 + CST816S + LVGL 9.4\nTouch anywhere (green dot)\nRotate button");
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    /* Rotate button */
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn, app_button_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_lbl = lv_label_create(btn);
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
 * MAIN
 * ========================================================= */
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);


#ifdef TOUCH_DISPLAY_DEMO

    touch_irq_sem = xSemaphoreCreateBinary();
    if (!touch_irq_sem) {
        ESP_LOGE(TAG, "Failed to create touch semaphore");
        return;
    }

    if (app_lcd_init() != ESP_OK) return;
    if (app_touch_init() != ESP_OK) return;
    if (app_lvgl_init() != ESP_OK) return;

    xTaskCreate(app_runtime_diag_task, "diag", 3072, NULL, 1, NULL);
    app_main_display();
#endif  // TOUCH_DISPLAY_DEMO
}


/* =========================================================
 * RUNTIME DIAGNOSTICS
 * ========================================================= */
static void app_runtime_diag_task(void *arg)
{
    (void)arg;
    while (1) {
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
