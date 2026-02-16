/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ali-Box Sensor Demo
 *  - LCD: 1.9", 170x320, ST7789V2, SPI
 *  - Touch: CST816S, I2C
 *  - Temp/Humidity: HDC2080, I2C
 *  - IMU: QMI8658, I2C (Wake-on-Motion)
 *  - LVGL 9.4
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include <stdio.h>
#include <inttypes.h>

/* Component helpers */
#include "lcd_helper.h"
#include "touch_helper.h"
#include "imu_helper.h"
#include "hdc2080_helper.h"

static const char *TAG = "MAIN";

/* === Pin definitions === */
#define LCD_GPIO_RST     (GPIO_NUM_9)
#define LCD_GPIO_SCLK    (GPIO_NUM_10)
#define LCD_GPIO_DC      (GPIO_NUM_11)
#define LCD_GPIO_CS      (GPIO_NUM_12)
#define LCD_GPIO_MOSI    (GPIO_NUM_13)
#define LCD_GPIO_BL      (GPIO_NUM_14)

/* === LCD settings === */
#define LCD_H_RES            (170)
#define LCD_V_RES            (320)
#define LCD_SPI_NUM          (SPI3_HOST)
#define LCD_PIXEL_CLK_HZ     (26 * 1000 * 1000)
#define LCD_DRAW_BUFF_HEIGHT (40)
#define LCD_BL_ON_LEVEL      (0)


#define TOUCH_I2C_SDA    (GPIO_NUM_47)
#define TOUCH_I2C_SCL    (GPIO_NUM_48)
#define TOUCH_GPIO_INT   (GPIO_NUM_21)
#define TOUCH_RST_PIN    (GPIO_NUM_17)

#define HDC2080_IRQ      (GPIO_NUM_15)
#define IMU_INT1_GPIO    (GPIO_NUM_8)



/* === I2C settings === */
#define I2C_NUM          (0)
#define I2C_CLK_HZ       (400000)

/* === Wake/sensor settings === */
#define HDC2080_ADDR         (0x40)
#define TEMP_DELTA_WAKE_C    (2.0f)
#define WOM_THRESHOLD_MG     (20)

typedef enum {
    WAKE_BOOT = 0,
    WAKE_TOUCH,
    WAKE_TEMP,
    WAKE_IMU,
    WAKE_OTHER,
} wake_cause_t;

/* Global handles */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static lcd_helper_handle_t s_lcd = {0};
static touch_helper_handle_t s_touch = {0};
static imu_helper_handle_t s_imu = {0};
static hdc2080_helper_handle_t s_hdc = {0};
static lv_display_t *s_lvgl_disp = NULL;
static lv_indev_t *s_lvgl_touch = NULL;

/* UI labels */
static lv_obj_t *s_wake_label = NULL;
static lv_obj_t *s_temp_label = NULL;

/* Forward declarations */
static esp_err_t init_i2c(void);
static esp_err_t init_lcd(void);
static esp_err_t init_touch(void);
static esp_err_t init_lvgl(void);
static esp_err_t init_sensors(void);
static wake_cause_t get_wakeup_cause(void);
static const char *wake_cause_str(wake_cause_t cause);
static void ui_init(void);
static void ui_update(wake_cause_t cause, float temp_c, bool temp_ok);
static bool wait_for_touch(uint32_t timeout_ms);
static void prepare_imu_wom_from_deep_sleep(void);
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);

/* =========================================================
 * I2C INIT
 * ========================================================= */
static void i2c_bus_scan(void)
{
    if (!s_i2c_bus) return;

    ESP_LOGI(TAG, "Scanning I2C bus...");
    /* Temporarily suppress I2C error logs during scan */
    esp_log_level_t old_level = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    uint8_t devices_found = 0;

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_master_dev_handle_t dev_handle;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };

        esp_err_t ret = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev_handle);
        if (ret == ESP_OK) {
            uint8_t dummy;
            ret = i2c_master_receive(dev_handle, &dummy, 1, 100);
            i2c_master_bus_rm_device(dev_handle);

            if (ret == ESP_OK || ret == ESP_ERR_TIMEOUT) {
                ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
                devices_found++;
            }
        }
    }
    esp_log_level_set("i2c.master", old_level);


    ESP_LOGI(TAG, "I2C scan complete: %d device(s) found", devices_found);
}

void touch_i2c_deinit(void)
{
    /* Don't try to delete the I2C bus - device handles are still attached.
     * Deep sleep will reset the I2C hardware peripheral anyway.
     * Just mark as NULL so init_i2c() will reinitialize on wake.
     */
    if (s_i2c_bus) {
        ESP_LOGI(TAG, "Marking I2C bus for re-init on wake (deep sleep will reset hardware)");
        s_i2c_bus = NULL;
    }
}

static esp_err_t init_i2c(void)
{
    /* After deep sleep, all RAM is cleared so s_i2c_bus will be NULL.
     * No need to check wake cause - just proceed with init if NULL.
     */
    if (s_i2c_bus) {
        ESP_LOGI(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    /* Release GPIO holds from deep sleep */
    gpio_hold_dis(TOUCH_I2C_SDA);
    gpio_hold_dis(TOUCH_I2C_SCL);
    gpio_hold_dis(TOUCH_RST_PIN);
    gpio_hold_dis(TOUCH_GPIO_INT);

    /* Configure RST pin FIRST and hold chip in reset during I2C recovery */
    ESP_LOGI(TAG, "Holding CST816S in reset during init...");
    const gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << TOUCH_RST_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));
    gpio_set_level(TOUCH_RST_PIN, 0);  // Hold in reset immediately
    vTaskDelay(pdMS_TO_TICKS(50));

    /* I2C bus recovery - based on Forward Computing method
     * http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
     */
    ESP_LOGI(TAG, "I2C bus recovery starting...");

    gpio_reset_pin(TOUCH_I2C_SDA);
    gpio_reset_pin(TOUCH_I2C_SCL);
    gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_PULLUP_ONLY);
    gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_INPUT);
    gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_INPUT);

    /* Wait for pins to stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Check if SCL is held low */
    if (gpio_get_level(TOUCH_I2C_SCL) == 0) {
        ESP_LOGE(TAG, "I2C bus error: SCL held low - cannot become master");
        return ESP_FAIL;
    }

    /* Check if SDA is held low (stuck transaction) */
    bool sda_low = (gpio_get_level(TOUCH_I2C_SDA) == 0);
    if (sda_low) {
        ESP_LOGW(TAG, "SDA stuck low - attempting recovery...");

        int clock_count = 20; // Max 20 clock pulses (> 2x9 bits)

        while (sda_low && (clock_count > 0)) {
            clock_count--;

            /* Clock SCL low using INPUT->OUTPUT_OD method to avoid driving HIGH */
            gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_FLOATING);  // Remove pullup
            gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_OUTPUT_OD);
            gpio_set_level(TOUCH_I2C_SCL, 0);  // Drive low
            esp_rom_delay_us(10);  // >5us

            /* Release SCL (let pullup bring it high) */
            gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_INPUT);
            gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_PULLUP_ONLY);
            esp_rom_delay_us(10);  // >5us

            /* Wait for SCL to go high (handle clock stretching) */
            int stretch_count = 200;  // 200 * 10ms = 2sec timeout
            while (gpio_get_level(TOUCH_I2C_SCL) == 0 && stretch_count > 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                stretch_count--;
            }

            if (gpio_get_level(TOUCH_I2C_SCL) == 0) {
                ESP_LOGE(TAG, "I2C bus error: SCL held low by clock stretch >2sec");
                return ESP_FAIL;
            }

            /* Check if SDA released */
            sda_low = (gpio_get_level(TOUCH_I2C_SDA) == 0);
            if (!sda_low) {
                ESP_LOGI(TAG, "SDA released after %d clock pulses", 20 - clock_count);
                break;
            }
        }

        if (sda_low) {
            ESP_LOGE(TAG, "I2C bus error: SDA still held low after 20 clocks");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGI(TAG, "SDA is HIGH - bus looks clean");
    }

    /* Send I2C STOP condition to reset all devices */
    ESP_LOGI(TAG, "Sending I2C STOP condition...");

    /* Pull SDA low (START condition) */
    gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_FLOATING);
    gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(TOUCH_I2C_SDA, 0);
    esp_rom_delay_us(10);  // >5us

    /* Release SDA (STOP condition) */
    gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(10);  // >5us

    /* Reset pins to tri-state inputs (default state) */
    gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_FLOATING);
    gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_FLOATING);
    gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_INPUT);
    gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "I2C bus recovery complete");

    /* CST816S power and pin initialization
     * Configure INT pin first to prevent holding chip in boot mode
     */
    ESP_LOGI(TAG, "CST816S initialization starting...");

    /* Configure INT pin as input with pull-up (chip may check this during power-up) */
    const gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << TOUCH_GPIO_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_cfg));
    ESP_LOGI(TAG, "  INT pin configured as input with pull-up");

    /* Wait for power rail to stabilize (critical on cold boot) */
    ESP_LOGI(TAG, "  Waiting 200ms for power rail stabilization...");
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(gpio_config(&rst_cfg));

    /* Try INVERTED reset polarity (active-HIGH) first
     * Some CST816S modules may have inverted reset logic
     */
    ESP_LOGI(TAG, "  Attempting reset with INVERTED polarity (active-HIGH)...");
    ESP_LOGI(TAG, "  Step 1: Set RST LOW (idle state for active-HIGH)");
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "  Step 2: Assert RESET HIGH for 200ms");
    gpio_set_level(TOUCH_RST_PIN, 1);  // Try active-HIGH reset
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "  Step 3: Release RESET (back to LOW)");
    gpio_set_level(TOUCH_RST_PIN, 0);

    ESP_LOGI(TAG, "  Step 4: Waiting 800ms for CST816S boot...");
    vTaskDelay(pdMS_TO_TICKS(800));

    const i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &s_i2c_bus),
                        TAG, "I2C bus init failed");
    ESP_LOGI(TAG, "I2C bus ready");

    /* Scan immediately to verify CST816S appears at 0x15 */
    i2c_bus_scan();

    /* Probe for CST816S at both possible addresses */
    bool cst816s_found = false;
    uint8_t cst816s_addr = 0x15;

    for (uint8_t addr = 0x14; addr <= 0x15; addr++) {
        ESP_LOGI(TAG, "Probing CST816S at address 0x%02X...", addr);
        i2c_master_dev_handle_t test_dev;
        i2c_device_config_t test_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };

        esp_err_t probe_ret = i2c_master_bus_add_device(s_i2c_bus, &test_cfg, &test_dev);
        if (probe_ret == ESP_OK) {
            uint8_t dummy;
            probe_ret = i2c_master_receive(test_dev, &dummy, 1, 100);
            i2c_master_bus_rm_device(test_dev);

            if (probe_ret == ESP_OK || probe_ret == ESP_ERR_TIMEOUT) {
                ESP_LOGI(TAG, "✓ CST816S FOUND at 0x%02X!", addr);
                cst816s_found = true;
                cst816s_addr = addr;
                break;
            } else {
                ESP_LOGW(TAG, "✗ CST816S at 0x%02X NACKed: %s", addr, esp_err_to_name(probe_ret));
            }
        } else {
            ESP_LOGW(TAG, "✗ Failed to add CST816S device at 0x%02X: %s", addr, esp_err_to_name(probe_ret));
        }
    }

    if (!cst816s_found) {
        ESP_LOGE(TAG, "✗✗✗ CST816S NOT FOUND on I2C bus! ✗✗✗");
        ESP_LOGE(TAG, "This indicates a hardware issue:");
        ESP_LOGE(TAG, "  1. Touch controller power issue (VDD not stable)");
        ESP_LOGE(TAG, "  2. Reset pin (GPIO%d) not connected to chip", TOUCH_RST_PIN);
        ESP_LOGE(TAG, "  3. Wrong I2C pins (using GPIO%d/GPIO%d)", TOUCH_I2C_SDA, TOUCH_I2C_SCL);
        ESP_LOGE(TAG, "  4. Touch controller damaged or in boot loader mode");
    } else {
        ESP_LOGI(TAG, "CST816S confirmed at address 0x%02X", cst816s_addr);
    }

    return ESP_OK;
}


/* =========================================================
 * LCD INIT
 * ========================================================= */
static esp_err_t init_lcd(void)
{
    const lcd_helper_config_t cfg = {
        .spi_host = LCD_SPI_NUM,
        .pixel_clk_hz = LCD_PIXEL_CLK_HZ,
        .cmd_bits = 8,
        .param_bits = 8,
        .bits_per_pixel = 16,
        .draw_buff_height = LCD_DRAW_BUFF_HEIGHT,
        .gpio_rst = LCD_GPIO_RST,
        .gpio_sclk = LCD_GPIO_SCLK,
        .gpio_dc = LCD_GPIO_DC,
        .gpio_cs = LCD_GPIO_CS,
        .gpio_mosi = LCD_GPIO_MOSI,
        .gpio_bl = LCD_GPIO_BL,
        .bl_on_level = LCD_BL_ON_LEVEL,
        .h_res = LCD_H_RES,
        .v_res = LCD_V_RES,
        .portrait_x_offset = 35,
        .portrait_y_offset = 0,
        .landscape_x_offset = 0,
        .landscape_y_offset = 35,
    };

    return lcd_helper_init(&cfg, &s_lcd);
}

/* =========================================================
 * TOUCH INIT
 * ========================================================= */
static esp_err_t init_touch(void)
{
    const touch_helper_config_t cfg = {
        .i2c_bus = s_i2c_bus,
        .i2c_clk_hz = I2C_CLK_HZ,
        .gpio_int = TOUCH_GPIO_INT,
        .gpio_rst = TOUCH_RST_PIN,
        .rst_active_level = 0,
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .skip_hw_reset = true,  // Already done in init_i2c()
    };

    return touch_helper_init(&cfg, &s_touch);
}

/* =========================================================
 * SENSORS INIT (HDC2080 + IMU)
 * ========================================================= */
static esp_err_t init_sensors(void)
{
    /* HDC2080 */
    const hdc2080_helper_config_t hdc_cfg = {
        .i2c_bus = s_i2c_bus,
        .i2c_addr = HDC2080_ADDR,
        .gpio_irq = HDC2080_IRQ,
    };
    esp_err_t ret = hdc2080_helper_init(&hdc_cfg, &s_hdc);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HDC2080 init failed (optional)");
    }

    /* IMU */
    const imu_helper_config_t imu_cfg = {
        .i2c_bus = s_i2c_bus,
        .gpio_int1 = IMU_INT1_GPIO,
        .wom_threshold_mg = WOM_THRESHOLD_MG,
        .wom_blanking_samples = 3,
    };
    ret = imu_helper_init(&imu_cfg, &s_imu);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed (optional)");
    }

    return ESP_OK;
}

/* =========================================================
 * LVGL INIT
 * ========================================================= */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    data->state = LV_INDEV_STATE_RELEASED;

    int32_t x, y;
    if (touch_helper_read(&s_touch, &x, &y)) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
}

static esp_err_t init_lvgl(void)
{
    ESP_LOGI(TAG, "LVGL v%d.%d.%d init", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);

    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 8192,
        .timer_period_ms = 5,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = s_lcd.io,
        .panel_handle = s_lcd.panel,
        .buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = 1,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags.buff_dma = true,
        .flags.swap_bytes = true,
    };
    s_lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Apply 90° landscape rotation */
    lcd_helper_apply_gap_for_rotation(&s_lcd, 90);
    lv_display_set_rotation(s_lvgl_disp, LV_DISPLAY_ROTATION_90);

    /* Touch input */
    s_lvgl_touch = lv_indev_create();
    lv_indev_set_type(s_lvgl_touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(s_lvgl_touch, s_lvgl_disp);
    lv_indev_set_read_cb(s_lvgl_touch, lvgl_touch_read_cb);

    ESP_LOGI(TAG, "LVGL initialized");
    return ESP_OK;
}

/* =========================================================
 * WAKEUP CAUSE
 * ========================================================= */
static wake_cause_t get_wakeup_cause(void)
{
    esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();

    if (wc == ESP_SLEEP_WAKEUP_UNDEFINED) {
        return WAKE_BOOT;
    }

    if (wc == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t mask = esp_sleep_get_ext1_wakeup_status();
        if (mask & (1ULL << TOUCH_GPIO_INT)) return WAKE_TOUCH;
        if (mask & (1ULL << HDC2080_IRQ)) return WAKE_TEMP;
        if (mask & (1ULL << IMU_INT1_GPIO)) return WAKE_IMU;
    }

    return WAKE_OTHER;
}

static const char *wake_cause_str(wake_cause_t cause)
{
    switch (cause) {
        case WAKE_BOOT:  return "Boot";
        case WAKE_TOUCH: return "Touch";
        case WAKE_TEMP:  return "Temperature";
        case WAKE_IMU:   return "Motion (IMU)";
        default:         return "Other";
    }
}

/* =========================================================
 * UI
 * ========================================================= */
static void ui_init(void)
{
    lvgl_port_lock(0);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Ali-Box Sensor");
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    s_wake_label = lv_label_create(scr);
    lv_label_set_text(s_wake_label, "Wake: --");
    lv_obj_set_style_text_align(s_wake_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_wake_label, LV_ALIGN_TOP_MID, 0, 35);

    s_temp_label = lv_label_create(scr);
    lv_label_set_text(s_temp_label, "Temp: --.-- C");
    lv_obj_set_style_text_align(s_temp_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_temp_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "Touch to sleep...");
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);

    lvgl_port_unlock();
}

static void ui_update(wake_cause_t cause, float temp_c, bool temp_ok)
{
    if (!s_wake_label || !s_temp_label) return;

    char buf[64];
    snprintf(buf, sizeof(buf), "Wake: %s", wake_cause_str(cause));

    lvgl_port_lock(0);
    lv_label_set_text(s_wake_label, buf);

    if (temp_ok) {
        snprintf(buf, sizeof(buf), "Temp: %.1f C", (double)temp_c);
    } else {
        snprintf(buf, sizeof(buf), "Temp: (sensor error)");
    }
    lv_label_set_text(s_temp_label, buf);
    lvgl_port_unlock();
}

/* =========================================================
 * WAIT FOR TOUCH
 * ========================================================= */
static bool wait_for_touch(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout) {
        lv_indev_state_t st = lv_indev_get_state(s_lvgl_touch);
        if (st == LV_INDEV_STATE_PRESSED) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

/* =========================================================
 * DEEP SLEEP PREPARATION (IMU wake only)
 * ========================================================= */
static void prepare_imu_wom_from_deep_sleep(void)
{

    /* Arm IMU Wake-on-Motion */
    if (s_imu.initialized) {
        imu_helper_arm_wom(&s_imu);
        /* Clear status to reset INT line before arming EXT1 */
        imu_helper_clear_wom_status(&s_imu);
    }

    /* Configure EXT1 wake source (IMU INT1 only) */
    uint64_t wake_mask = (1ULL << IMU_INT1_GPIO);

    /* Ensure wake pin is input with pull-up */
    const gpio_config_t wake_cfg = {
        .pin_bit_mask = wake_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&wake_cfg));

    /* Disable all wake sources first, then enable EXT1 */
    ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW));
    ESP_LOGI(TAG, "Deep sleep prepared: EXT1 mask=0x%016" PRIx64 " (IMU only)", wake_mask);
}

/* =========================================================
 * TOUCH IRQ HANDLING FOR APP_MAIN LOOP
 * ========================================================= */
static SemaphoreHandle_t s_touch_irq_sem = NULL;

static void IRAM_ATTR touch_irq_isr(void *arg)
{
    (void)arg;
    if (s_touch_irq_sem) {
        BaseType_t high_task_wake = pdFALSE;
        xSemaphoreGiveFromISR(s_touch_irq_sem, &high_task_wake);
        if (high_task_wake) portYIELD_FROM_ISR();
    }
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "\n\n========== CST816S + HDC2080 + QMI8658 + IMU WAKE TEST ==========\n");

    int loop_count = 0;

    while (1) {
        loop_count++;
        ESP_LOGI(TAG, "\n\n========== ITERATION %d ==========\n", loop_count);



        /* ===== STEP 1: INIT I2C ===== */
        ESP_LOGI(TAG, "[1/5] INIT I2C BUS");

        gpio_hold_dis(TOUCH_I2C_SDA);
        gpio_hold_dis(TOUCH_I2C_SCL);
        gpio_hold_dis(TOUCH_RST_PIN);
        gpio_hold_dis(TOUCH_GPIO_INT);
        gpio_hold_dis(IMU_INT1_GPIO);

        const gpio_config_t rst_cfg = {
            .pin_bit_mask = 1ULL << TOUCH_RST_PIN,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&rst_cfg));
        gpio_set_level(TOUCH_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_reset_pin(TOUCH_I2C_SDA);
        gpio_reset_pin(TOUCH_I2C_SCL);
        gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_PULLUP_ONLY);
        gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_INPUT);
        gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_INPUT);
        vTaskDelay(pdMS_TO_TICKS(10));

        const gpio_config_t int_cfg = {
            .pin_bit_mask = 1ULL << TOUCH_GPIO_INT,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&int_cfg));

        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_set_level(TOUCH_RST_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(TOUCH_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "  ✓ I2C bus ready, CST816S reset complete\n");

        // Create I2C master bus
        // NOTE: Consider adding a bounded timeout in production builds; this demo uses -1 (wait forever)
        // in some I2C calls below. A stuck bus can block forever.
        const i2c_master_bus_config_t i2c_cfg = {
            .i2c_port = I2C_NUM,
            .sda_io_num = TOUCH_I2C_SDA,
            .scl_io_num = TOUCH_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags.enable_internal_pullup = true,
        };

        i2c_master_bus_handle_t i2c_bus = NULL;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

        // Add CST816S touch device
        i2c_master_dev_handle_t cst_dev = NULL;
        i2c_device_config_t cst_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x15,
            .scl_speed_hz = 100000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &cst_cfg, &cst_dev));

        // Add HDC2080 device
        i2c_master_dev_handle_t hdc_dev = NULL;
        i2c_device_config_t hdc_cfg_i2c = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x40,  // HDC2080 address
            .scl_speed_hz = 100000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &hdc_cfg_i2c, &hdc_dev));

        // Configure IMU INT1 pin (wake source)
        const gpio_config_t imu_int_cfg = {
            .pin_bit_mask = 1ULL << IMU_INT1_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&imu_int_cfg));

        // ...initialize QMI8658 IMU with qmi8658_init...
        qmi8658_dev_t qmi_dev = {0};
        esp_err_t imu_ret = qmi8658_init(&qmi_dev, i2c_bus, 0x6B);  // Try primary address
        bool imu_found = (imu_ret == ESP_OK);
        if (!imu_found) {
            imu_ret = qmi8658_init(&qmi_dev, i2c_bus, 0x6A);  // Try alternate address
            imu_found = (imu_ret == ESP_OK);
        }

        if (imu_found) {
            ESP_LOGI(TAG, "  ✓ QMI8658 initialized");
        } else {
            ESP_LOGW(TAG, "  ⚠ QMI8658 NOT FOUND on I2C bus");
        }
        ESP_LOGI(TAG, "  ✓ All devices added to I2C bus\n");

        /* ===== STEP 1b: INIT LCD & LVGL ===== */
        ESP_LOGI(TAG, "[1b/5] INIT LCD & LVGL");
        // Logging suggestion (keep bring-up easy): after init, it's helpful to log the key handles are non-NULL:
        //  - s_lcd.panel, s_lcd.io (inside lcd_helper_handle_t)
        //  - s_lvgl_disp, s_lvgl_touch

        ESP_ERROR_CHECK(init_lcd());
        vTaskDelay(pdMS_TO_TICKS(10));

        ESP_ERROR_CHECK(init_lvgl());
        vTaskDelay(pdMS_TO_TICKS(10));

        ESP_LOGI(TAG, "  ✓ Display initialized\n");

        /* ===== STEP 2: READ SENSORS ===== */
        ESP_LOGI(TAG, "[2/5] READ SENSORS");


        // Read CST816S ID
        uint8_t id_reg = 0xA7;
        uint8_t id_val = 0;
        esp_err_t ret = i2c_master_transmit_receive(cst_dev, &id_reg, 1, &id_val, 1, -1);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ CST816S ID = 0x%02X", id_val);
        } else {
            ESP_LOGE(TAG, "  ✗ CST816S READ FAILED: %s", esp_err_to_name(ret));
        }

        // Read HDC2080 temperature
        float temp_c = 0.0f;
        bool temp_ok = false;
        uint8_t meas_cfg_reg = 0x0F;  // MEAS_CFG register
        uint8_t meas_cfg_val = 0x01;  // Trigger measurement (MEAS_TRIG bit)
        ret = i2c_master_transmit(hdc_dev, (uint8_t[]){meas_cfg_reg, meas_cfg_val}, 2, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "  ✗ HDC2080 TRIG FAILED: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Wait for measurement

        // Read temperature from HDC2080 (TEMP_L = 0x00)
        uint8_t temp_l_reg = 0x00;
        uint8_t temp_data[2];
        ret = i2c_master_transmit_receive(hdc_dev, &temp_l_reg, 1, temp_data, 2, -1);
        if (ret == ESP_OK) {
            uint16_t raw_temp = ((uint16_t)temp_data[1] << 8) | temp_data[0];
            temp_c = ((float)raw_temp * 165.0f / 65536.0f) - 40.5f;
            temp_ok = true;
            ESP_LOGI(TAG, "  ✓ HDC2080 TEMP = %.2f °C (raw=0x%04X)", temp_c, raw_temp);
        } else {
            ESP_LOGE(TAG, "  ✗ HDC2080 READ FAILED: %s", esp_err_to_name(ret));
        }

        // Read QMI8658 IMU acceleration
        if (imu_found) {
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            ret = qmi8658_read_accel(&qmi_dev, &ax, &ay, &az);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "  ✓ QMI8658 ACCEL = (%.2f, %.2f, %.2f) g", ax, ay, az);
            } else {
                ESP_LOGE(TAG, "  ✗ QMI8658 READ FAILED: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "  ⚠ QMI8658 NOT FOUND on I2C bus");
        }
        ESP_LOGI(TAG, "");

        /* ===== STEP 2b: SHOW TEMPERATURE AND WAKEUP REASON ON DISPLAY ===== */
        ESP_LOGI(TAG, "[2b/5] DISPLAY TEMPERATURE");
        ui_init();
        ui_update(WAKE_OTHER, temp_c, temp_ok);
        ESP_LOGI(TAG, "  ✓ UI updated with sensor data\n");

        /* Wait for CST816S touch IRQ */
        if (!s_touch_irq_sem) {
            s_touch_irq_sem = xSemaphoreCreateBinary();
            esp_err_t isr_ret = gpio_install_isr_service(0);
            ESP_LOGI(TAG, "GPIO ISR service install: %s", esp_err_to_name(isr_ret));
        }

        /* Configure INT pin interrupt on falling edge */
        ESP_LOGI(TAG, "Touch INT level before enable: %d", gpio_get_level(TOUCH_GPIO_INT));
        ESP_ERROR_CHECK(gpio_set_intr_type(TOUCH_GPIO_INT, GPIO_INTR_NEGEDGE));
        ESP_ERROR_CHECK(gpio_isr_handler_add(TOUCH_GPIO_INT, touch_irq_isr, NULL));
        ESP_ERROR_CHECK(gpio_intr_enable(TOUCH_GPIO_INT));

        ESP_LOGI(TAG, "Waiting for CST816S touch IRQ...");
        // NOTE: If this blocks forever, either:
        xSemaphoreTake(s_touch_irq_sem, portMAX_DELAY);
        gpio_isr_handler_remove(TOUCH_GPIO_INT);
        ESP_LOGI(TAG, "Touch IRQ received!\n");


        //==========================================================================================================
        // DEEP SLEEP PREPARATION
        //==========================================================================================================


        /* ===== STEP 3: SEND Touch CST816S TO SLEEP ===== */
        ESP_LOGI(TAG, "[3/5] SEND CST816S Touch TO SLEEP");
        // CST816S sleep command: 0xA5 0x03
        // Best-effort: if the touch controller brownouts or is already asleep, this can fail.
        uint8_t sleep_cmd[2] = {0xA5, 0x03};
        ret = i2c_master_transmit(cst_dev, sleep_cmd, 2, -1);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ Sleep command sent");
        } else {
            ESP_LOGE(TAG, "  ✗ FAILED: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "[3/5] SEND Display & Backlight TO SLEEP");
        // Power intent:
        //  - Backlight GPIO is driven to OFF and held across deep sleep.
        //  - Panel is asked to go to sleep via esp_lcd panel ops.
        // Note: esp_lcd_* calls here are cast to (void); for bring-up you may want to log their return values.
        gpio_set_level(s_lcd.gpio_bl, !s_lcd.bl_on_level);
        gpio_hold_en(s_lcd.gpio_bl);

        // Bring-up visibility: capture and log return values.
        // (We still proceed to deep sleep either way; logging only.)
        esp_err_t lcd_off_ret = esp_lcd_panel_disp_on_off(s_lcd.panel, false);
        if (lcd_off_ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ esp_lcd_panel_disp_on_off(false) OK");
        } else {
            ESP_LOGW(TAG, "  ⚠ esp_lcd_panel_disp_on_off(false) failed: %s", esp_err_to_name(lcd_off_ret));
        }

        esp_err_t lcd_sleep_ret = esp_lcd_panel_disp_sleep(s_lcd.panel, true);
        if (lcd_sleep_ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ esp_lcd_panel_disp_sleep(true) OK");
        } else {
            ESP_LOGW(TAG, "  ⚠ esp_lcd_panel_disp_sleep(true) failed: %s", esp_err_to_name(lcd_sleep_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_LOGI(TAG, "Entered sleep mode");


        /* ===== STEP 4: SETUP IMU WAKE-ON-MOTION (while I2C is still active) ===== */
        ESP_LOGI(TAG, "[4/5] SETUP IMU WAKE-ON-MOTION");

        const qmi8658_wom_config_t wom_cfg = {
            .int_pin = QMI8658_INT_PIN1,
            .initial_level = QMI8658_WOM_INITIAL_LEVEL_HIGH,
            .threshold_mg = 20,  // WoM threshold in mg
            .blanking_samples = 3,
        };

        esp_err_t wom_ret = qmi8658_enable_wake_on_motion_cfg(&qmi_dev, &wom_cfg);
        if (wom_ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ WoM configured (threshold=20 mg)");
        } else {
            ESP_LOGE(TAG, "  ✗ WoM config failed: %s", esp_err_to_name(wom_ret));
        }

        // Clear WoM status to reset INT line
        qmi8658_wom_clear_status(&qmi_dev);
        ESP_LOGI(TAG, "  ✓ WoM status cleared, INT pin reset\n");

        /* ===== STEP 5: CLEANUP I2C ===== */
        ESP_LOGI(TAG, "[5/5] CLEANUP I2C AND PREPARE SLEEP");

        i2c_master_bus_rm_device(cst_dev);
        i2c_master_bus_rm_device(hdc_dev);

        if (imu_found && qmi_dev.dev_handle != NULL) {
            i2c_master_bus_rm_device(qmi_dev.dev_handle);
        }

        ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));

        // Configure EXT1 wake source (IMU INT1 only, active-LOW)
        // EXT1 notes:
        //  - EXT1 uses one shared polarity for all pins in the mask.
        //  - Print the wake mask and the pin level right before sleep; it saves a ton of time.
        uint64_t wake_mask = (1ULL << IMU_INT1_GPIO);
        const gpio_config_t wake_cfg = {
            .pin_bit_mask = wake_mask,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&wake_cfg));

        ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW));
        ESP_LOGI(TAG, "  EXT1 wake configured: GPIO%d (IMU INT1) ANY_LOW", (int)IMU_INT1_GPIO);

        // Prepare PINs for deep sleep:
        //  - Touch reset held low (keeps touch IC quiescent)
        //  - I2C pins high-Z + floating, then held (avoid pulling against external devices)
        //  - gpio_deep_sleep_hold_en() latches digital pad holds through deep sleep
        // If you still see leakage, consider reviewing which pins are RTC vs digital pads.
        gpio_set_direction(TOUCH_RST_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(TOUCH_RST_PIN, 0);

        gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_INPUT);
        gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_INPUT);
        gpio_set_direction(TOUCH_GPIO_INT, GPIO_MODE_INPUT);

        gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_FLOATING);
        gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_FLOATING);
        gpio_set_pull_mode(TOUCH_GPIO_INT, GPIO_FLOATING);

        gpio_hold_en(TOUCH_RST_PIN);
        gpio_hold_en(TOUCH_I2C_SDA);
        gpio_hold_en(TOUCH_I2C_SCL);
        gpio_hold_en(TOUCH_GPIO_INT);

        // Ensure GPIO hold is latched during deep sleep (required for digital GPIO hold)
        gpio_deep_sleep_hold_en();

        ESP_LOGI(TAG, "Entering deep sleep...");
        // Final sanity logs can help diagnose instant-wake loops.
        // (No behavior changes; just visibility.)
        ESP_LOGI(TAG, "Final pin levels before sleep: IMU_INT1=%d TOUCH_INT=%d SDA=%d SCL=%d RST=%d",
                 gpio_get_level(IMU_INT1_GPIO),
                 gpio_get_level(TOUCH_GPIO_INT),
                 gpio_get_level(TOUCH_I2C_SDA),
                 gpio_get_level(TOUCH_I2C_SCL),
                 gpio_get_level(TOUCH_RST_PIN));
        esp_deep_sleep_start();
        /* Code will NOT reach here - ESP will wake on IMU motion */
    }
}

#ifdef no_used
/* =========================================================
 * APP MAIN
 * ========================================================= */
void app_main_old(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "=== Ali-Box Sensor Demo ===");

    /* 1) Print wakeup reason */
    wake_cause_t wake = get_wakeup_cause();
    ESP_LOGI(TAG, "Wakeup reason: %s", wake_cause_str(wake));

    /* Initialize peripherals:
     * - I2C bus first (with GPIO hold release and touch HW reset)
     * - Touch init FIRST (CST816S needs undisturbed boot time)
     * - Then scan I2C bus to verify all devices
     */

    ESP_ERROR_CHECK(init_i2c());
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_ERROR_CHECK(init_touch());
    vTaskDelay(pdMS_TO_TICKS(50));


    ESP_ERROR_CHECK(init_lcd());
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_ERROR_CHECK(init_lvgl());
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_ERROR_CHECK(init_sensors());
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Clear IMU WoM status if woke from IMU */
    if (wake == WAKE_IMU && s_imu.initialized) {
        imu_helper_clear_wom_status(&s_imu);
    }

    /* 2) Read temperature */
    float temp_c = 0.0f;
    bool temp_ok = false;
    if (s_hdc.initialized) {
        if (hdc2080_helper_read_temp(&s_hdc, &temp_c, NULL) == ESP_OK) {
            temp_ok = true;
            ESP_LOGI(TAG, "Temperature: %.1f C", (double)temp_c);
        } else {
            ESP_LOGW(TAG, "Failed to read temperature");
        }
    }

    /* 3) Show temperature and wakeup reason on display */
    ui_init();
    ui_update(wake, temp_c, temp_ok);

    /* 4) Wait until user touches the display */
    ESP_LOGI(TAG, "Waiting for touch...");
    while (!wait_for_touch(60000)) {
        /* Keep waiting (timeout just for safety) */
        ESP_LOGI(TAG, "Still waiting for touch...");
    }
    ESP_LOGI(TAG, "Touch detected!");

    /* Small delay for visual feedback */
    vTaskDelay(pdMS_TO_TICKS(200));

     /******************************+  5) When touched: prepare deep sleep *********************************************
     *
     */
    ESP_LOGI(TAG, "Preparing for deep sleep (IMU wake only)...");
    prepare_imu_wom_from_deep_sleep();

    /* Disable touch runtime IRQ */
    touch_helper_disable_irq(&s_touch);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Put touch controller to sleep */
    touch_helper_enter_sleep(&s_touch);

    /* Hold touch controller in reset (optional, but safe). */
    gpio_set_direction(TOUCH_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TOUCH_RST_PIN, 0);
    gpio_hold_en(TOUCH_RST_PIN);


    /* Turn off LCD */
    lcd_helper_enter_sleep(&s_lcd);

    /* Make I2C + INT pins high-Z (input, floating). */
    gpio_set_direction(TOUCH_I2C_SDA, GPIO_MODE_INPUT);
    gpio_set_direction(TOUCH_I2C_SCL, GPIO_MODE_INPUT);
    gpio_set_direction(TOUCH_GPIO_INT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TOUCH_I2C_SDA, GPIO_FLOATING);
    gpio_set_pull_mode(TOUCH_I2C_SCL, GPIO_FLOATING);
    gpio_set_pull_mode(TOUCH_GPIO_INT, GPIO_FLOATING);
    gpio_hold_en(TOUCH_I2C_SDA);
    gpio_hold_en(TOUCH_I2C_SCL);
    gpio_hold_en(TOUCH_GPIO_INT);

    /* Let logs flush */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 6) Go into deep sleep */
    ESP_LOGI(TAG, "Entering deep sleep...");
    esp_deep_sleep_start();
    ESP_LOGI(TAG, "This will never be printed - ESP is now sleeping and will wake on IMU motion");
}
#endif
/* =========================================================
 * UNUSED FUNCTIONS (kept for future use)
 * ========================================================= */

/* NOTE: app_runtime_diag_task - diagnostic task for debugging, not used in current flow */
/*
static void app_runtime_diag_task(void *arg)
{
    (void)arg;
    while (1) {
        ESP_LOGI(TAG, ".");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
*/

/* NOTE: app_main_display - touch demo UI with rotation button, not used in current flow */
/*
static void app_main_display(void)
{
    // Demo UI with logo, touch dot, and rotation button
}
*/

/* NOTE: app_touch_dot_update - touch dot visualizer, not used in current flow */
/*
static void app_touch_dot_update(int32_t x, int32_t y, bool pressed)
{
    // Update touch dot position
}
*/

/* NOTE: app_note_activity - activity tracker for display timeout, not used in current flow */
/*
static void app_note_activity(wake_cause_t cause, TickType_t *last_activity_ticks)
{
    // Track activity for display timeout
}
*/

/* NOTE: app_configure_gpio_wakeup_sources - GPIO wake config (replaced by prepare_deep_sleep_imu_only) */
/*
static void app_configure_gpio_wakeup_sources(void)
{
    // Configure touch + HDC2080 as GPIO wake sources
}
*/

/* NOTE: app_prepare_for_deep_sleep - legacy sleep prep (replaced by prepare_deep_sleep_imu_only) */
/*
static void app_prepare_for_deep_sleep(void)
{
    // Legacy deep sleep preparation
}
*/

/* NOTE: app_power_save_shutdown_and_sleep_ext1 - unified power save routine, not used in current flow */
/*
static void app_power_save_shutdown_and_sleep_ext1(bool keep_touch_wake, bool keep_hdc_wake)
{
    // Full power save with optional touch/HDC wake sources
}
*/

/* NOTE: app_wake_debug_dump_ext1 - debug EXT1 wake status, not used in current flow */
/*
static void app_wake_debug_dump_ext1(void)
{
    // Debug: dump EXT1 wake mask and pin levels
}
*/

/* NOTE: app_sleep_debug_dump_pins - debug pin levels before sleep, not used in current flow */
/*
static void app_sleep_debug_dump_pins(void)
{
    // Debug: dump wake pin levels
}
*/

/* NOTE: app_hdc2080_init - full HDC2080 init with temp threshold arming, not used in current flow */
/*
static esp_err_t app_hdc2080_init(void)
{
    // Full HDC2080 init with temperature threshold arming
}
*/

/* NOTE: app_qmi8658_init_optional - legacy IMU init, replaced by imu_helper_init */
/*
static esp_err_t app_qmi8658_init_optional(void)
{
    // Legacy QMI8658 init
}
*/
