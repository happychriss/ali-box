#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* SPI settings */
    int spi_host;
    int pixel_clk_hz;
    int cmd_bits;
    int param_bits;
    int bits_per_pixel;
    int draw_buff_height;

    /* Pins */
    gpio_num_t gpio_rst;
    gpio_num_t gpio_sclk;
    gpio_num_t gpio_dc;
    gpio_num_t gpio_cs;
    gpio_num_t gpio_mosi;
    gpio_num_t gpio_bl;
    int bl_on_level;

    /* Resolution (native portrait) */
    int h_res;
    int v_res;

    /* Panel RAM gap offsets */
    int portrait_x_offset;
    int portrait_y_offset;
    int landscape_x_offset;
    int landscape_y_offset;
} lcd_helper_config_t;

typedef struct {
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    gpio_num_t gpio_bl;
    int bl_on_level;
    int h_res;
    int v_res;
    int portrait_x_offset;
    int portrait_y_offset;
    int landscape_x_offset;
    int landscape_y_offset;
} lcd_helper_handle_t;

/**
 * Initialize LCD (SPI bus, panel, backlight).
 * After init, display is ON and backlight is ON.
 */
esp_err_t lcd_helper_init(const lcd_helper_config_t *cfg, lcd_helper_handle_t *out);

/**
 * Turn display and backlight ON.
 */
void lcd_helper_turn_on(lcd_helper_handle_t *h);

/**
 * Turn backlight and display OFF.
 */
void lcd_helper_turn_off(lcd_helper_handle_t *h);

/**
 * Put display into sleep mode (before deep sleep).
 */
void lcd_helper_enter_sleep(lcd_helper_handle_t *h);

/**
 * Apply RAM gap offset for a given rotation (0=portrait, 90/270=landscape).
 * rotation: 0, 90, 180, or 270
 */
esp_err_t lcd_helper_apply_gap_for_rotation(lcd_helper_handle_t *h, int rotation);

#ifdef __cplusplus
}
#endif

