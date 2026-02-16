#include "lcd_helper.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_vendor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"

static const char *TAG = "LCD";

esp_err_t lcd_helper_init(const lcd_helper_config_t *cfg, lcd_helper_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "cfg/out NULL");

    /* Release any GPIO hold from deep sleep */
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(cfg->gpio_bl);

    ESP_LOGI(TAG, "Init %dx%d ST7789", cfg->h_res, cfg->v_res);

    /* Backlight GPIO */
    gpio_config_t bk_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << cfg->gpio_bl
    };
    ESP_ERROR_CHECK(gpio_config(&bk_config));
    gpio_set_level(cfg->gpio_bl, !cfg->bl_on_level);

    /* SPI bus */
    const spi_bus_config_t buscfg = {
        .sclk_io_num = cfg->gpio_sclk,
        .mosi_io_num = cfg->gpio_mosi,
        .miso_io_num = GPIO_NUM_NC,
        .max_transfer_sz = cfg->h_res * cfg->draw_buff_height * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO));

    /* Panel IO */
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = cfg->gpio_dc,
        .cs_gpio_num = cfg->gpio_cs,
        .pclk_hz = cfg->pixel_clk_hz,
        .lcd_cmd_bits = cfg->cmd_bits,
        .lcd_param_bits = cfg->param_bits,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)cfg->spi_host, &io_config, &out->io));

    /* ST7789 panel */
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = cfg->gpio_rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = cfg->bits_per_pixel,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(out->io, &panel_config, &out->panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(out->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(out->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(out->panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(out->panel, true));

    /* Backlight ON */
    gpio_set_level(cfg->gpio_bl, cfg->bl_on_level);

    /* Store config in handle */
    out->gpio_bl = cfg->gpio_bl;
    out->bl_on_level = cfg->bl_on_level;
    out->h_res = cfg->h_res;
    out->v_res = cfg->v_res;
    out->portrait_x_offset = cfg->portrait_x_offset;
    out->portrait_y_offset = cfg->portrait_y_offset;
    out->landscape_x_offset = cfg->landscape_x_offset;
    out->landscape_y_offset = cfg->landscape_y_offset;

    return ESP_OK;
}

void lcd_helper_turn_on(lcd_helper_handle_t *h)
{
    if (!h || !h->panel) return;
    (void)esp_lcd_panel_disp_on_off(h->panel, true);
    gpio_set_level(h->gpio_bl, h->bl_on_level);
}

void lcd_helper_turn_off(lcd_helper_handle_t *h)
{
    if (!h) return;
    gpio_set_level(h->gpio_bl, !h->bl_on_level);
    if (h->panel) {
        (void)esp_lcd_panel_disp_on_off(h->panel, false);
    }
}

void lcd_helper_enter_sleep(lcd_helper_handle_t *h)
{
    if (!h) return;
    gpio_set_level(h->gpio_bl, !h->bl_on_level);
    gpio_hold_en(h->gpio_bl);
    if (h->panel) {
        (void)esp_lcd_panel_disp_on_off(h->panel, false);
        (void)esp_lcd_panel_disp_sleep(h->panel, true);
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_LOGI(TAG, "Entered sleep mode");
    }

}

esp_err_t lcd_helper_apply_gap_for_rotation(lcd_helper_handle_t *h, int rotation)
{
    if (!h || !h->panel) return ESP_ERR_INVALID_ARG;

    int x_off, y_off;
    if (rotation == 90 || rotation == 270) {
        x_off = h->landscape_x_offset;
        y_off = h->landscape_y_offset;
    } else {
        x_off = h->portrait_x_offset;
        y_off = h->portrait_y_offset;
    }

    return esp_lcd_panel_set_gap(h->panel, x_off, y_off);
}

