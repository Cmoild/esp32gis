#ifndef INITS_H
#define INITS_H

#include <sys/types.h>
#include <dirent.h>
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "esp_lcd_ili9341.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"
#include "driver/uart.h"

#define INIT_TAG "INIT"

typedef struct esp_ili9341 {
    esp_lcd_panel_io_handle_t panel_io;
    esp_lcd_panel_handle_t panel_handle;
} esp_ili9341;

typedef struct xpt2046 {
    esp_lcd_touch_handle_t tp;
    esp_lcd_panel_io_handle_t io;
} xpt2046;

extern spi_bus_config_t* global_buscfg;

extern esp_lcd_panel_handle_t g_panel_handle;

extern SemaphoreHandle_t refresh_finish;

void init_spi();

esp_ili9341* ili9341_init();

void ili9341_deinit(esp_lcd_panel_handle_t panel_handle, esp_lcd_panel_io_handle_t io_handle);

sdmmc_card_t* init_sd();

xpt2046 xpt2046_init();

void xpt2046_deinit(xpt2046 xpt);

void neo6m_init();

#endif