#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <mapfuncs.h>
#include <sys/types.h>
#include <dirent.h>
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include <puff.h>
#include "esp_lcd_ili9341.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9341.h"
#include "unity.h"
#include "colors.h"


static uint16_t* windowBuffer;
spi_bus_config_t* global_buscfg;


#define LCD_HOST               SPI2_HOST
#define LCD_H_RES              (320)
#define LCD_V_RES              (240)
#define LCD_BIT_PER_PIXEL      (16)

#define PIN_NUM_LCD_CS         (GPIO_NUM_15)
#define PIN_NUM_LCD_PCLK       (GPIO_NUM_18)
#define PIN_NUM_LCD_DATA0      (GPIO_NUM_23)
#define PIN_NUM_LCD_DC         (GPIO_NUM_2)
#define PIN_NUM_LCD_RST        (GPIO_NUM_4)
#define PIN_NUM_LCD_BL         (GPIO_NUM_16)

#define DELAY_TIME_MS          (3000)


#define TAG "test"
static SemaphoreHandle_t refresh_finish = NULL;


typedef struct esp_ili9341 {
    esp_lcd_panel_io_handle_t panel_io;
    esp_lcd_panel_handle_t panel_handle;
} esp_ili9341;


IRAM_ATTR static bool notify_refresh_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    BaseType_t need_yield = pdFALSE;

    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}


static void draw_map(esp_lcd_panel_handle_t panel_handle, const uint16_t* color_data) {
    refresh_finish = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(refresh_finish);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, MAP_WIDTH, MAP_HEIGHT, color_data);
    xSemaphoreTake(refresh_finish, portMAX_DELAY);
    vSemaphoreDelete(refresh_finish);
}


esp_ili9341* ili9341_init_spi() {
    /*
     * DISPLAY SETUP
     * MISO     Not connected
     * MOSI     23
     * CLK      18
     * CS       15
     * DC       2
     * RST      4
     * LED      3.3V
     */
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(PIN_NUM_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_LCD_BL, 1);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = ILI9341_PANEL_BUS_SPI_CONFIG(PIN_NUM_LCD_PCLK, PIN_NUM_LCD_DATA0,
                                    LCD_H_RES * 80 * LCD_BIT_PER_PIXEL / 8);
    buscfg.miso_io_num = 19;
    global_buscfg = &buscfg;
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_LCD_CS, PIN_NUM_LCD_DC,
            notify_refresh_ready, NULL);
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

    ESP_LOGI(TAG, "Install ili9341 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST, // Shared with Touch reset
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };
    esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_mirror(panel_handle, true, true);
    esp_lcd_panel_disp_off(panel_handle, false);
    esp_ili9341* device = (esp_ili9341*)malloc(sizeof(esp_ili9341*));
    device->panel_handle = panel_handle;
    device->panel_io = io_handle;
    return device;
}


void ili9341_deinit(esp_lcd_panel_handle_t panel_handle, esp_lcd_panel_io_handle_t io_handle) {
    gpio_reset_pin(PIN_NUM_LCD_BL);
    esp_lcd_panel_del(panel_handle);
    esp_lcd_panel_io_del(io_handle);
    spi_bus_free(LCD_HOST);
}


sdmmc_card_t* init_sd() {
    /*
     * SD card setup:
     * 
     * CS       5
     * MOSI     23
     * CLK      18
     * MISO     19
     * 
     */
    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t config = SDSPI_DEVICE_CONFIG_DEFAULT();
    config.gpio_cs = 5;
    config.host_id = host.slot;
    esp_vfs_fat_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 1024,
    };
    sdmmc_card_t* card;

    ESP_LOGI("example", "Mounting FS");
    ret = esp_vfs_fat_sdspi_mount("/card", &host, &config, &mount, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("example", "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE("example", "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        // esp_restart();
    }
    ESP_LOGI("example", "Filesystem mounted");
    sdmmc_card_print_info(stdout, card);

    return card;
}


void lstdir(char* path) {
    DIR *dp;
    struct dirent *ep;     
    dp = opendir (path);
    if (dp != NULL)
    {
        while ((ep = readdir (dp)) != NULL)
        puts (ep->d_name);
            
        (void) closedir (dp);
        return;
    }
    else
    {
        perror ("Couldn't open the directory");
        return;
    }
}


void app_main(void)
{
    printf("Hello world!\n");
    windowBuffer = (uint16_t*)malloc(MAP_HEIGHT * MAP_WIDTH * 2);
    if (!windowBuffer) {
        ESP_LOGI("main", "FAILED MEMORY ALLOCATION FOR WINDOW BUFFER");
    }
    esp_ili9341* device = ili9341_init_spi();
    sdmmc_card_t* card = init_sd();

    UpdateWindowBuffer(windowBuffer, 59.971485, 30.323116, 15, "/card/cmptls/", ".gz", colorPalette);
    draw_map(device->panel_handle, windowBuffer);

    esp_vfs_fat_sdcard_unmount("/card", card);
    ili9341_deinit(device->panel_handle, device->panel_io);
    printf("Restarting now.\n");
    fflush(stdout);
}