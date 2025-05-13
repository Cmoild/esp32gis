#include <inits.h>
#include <defines.h>


SemaphoreHandle_t refresh_finish = NULL;


IRAM_ATTR static bool notify_refresh_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    BaseType_t need_yield = pdFALSE;

    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}


void init_spi() {
    gpio_config_t tp_cs_cfg = {
        .pin_bit_mask = BIT64(PIN_NUM_TOUCH_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&tp_cs_cfg);
    gpio_set_level(PIN_NUM_TOUCH_CS, 1); 

    ESP_LOGI(INIT_TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = ILI9341_PANEL_BUS_SPI_CONFIG(PIN_NUM_LCD_PCLK, PIN_NUM_LCD_DATA0,
                                    LCD_H_RES * 80 * LCD_BIT_PER_PIXEL / 8);
    buscfg.miso_io_num = 19;
    global_buscfg = &buscfg;
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
}


esp_ili9341* ili9341_init() {
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

    ESP_LOGI(INIT_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_LCD_CS, PIN_NUM_LCD_DC,
            notify_refresh_ready, NULL);

    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

    ESP_LOGI(INIT_TAG, "Install ili9341 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST, // Shared with Touch reset
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };
    esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_mirror(panel_handle, false, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);
    esp_ili9341* device = (esp_ili9341*)malloc(sizeof(esp_ili9341));
    device->panel_handle = panel_handle;
    g_panel_handle = panel_handle;
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
    if (ret != ESP_OK) {
        ESP_LOGE(INIT_TAG, "Failed to mount filesystem.");
        return NULL;
    }
    ESP_LOGI("example", "Filesystem mounted");
    sdmmc_card_print_info(stdout, card);

    return card;
}


xpt2046 xpt2046_init() {
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(PIN_NUM_TOUCH_CS);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_err_t err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &tp_io_config, &tp_io_handle);
    ESP_LOGI("ERROR 1 STATUS", "ERROR: %d", err); 
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ESP_LOGI(INIT_TAG, "IO HANDLE MEM: %p", &tp_io_handle);
    ESP_LOGI(INIT_TAG, "IO CONFIG MEM: %p", &tp_io_config);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI(INIT_TAG, "Initialize touch controller XPT2046");
    err = esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI("ERROR 2 STATUS", "ERROR: %d", err);
    xpt2046 xpt = {
        .tp =tp,
        .io = tp_io_handle
    };
    return xpt;
}


void xpt2046_deinit(xpt2046 xpt) {
    esp_lcd_touch_del(xpt.tp);
    esp_lcd_panel_io_del(xpt.io);
    gpio_reset_pin(PIN_NUM_TOUCH_CS);
}

void neo6m_init() {
    uart_port_t uart_port_num = UART_NUM_2;
    uart_config_t config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t err = uart_param_config(uart_port_num, &config);
    
    if (err == ESP_FAIL) {
        ESP_LOGE("NEO6M", "FAILED UART PARAM CONFIG");
        return;
    }

    err = uart_set_pin(uart_port_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (err == ESP_FAIL) {
        ESP_LOGE("NEO6M", "FAILED UART SET PIN");
        return;
    }

    err = uart_driver_install(uart_port_num, 256, 0, 0, NULL, 0);

    if (err == ESP_FAIL) {
        ESP_LOGE("NEO6M", "FAILED UART DRIVER INSTALL");
        return;
    }
}
