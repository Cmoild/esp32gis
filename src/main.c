#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <map.h>
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
#include "esp_lcd_touch_xpt2046.h"
#include "lvgl.h"
#include "driver/uart.h"


#define LCD_HOST               SPI2_HOST
#define LCD_H_RES              (320)
#define LCD_V_RES              (240)
#define LCD_BIT_PER_PIXEL      (16)

#define DASH_BAR_HEIGHT 80

#define PIN_NUM_LCD_CS         (GPIO_NUM_15)
#define PIN_NUM_LCD_PCLK       (GPIO_NUM_18)
#define PIN_NUM_LCD_DATA0      (GPIO_NUM_23)
#define PIN_NUM_LCD_DC         (GPIO_NUM_2)
#define PIN_NUM_LCD_RST        (GPIO_NUM_4)
#define PIN_NUM_LCD_BL         (GPIO_NUM_16)
#define PIN_NUM_TOUCH_CS       (GPIO_NUM_21)

#define DELAY_TIME_MS          (3000)

#define BUFFER_HEIGHT 4

typedef struct esp_ili9341 {
    esp_lcd_panel_io_handle_t panel_io;
    esp_lcd_panel_handle_t panel_handle;
} esp_ili9341;

typedef struct xpt2046 {
    esp_lcd_touch_handle_t tp;
    esp_lcd_panel_io_handle_t io;
} xpt2046;

static uint16_t* windowBuffer;
spi_bus_config_t* global_buscfg;
static lv_color_t draw_buf_array[LCD_V_RES * BUFFER_HEIGHT];
static lv_draw_buf_t draw_buf;
esp_lcd_panel_handle_t g_panel_handle;
static _lock_t lvgl_api_lock;
static lv_obj_t* btnMinus;
static lv_obj_t* btnPlus;
static int8_t g_map_zoom = 15;
static float cur_lat = 59.941846;
static float cur_lon = 30.322033;


// Touch panel borders
const int y_min = 256;
const int y_max = 3900;
const int x_min = 300;
const int x_max = 3800;


#define TAG "test"
static SemaphoreHandle_t refresh_finish = NULL;


IRAM_ATTR static bool notify_refresh_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    BaseType_t need_yield = pdFALSE;

    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}


static void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map) {
    ESP_LOGI("FLUSH CB", "called");
    refresh_finish = xSemaphoreCreateBinary();

    int width  = area->x2 - area->x1 + 1;
    int height = area->y2 - area->y1 + 1;
    int size = width * height;
    
    uint16_t* buf = (uint16_t*)px_map;

   
    for (int i = 0; i < size; i++) {
        buf[i] = ((buf[i] & 0xFF) << 8) | (buf[i] >> 8);
    }
    esp_err_t err = esp_lcd_panel_draw_bitmap(
        g_panel_handle, 
        area->x1, area->y1 + MAP_HEIGHT, 
        area->x2 + 1, area->y2 + MAP_HEIGHT + 1, 
        buf
    );
    if (err != ESP_OK) {
        
    }


    xSemaphoreTake(refresh_finish, portMAX_DELAY);
    vSemaphoreDelete(refresh_finish);

    
    lv_display_flush_ready(display);
}


static void my_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    ESP_LOGI("TOUCH CB", "called");
    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
    esp_lcd_touch_read_data(touch_pad);
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}


static void draw_map(esp_lcd_panel_handle_t panel_handle, const uint16_t* color_data) {
    refresh_finish = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(refresh_finish);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, MAP_WIDTH, MAP_HEIGHT, color_data);
    xSemaphoreTake(refresh_finish, portMAX_DELAY);
    vSemaphoreDelete(refresh_finish);
}


void init_spi() {
    // gpio_config_t io_conf = {
    //     .pin_bit_mask = BIT64(PIN_NUM_LCD_BL),
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE,
    // };
    // gpio_config(&io_conf);
    // gpio_set_level(PIN_NUM_LCD_BL, 1);

    gpio_config_t tp_cs_cfg = {
    .pin_bit_mask = BIT64(PIN_NUM_TOUCH_CS),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&tp_cs_cfg);
    gpio_set_level(PIN_NUM_TOUCH_CS, 1); 

    ESP_LOGI(TAG, "Initialize SPI bus");
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
    esp_lcd_panel_mirror(panel_handle, false, true);
    esp_lcd_panel_disp_off(panel_handle, false);
    esp_ili9341* device = (esp_ili9341*)malloc(sizeof(esp_ili9341*));
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
        ESP_LOGE(TAG, "Failed to mount filesystem.");
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
    ESP_LOGI(TAG, "IO HANDLE MEM: %p", &tp_io_handle);
    ESP_LOGI(TAG, "IO CONFIG MEM: %p", &tp_io_config);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
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

void raw_nmea() { 
	char buf[256];
    int len = uart_read_bytes(UART_NUM_2, buf, 256, portMAX_DELAY);
    if (len > 0) {
        buf[len] = 0;
        printf("%s\n", buf);
    }
}

lv_display_t* lvgl_init(void) {
    ESP_LOGI(TAG, "Initialize LVGL library");

    
    lv_init();

    
    lv_result_t res = lv_draw_buf_init(&draw_buf,
                                       LCD_V_RES,
                                       BUFFER_HEIGHT, 
                                       LV_COLOR_FORMAT_RGB565,
                                       0, 
                                       draw_buf_array,
                                       sizeof(draw_buf_array));
    if (res != LV_RESULT_OK) {
        ESP_LOGE(TAG, "Failed to initialize draw buffer");
        return NULL;
    }

  
    lv_display_t* display = lv_display_create(LCD_V_RES, DASH_BAR_HEIGHT);
    if (!display) {
        ESP_LOGE(TAG, "FAILED DISPLAY INITIALIZATION");
        return NULL;
    }

  
    lv_display_set_draw_buffers(display, &draw_buf, NULL);


    lv_display_set_flush_cb(display, my_flush_cb);

    return display;
}


static void btnMinus_event_cb(lv_event_t *e) {
    ESP_LOGI(TAG, "minus");
    if (g_map_zoom > MIN_ZOOM) {
        g_map_zoom--;
    }
}


static void btnPlus_event_cb(lv_event_t *e) {
    ESP_LOGI(TAG, "plus");
    if (g_map_zoom < MAX_ZOOM) {
        g_map_zoom++;
    }
}


void create_gui(void) {

    static lv_obj_t* labelSPD;
    labelSPD = lv_label_create(lv_screen_active());
    lv_label_set_text(labelSPD, "28.3");
    lv_obj_align(labelSPD, LV_ALIGN_CENTER, -3, -20);
    lv_obj_set_style_text_font(labelSPD, &lv_font_montserrat_32, 0);
    
    static lv_obj_t* labelKMH;
    labelKMH = lv_label_create(lv_screen_active());
    lv_label_set_text(labelKMH, "km/h");
    lv_obj_align(labelKMH, LV_ALIGN_CENTER, 50, -15);
    lv_obj_set_style_text_font(labelKMH, &lv_font_montserrat_14, 0);

    static lv_obj_t* labelT;
    labelT = lv_label_create(lv_screen_active());
    lv_label_set_text(labelT, "00:27:45");
    lv_obj_align(labelT, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_text_font(labelT, &lv_font_montserrat_32, 0);

    static lv_obj_t* labelDIST;
    labelDIST = lv_label_create(lv_screen_active());
    lv_label_set_text(labelDIST, "100.2 km");
    lv_obj_align(labelDIST, LV_ALIGN_LEFT_MID, 10, -15);
    lv_obj_set_style_text_font(labelDIST, &lv_font_montserrat_14, 0);

    btnMinus = lv_button_create(lv_screen_active());
    lv_obj_align(btnMinus, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_size(btnMinus, 40, 39);
    lv_obj_add_event_cb(btnMinus, btnMinus_event_cb, LV_EVENT_ALL, NULL);

    static lv_obj_t* btn_label;
    btn_label = lv_label_create(btnMinus);
    lv_label_set_text(btn_label, "-");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_48, 0);
    lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, -5);

    btnPlus = lv_button_create(lv_screen_active());
    lv_obj_align(btnPlus, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_size(btnPlus, 40, 39);
    lv_obj_add_event_cb(btnPlus, btnPlus_event_cb, LV_EVENT_ALL, NULL);

    static lv_obj_t* btn_label2;
    btn_label2 = lv_label_create(btnPlus);
    lv_label_set_text(btn_label2, "+");
    lv_obj_set_style_text_font(btn_label2, &lv_font_montserrat_48, 0);
    lv_obj_align(btn_label2, LV_ALIGN_CENTER, 0, 0);
}


void lv_port_indev_init(lv_display_t* disp, esp_lcd_touch_handle_t tp) {
    static lv_indev_t* indev;
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, disp);
    lv_indev_set_user_data(indev, tp);
    lv_indev_set_read_cb(indev, my_touch_cb);
}


static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void map_drawing_routine(esp_ili9341* device) {
    mapBorders borders;
    UpdateWindowBuffer(windowBuffer, cur_lat, cur_lon, g_map_zoom, "/card/cmptls/", ".gz", colorPalette, &borders);
    draw_map(device->panel_handle, windowBuffer);
    ESP_LOGI(TAG, "ZOOM: %d", g_map_zoom);
}


void raw_val_to_pixel(uint16_t* x, uint16_t* y) {
    x[0] = (uint16_t)((float)MAX((int)x[0] - x_min, 0) / (float)(x_max - x_min) * (float)LCD_V_RES);
    y[0] = (uint16_t)((float)MAX((int)y[0] - y_min, 0) / (float)(y_max - y_min) * (float)LCD_H_RES);
}


void compute_map_shift(const char command) {
    vec2 deg, linear;
    switch (command)
    {
    case 'd':
        deg.x = cur_lat;
        deg.y = cur_lon;
        linear = deg2float(deg, (uint32_t)g_map_zoom);
        linear.y += 0.3;
        deg = float2deg(linear, g_map_zoom);
        cur_lon = deg.x;
        cur_lat = deg.y;
        break;
    case 'u':
        deg.x = cur_lat;
        deg.y = cur_lon;
        linear = deg2float(deg, (uint32_t)g_map_zoom);
        linear.y -= 0.3;
        deg = float2deg(linear, g_map_zoom);
        cur_lon = deg.x;
        cur_lat = deg.y;
        break;
    case 'l':
        deg.x = cur_lat;
        deg.y = cur_lon;
        linear = deg2float(deg, (uint32_t)g_map_zoom);
        linear.x -= 0.3;
        deg = float2deg(linear, g_map_zoom);
        cur_lon = deg.x;
        cur_lat = deg.y;
        break;
    case 'r':
        deg.x = cur_lat;
        deg.y = cur_lon;
        linear = deg2float(deg, (uint32_t)g_map_zoom);
        linear.x += 0.3;
        deg = float2deg(linear, g_map_zoom);
        cur_lon = deg.x;
        cur_lat = deg.y;
        break;
    default:
        break;
    }
}


static void manual_tp_handling(xpt2046 panel, esp_ili9341* device) {
    g_map_zoom = 15;
    ESP_LOGI(TAG, "Starting manual_tp_handling task");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30));
        uint16_t touchpad_x[1] = {0};
        uint16_t touchpad_y[1] = {0};
        uint8_t touchpad_cnt = 0;
        esp_lcd_touch_read_data(panel.tp);
        /* Get coordinates */
        bool touchpad_pressed = esp_lcd_touch_get_coordinates(panel.tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
        bool map_changed = false;

        raw_nmea();
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // continue;

        if (touchpad_pressed && touchpad_cnt > 0) {
            raw_val_to_pixel(touchpad_x, touchpad_y);
            // Button pressed
            if (touchpad_x[0] < 50) {
                if (touchpad_y[0] < 280 && touchpad_y[0] > 240) {
                    lv_obj_send_event(btnPlus, LV_EVENT_ALL, NULL);
                    map_changed = true;
                }
                if (touchpad_y[0] > 280) {
                    lv_obj_send_event(btnMinus, LV_EVENT_ALL, NULL);
                    map_changed = true;
                }
            }
            // Map pressed
            if (touchpad_y[0] < MAP_HEIGHT) {
                //Bottom right corner
                if (touchpad_y[0] > touchpad_x[0]) {
                    //Bottom left corner
                    if (touchpad_y[0] > -(touchpad_x[0] - MAP_WIDTH)) {
                        ESP_LOGI(TAG, "down");
                        compute_map_shift('d');
                        map_changed = true;
                    }
                    //Top right corner
                    else {
                        ESP_LOGI(TAG, "right");
                        compute_map_shift('r');
                        map_changed = true;
                    }
                }
                // Top Left corner
                else {
                    //Bottom left corner
                    if (touchpad_y[0] > -(touchpad_x[0] - MAP_WIDTH)) {
                        ESP_LOGI(TAG, "left");
                        compute_map_shift('l');
                        map_changed = true;
                    }
                    //Top right corner
                    else {
                        ESP_LOGI(TAG, "up");
                        compute_map_shift('u');
                        map_changed = true;
                    }
                }
            }
            if (map_changed) {
                map_drawing_routine(device);
            }
        }
    }
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
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    printf("\n[DRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_EXEC);
    printf("[IRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_DMA);
    printf("[DMA-capable] Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);
    
    printf("Hello world!\n");
    mapBorders borders;
    windowBuffer = (uint16_t*)malloc(MAP_HEIGHT * MAP_WIDTH * 2);

    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    printf("\n[DRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_EXEC);
    printf("[IRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_DMA);
    printf("[DMA-capable] Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    if (!windowBuffer) {
        ESP_LOGI("main", "FAILED MEMORY ALLOCATION FOR WINDOW BUFFER");
        return;
    }
    init_spi();
    sdmmc_card_t* card = init_sd();
    esp_ili9341* device = ili9341_init();
    xpt2046 panel = xpt2046_init();
    lv_display_t* lvgl_display = lvgl_init();
    neo6m_init();

    lstdir("/card/");
    

    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    printf("\n[DRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_EXEC);
    printf("[IRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_DMA);


    printf("[DMA-capable] Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);
    ESP_LOGI("MEM", "Free: %u, Largest block: %u", info.total_free_bytes, info.largest_free_block);

    UpdateWindowBuffer(windowBuffer, 59.935855, 30.307444, g_map_zoom, "/card/cmptls/", ".gz", colorPalette, &borders);
    draw_map(device->panel_handle, windowBuffer);
    
    xTaskCreate(example_lvgl_port_task, "lvgl_loop", 4096, NULL, 1, NULL);
    _lock_acquire(&lvgl_api_lock);
    create_gui();
    _lock_release(&lvgl_api_lock);
    
    ESP_LOGI(TAG, "TOUCH: %p", &panel);
    vTaskDelay(1000);
    
    manual_tp_handling(panel, device);


    esp_vfs_fat_sdcard_unmount("/card", card);
    xpt2046_deinit(panel);
    ili9341_deinit(device->panel_handle, device->panel_io);
    printf("Restarting now.\n");
    fflush(stdout);
}