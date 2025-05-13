#include <stdio.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>
#include <map.h>
#include <sys/types.h>
#include <puff.h>
#include <defines.h>
#include <inits.h>
#include <lvgl_gui.h>
#include <utils.h>
#include <colors.h>

#define TAG "MAIN"

spi_bus_config_t* global_buscfg;
esp_lcd_panel_handle_t g_panel_handle;
static uint16_t* windowBuffer;
static _lock_t lvgl_api_lock;
lv_obj_t* btnMinus;
lv_obj_t* btnPlus;
int8_t g_map_zoom = 15;
static float cur_lat = 59.941846;
static float cur_lon = 30.322033;


// Touch panel borders
const int y_min = 256;
const int y_max = 3900;
const int x_min = 300;
const int x_max = 3800;


static void draw_map(esp_lcd_panel_handle_t panel_handle, const uint16_t* color_data) {
    refresh_finish = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(refresh_finish);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, MAP_WIDTH, MAP_HEIGHT, color_data);
    xSemaphoreTake(refresh_finish, portMAX_DELAY);
    vSemaphoreDelete(refresh_finish);
}


void raw_nmea() { 
	char buf[256];
    int len = uart_read_bytes(UART_NUM_2, buf, 256, portMAX_DELAY);
    if (len > 0) {
        buf[len] = 0;
        printf("%s\n", buf);
    }
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


void app_main(void)
{
    print_memory_info();
    printf("Hello world!\n");

    mapBorders borders;

    windowBuffer = (uint16_t*)malloc(MAP_HEIGHT * MAP_WIDTH * 2);

    if (!windowBuffer) {
        ESP_LOGI("main", "FAILED MEMORY ALLOCATION FOR WINDOW BUFFER");
        return;
    }
    init_spi();
    sdmmc_card_t* card = init_sd();
    esp_ili9341* device = ili9341_init();
    xpt2046 panel = xpt2046_init();
    lvgl_init();
    neo6m_init();

    lstdir("/card/");

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