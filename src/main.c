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
#include <esp_timer.h>

#define TAG "MAIN"

spi_bus_config_t* global_buscfg;
esp_lcd_panel_handle_t g_panel_handle;
static uint16_t* windowBuffer;
static _lock_t lvgl_api_lock;
lv_obj_t* btnMinus;
lv_obj_t* btnPlus;
lv_obj_t* labelT;
lv_obj_t* labelSPD;
lv_obj_t* btnFollowLocation;
int8_t g_map_zoom = 15;
static float cur_lat = 59.941846;
static float cur_lon = 30.322033;
static gps_data_t g_gps_data;
bool use_gps;


// Touch panel borders
const int y_min = 256;
const int y_max = 3900;
const int x_min = 300;
const int x_max = 3800;


static const int location_pointer_bitmap[13 * 13] = {
    -1, -1, -1, -1,  0,  0,  0,  0,  0, -1, -1, -1, -1,
    -1, -1, -1,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1,
    -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1,
    -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
    -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1,
    -1, -1, -1,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1,
    -1, -1, -1, -1,  0,  0,  0,  0,  0, -1, -1, -1, -1,
};


static void draw_map(esp_lcd_panel_handle_t panel_handle, const uint16_t* color_data) {
    refresh_finish = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(refresh_finish);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, MAP_WIDTH, MAP_HEIGHT, color_data);
    xSemaphoreTake(refresh_finish, portMAX_DELAY);
    vSemaphoreDelete(refresh_finish);
}


void read_nmea() { 
	char buf[GPS_BUFFER_LENGTH + 1];
    int byte_len = uart_read_bytes(UART_NUM_2, buf, GPS_BUFFER_LENGTH, portMAX_DELAY);
    if (byte_len > 0) {
        buf[byte_len] = 0;
        enum GPS_message_status status = EMPTY_STATUS;
        char gpsline[100] = {0};
        int len = get_gps_line(buf, &status, gpsline, 0);
        if (status == CHECK_TAG) {
            char tag_buf[7];
            byte_len = uart_read_bytes(UART_NUM_2, tag_buf, 7, portMAX_DELAY);
            if (byte_len > 0) {
                for (int i = 0; i < GPS_BUFFER_LENGTH - 7; i++) {
                    buf[i] = buf[i + 7];
                }
                for (int i = GPS_BUFFER_LENGTH - 7; i < GPS_BUFFER_LENGTH; i++) {
                    buf[i] = tag_buf[i - (GPS_BUFFER_LENGTH - 7)];
                }
                status = EMPTY_STATUS;
                len = get_gps_line(buf, &status, gpsline, 0);
            }
        }
        if (status == MESSAGE_STARTED) {
            byte_len = uart_read_bytes(UART_NUM_2, buf, GPS_BUFFER_LENGTH, portMAX_DELAY);
            if (byte_len > 0) {
                len = get_gps_line(buf, &status, gpsline, len);
            }
        }
        if (len > 0) {
            g_gps_data = parse_gps_line(gpsline, len, 3);
            g_gps_data.timestamp = (float)esp_timer_get_time() / 1000. / 1000.;
        }
    }
}


static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        vTaskDelay(pdMS_TO_TICKS(3));
        read_nmea();
    }
}


void map_drawing_routine(esp_ili9341* device) {
    mapBorders borders;
    if (!use_gps) {
        UpdateWindowBuffer(windowBuffer, cur_lat, cur_lon, g_map_zoom, "/card/cmptls/", ".gz", colorPalette, &borders);
    }
    else {
        UpdateWindowBuffer(windowBuffer, g_gps_data.lat, g_gps_data.lon, g_map_zoom, "/card/cmptls/", ".gz", colorPalette, &borders);
        for (int i = 0; i < 13; i++) {
            for (int j = 0; j < 13; j++) {
                if (location_pointer_bitmap[i * 13 + j] != -1) windowBuffer[(MAP_CENTER_Y + i - 6) * MAP_WIDTH + MAP_CENTER_X + j - 6] = 0;
            }
        }
    }
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
    ESP_LOGI(TAG, "Starting manual_tp_handling task");
    gps_data_t prev_data = {
        .lat = 0,
        .lon = 0,
        .altitude = 0,
        .hoursMinutes = {'0', '0', ':', '0', '0', 0},
        .is_valid = false,
        .timestamp = 0.,
    };
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30));
        uint16_t touchpad_x[1] = {0};
        uint16_t touchpad_y[1] = {0};
        uint8_t touchpad_cnt = 0;
        esp_lcd_touch_read_data(panel.tp);
        /* Get coordinates */
        bool touchpad_pressed = esp_lcd_touch_get_coordinates(panel.tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
        bool map_changed = false;


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
            if (touchpad_x[0] > 200 && touchpad_y[0] > 280) {
                lv_obj_send_event(btnFollowLocation, LV_EVENT_ALL, NULL);
                if (g_gps_data.is_valid) {
                    cur_lat = g_gps_data.lat;
                    cur_lon = g_gps_data.lon;
                }
                map_changed = true;
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
            if (map_changed && !use_gps) {
                map_drawing_routine(device);
            }
        }

        if (g_gps_data.is_valid && (prev_data.hoursMinutes[4] != g_gps_data.hoursMinutes[4])) {
            lv_label_set_text(labelT, g_gps_data.hoursMinutes);
            lv_refr_now(NULL);
        }
        
        if (use_gps && g_gps_data.is_valid && ((prev_data.lat != g_gps_data.lat && prev_data.lon != g_gps_data.lon) || map_changed)) {
            map_drawing_routine(device);
        }

        if (g_gps_data.is_valid && prev_data.timestamp != g_gps_data.timestamp) {
            float spd = calculate_speed_kmh(prev_data, g_gps_data);
            char lbl[10];
            sprintf(lbl, "%.1f", spd);
            lv_label_set_text_fmt(labelSPD, lbl);
            lv_refr_now(NULL);
            prev_data = g_gps_data;
        }

    }
}


void app_main(void) {
    gps_data_t gps_data = {
        .lat = 0,
        .lon = 0,
        .altitude = 0,
        .hoursMinutes = {'0', '0', ':', '0', '0', 0},
        .is_valid = false,
    };
    g_gps_data = gps_data;

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

    UpdateWindowBuffer(windowBuffer, 59.935855, 30.307444, MIN_ZOOM, "/card/cmptls/", ".gz", colorPalette, &borders);
    draw_map(device->panel_handle, windowBuffer);
    
    xTaskCreate(example_lvgl_port_task, "lvgl_loop", 4096, NULL, 1, NULL);
    _lock_acquire(&lvgl_api_lock);
    create_gui();
    _lock_release(&lvgl_api_lock);
    
    ESP_LOGI(TAG, "TOUCH: %p", &panel);
    vTaskDelay(pdMS_TO_TICKS(100));

    use_gps = false;
    manual_tp_handling(panel, device);


    esp_vfs_fat_sdcard_unmount("/card", card);
    xpt2046_deinit(panel);
    ili9341_deinit(device->panel_handle, device->panel_io);
    printf("Restarting now.\n");
    fflush(stdout);
}