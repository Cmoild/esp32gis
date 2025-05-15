#include <lvgl_gui.h>


static lv_color_t draw_buf_array[LCD_V_RES * BUFFER_HEIGHT];

static lv_draw_buf_t draw_buf;

static void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map) {
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


lv_display_t* lvgl_init(void) {
    ESP_LOGI(LVGL_TAG, "Initialize LVGL library");

    
    lv_init();

    
    lv_result_t res = lv_draw_buf_init(&draw_buf,
                                       LCD_V_RES,
                                       BUFFER_HEIGHT, 
                                       LV_COLOR_FORMAT_RGB565,
                                       0, 
                                       draw_buf_array,
                                       sizeof(draw_buf_array));
    if (res != LV_RESULT_OK) {
        ESP_LOGE(LVGL_TAG, "Failed to initialize draw buffer");
        return NULL;
    }

  
    lv_display_t* display = lv_display_create(LCD_V_RES, DASH_BAR_HEIGHT);
    if (!display) {
        ESP_LOGE(LVGL_TAG, "FAILED DISPLAY INITIALIZATION");
        return NULL;
    }

  
    lv_display_set_draw_buffers(display, &draw_buf, NULL);


    lv_display_set_flush_cb(display, my_flush_cb);

    return display;
}


static void btnMinus_event_cb(lv_event_t *e) {
    ESP_LOGI(LVGL_TAG, "minus");
    if (g_map_zoom > MIN_ZOOM) {
        g_map_zoom--;
    }
}


static void btnPlus_event_cb(lv_event_t *e) {
    ESP_LOGI(LVGL_TAG, "plus");
    if (g_map_zoom < MAX_ZOOM) {
        g_map_zoom++;
    }
}


static void btnFollowLocation_event_cb(lv_event_t *e) {
    ESP_LOGI(LVGL_TAG, "follow");
    use_gps = !use_gps;
}


void create_gui(void) {

    labelSPD = lv_label_create(lv_screen_active());
    lv_label_set_text(labelSPD, "0.0");
    lv_obj_align(labelSPD, LV_ALIGN_CENTER, -3, -20);
    lv_obj_set_style_text_font(labelSPD, &lv_font_montserrat_32, 0);
    
    static lv_obj_t* labelKMH;
    labelKMH = lv_label_create(lv_screen_active());
    lv_label_set_text(labelKMH, "km/h");
    lv_obj_align(labelKMH, LV_ALIGN_CENTER, 50, -15);
    lv_obj_set_style_text_font(labelKMH, &lv_font_montserrat_14, 0);

    labelT = lv_label_create(lv_screen_active());
    lv_label_set_text(labelT, "00:00");
    lv_obj_align(labelT, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_text_font(labelT, &lv_font_montserrat_32, 0);

    // static lv_obj_t* labelDIST;
    // labelDIST = lv_label_create(lv_screen_active());
    // lv_label_set_text(labelDIST, "100.2 km");
    // lv_obj_align(labelDIST, LV_ALIGN_LEFT_MID, 10, -15);
    // lv_obj_set_style_text_font(labelDIST, &lv_font_montserrat_14, 0);

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

    btnFollowLocation = lv_button_create(lv_screen_active());
    lv_obj_align(btnFollowLocation, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_size(btnFollowLocation, 40, 39);
    lv_obj_add_event_cb(btnFollowLocation, btnFollowLocation_event_cb, LV_EVENT_ALL, NULL);

    static lv_obj_t* btn_label3;
    btn_label3 = lv_label_create(btnFollowLocation);
    lv_label_set_text(btn_label3, "o");
    lv_obj_set_style_text_font(btn_label3, &lv_font_montserrat_48, 0);
    lv_obj_align(btn_label3, LV_ALIGN_CENTER, 0, -4);
}