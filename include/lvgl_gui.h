#ifndef LVGL_GUI_H
#define LVGL_GUI_H

#include <lvgl.h>
#include <defines.h>
#include <inits.h>
#include <map.h>

#define LVGL_TAG "LVGL"

extern int8_t g_map_zoom;

extern lv_obj_t* btnMinus;
extern lv_obj_t* btnPlus;

lv_display_t* lvgl_init(void);

void create_gui(void);

#endif