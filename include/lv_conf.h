#define LV_CONF_INCLUDE_SIMPLE 1


#define LV_MEM_SIZE (16 * 1024)
#define LV_MEM_CUSTOM 1

#define LV_MEM_CUSTOM_INCLUDE <lv_allocation.h>
#define LV_MEM_CUSTOM_ALLOC   lvgl_malloc
#define LV_MEM_CUSTOM_FREE    lvgl_free


#define LV_COLOR_DEPTH 16
#define LV_HOR_RES_MAX 320
#define LV_VER_RES_MAX 240


#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 0
#define LV_USE_ASSERT_MALLOC 0
#define LV_USE_ASSERT_STYLE 0


#define LV_USE_BTN 1
#define LV_USE_LABEL 1


#define LV_USE_ARC 1
#define LV_USE_BAR 1
#define LV_USE_CHECKBOX 1
#define LV_USE_DROPDOWN 1
#define LV_USE_IMG 1
#define LV_USE_LINE 1
#define LV_USE_LIST 1
#define LV_USE_SLIDER 1
#define LV_USE_SWITCH 1
#define LV_USE_TEXTAREA 1
#define LV_USE_TABLE 1
#define LV_USE_CANVAS 1
#define LV_USE_CONT 1
#define LV_USE_TILEVIEW 1
#define LV_USE_TABVIEW 1
#define LV_USE_SPINBOX 1
#define LV_USE_OBJREALIGN 1


#define LV_USE_ANIMATION 0
#define LV_USE_THEME_DEFAULT 0
#define LV_USE_THEME_BASIC 0


#define LV_SPRINTF_CUSTOM 1
#define LV_MEM_POOL_INCLUDE <esp_heap_caps.h>
#define LV_MEM_POOL_ALLOC(size) heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

#define LV_COLOR_SWAP 1

#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_FONT_MONTSERRAT_32 1