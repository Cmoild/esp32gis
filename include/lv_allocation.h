#include <stdlib.h>
#include <esp_heap_caps.h>

void *lvgl_malloc(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

void lvgl_free(void *ptr) {
    heap_caps_free(ptr);
}