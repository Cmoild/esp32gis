#ifndef MAP_FUNCS_H
#define MAP_FUNCS_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.141593
#endif


typedef struct vec2 {
    float x;
    float y;
} vec2;


typedef struct u16vec2 {
    uint16_t x;
    uint16_t y;
} u16vec2;


vec2 deg2float(vec2 coords, uint32_t zoom);


u16vec2 deg2pointOnMap(vec2 tile, uint32_t zoom, uint16_t tile_resolition);


uint8_t* getFileContent(const char* filePath);


#ifdef __cplusplus
}
#endif

#endif /* MAP_FUNCS_H */