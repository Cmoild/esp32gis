#include "mapfuncs.h"
#include <stdio.h>


vec2 deg2float(vec2 coords, uint32_t zoom) {
    uint32_t z = (1 << zoom);
    vec2 ret;
    ret.x = (coords.y + 180.0) / 360.0 * z;
    ret.y = (1.0 - asinh(tan(coords.x * M_PI/180.0)) / M_PI) / 2.0 * z;
    return ret;
}


u16vec2 deg2pointOnMap(vec2 tile, uint32_t zoom, uint16_t tile_resolution) {
    u16vec2 ret;
    ret.x = (tile.x - (uint32_t)tile.x) * tile_resolution;
    ret.y = (tile.y - (uint32_t)tile.y) * tile_resolution;
    return ret;
}


// uint8_t* getFileContent(const char* filePath) {
//     FILE* file = fopen(filePath, "rb");
//     int length = 0;
//     uint8_t* buffer = NULL;

//     // if (file) {
//     //     fseek(file, 0, SEEK_END);
//     //     length = ftell(file);
//     //     buffer = (uint8_t*)malloc(length);
//     //     if (buffer) {
//     //         fread(buffer, 1, length, )
//     //     }
//     // }
// }