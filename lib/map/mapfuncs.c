#include "mapfuncs.h"
#include <stdio.h>
#include <puff.h>
#include "esp_log.h"
#include "string.h"


vec2 deg2float(vec2 coords, uint32_t zoom) {
    uint32_t z = (1 << zoom);
    vec2 ret;
    ret.x = (coords.y + 180.0) / 360.0 * z;
    ret.y = (1.0 - asinh(tan(coords.x * M_PI/180.0)) / M_PI) / 2.0 * z;
    return ret;
}


u16vec2 deg2pointOnMap(vec2 tile, uint32_t zoom, uint16_t tile_resolution) {
    u16vec2 ret;
    ret.x = (uint16_t)((tile.x - roundf(tile.x)) * (float)tile_resolution);
    ret.y = (uint16_t)((tile.y - roundf(tile.y)) * (float)tile_resolution);
    return ret;
}


uint8_t* getFileContent(const char* filePath) {
    unsigned long size = 0;
    unsigned char* data = 0;
    FILE* fp = fopen(filePath, "rb");
    if (!fp) {
        ESP_LOGE("map", "FAILED TO LOAD FILE");
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc(size);
    if (!data) {
        ESP_LOGE("map", "FAILED TO ALLOCATE MEMORY FOR STORING FILE CONTENT");
        return NULL;
    }
    fread(data, 1, size, fp);
    fclose(fp);
    printf("SIZE: %d\n", (int)size);

    uint8_t* decomp = (uint8_t*)malloc(65536);
    if (!decomp) {
        ESP_LOGE("map", "FAILED TO ALLOCATE MEMORY FOR STORING FILE DECOMPRESSED CONTENT");
        return NULL;
    }

    unsigned long len = 65536;
    int status = puff(decomp, &len, data + 10, &size);
    free(data);
    if (status != 0) {
        free(decomp);
        ESP_LOGE("map", "FAILED TO DECOMPRESS FILE");
        return NULL;
    }

    return decomp;
}


// tilesRoorDir must end with '/', e.g. "/sdcard/tiles/"
char* getFileName(float lat, float lon, uint32_t zoom, char* tilesRootDir, char* imageFileExtesion) {
    char* filePathBuffer = malloc(50);
    size_t n = strlen(tilesRootDir);
    memcpy(filePathBuffer, tilesRootDir, n);
    vec2 coords = {
        .x = lat,
        .y = lon
    };
    coords = deg2float(coords, zoom);
    int nApp = sprintf(&filePathBuffer[n], "%u", zoom);
    n += nApp;
    filePathBuffer[n] = '/';
    n++;
    nApp = sprintf(&filePathBuffer[n], "%u", (uint32_t)coords.x);
    n += nApp;
    filePathBuffer[n] = '/';
    n++;
    nApp = sprintf(&filePathBuffer[n], "%u", (uint32_t)coords.y);
    n += nApp;
    nApp = strlen(imageFileExtesion);
    memcpy(&filePathBuffer[n], imageFileExtesion, nApp);
    filePathBuffer[n + nApp] = '\0';
    return filePathBuffer;
}


void UpdateWindowBuffer(uint16_t* buffer, float lat, float lon, uint32_t zoom, char* tilesRootDir, char* imageFileExtesion, const uint16_t* colorPalette){
    if (zoom < MIN_ZOOM || zoom > MAX_ZOOM) {
        return;
    }
    char* fileName = getFileName(lat, lon, zoom, tilesRootDir, imageFileExtesion);
    uint8_t* decomp = getFileContent(fileName);
    free(fileName);
    int n = 0;
    for (int i = 0; i < MAP_HEIGHT; i++) {
        for (int j = TILE_WIDTH - 1; j >= 0; j--) {
            if (j < MAP_WIDTH) {
                buffer[n] = colorPalette[decomp[i * TILE_WIDTH + j]];
                // change endianess (to be deleted after editing lookup table)
                buffer[n] = ((buffer[n] & 0xFF) << 8) | (buffer[n] >> 8);
                n++;
            }
        }
    }
    free(decomp);
}