#include "map.h"
#include <stdio.h>
#include <puff.h>
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"


vec2 deg2float(vec2 coords, uint32_t zoom) {
    uint32_t z = (1 << zoom);
    vec2 ret;
    ret.x = (coords.y + 180.0) / 360.0 * z;
    ret.y = (1.0 - asinh(tan(coords.x * M_PI/180.0)) / M_PI) / 2.0 * z;
    return ret;
}


// Returns vec2, where x - lon, y - lat
vec2 float2deg(vec2 coords, uint32_t zoom) {
    uint32_t n = (1 << zoom);
    vec2 ret = {
        .x = coords.x / (float)n * 360.f - 180.f,
        .y = (atan(sinh(M_PI * (1.f - 2.f * coords.y / (float)n)))) * 180.f / M_PI
    };
    return ret;
}


u16vec2 deg2pointOnMap(vec2 tile, uint32_t zoom, uint16_t tile_resolution) {
    u16vec2 ret;
    ret.x = (uint16_t)((tile.x - floorf(tile.x)) * (float)tile_resolution);
    ret.y = (uint16_t)((tile.y - floorf(tile.y)) * (float)tile_resolution);
    return ret;
}


u16vec2 pointOnMapXY(float x, float y) {
    u16vec2 ret = {
        .x = (uint16_t)((x - floorf(x)) * (float)TILE_WIDTH),
        .y = (uint16_t)((y - floorf(y)) * (float)TILE_HEIGHT)
    };
    return ret;
}


uint8_t* getFileContent(const char* filePath) {
    unsigned long size = 0;
    unsigned char* data = 0;
    FILE* fp = fopen(filePath, "rb");
    if (!fp) {
        ESP_LOGE("map", "FAILED TO LOAD FILE");
        uint8_t* failed = (uint8_t*)calloc(65536, 1);
        return failed;
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
        ESP_LOGE("map", "FAILED TO ALLOCATE MEMORY FOR STORING FILE UNCOMPRESSED CONTENT");
        return NULL;
    }

    unsigned long len = 65536;
    int status = puff(decomp, &len, data + 10, &size);
    free(data);
    if (status != 0) {
        free(decomp);
        ESP_LOGE("map", "FAILED TO UNCOMPRESS FILE");
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
    int nApp = sprintf(&filePathBuffer[n], "%lu", zoom);
    n += nApp;
    filePathBuffer[n] = '/';
    n++;
    nApp = sprintf(&filePathBuffer[n], "%lu", (uint32_t)coords.x);
    n += nApp;
    filePathBuffer[n] = '/';
    n++;
    nApp = sprintf(&filePathBuffer[n], "%lu", (uint32_t)coords.y);
    n += nApp;
    nApp = strlen(imageFileExtesion);
    memcpy(&filePathBuffer[n], imageFileExtesion, nApp);
    filePathBuffer[n + nApp] = '\0';
    return filePathBuffer;
}


char* getFileNameXY(float x, float y, uint32_t zoom, char* tilesRootDir, char* imageFileExtesion) {
    char* filePathBuffer = malloc(50);
    size_t n = strlen(tilesRootDir);
    memcpy(filePathBuffer, tilesRootDir, n);
    vec2 coords = {
        .x = x,
        .y = y
    };
    int nApp = sprintf(&filePathBuffer[n], "%lu", zoom);
    n += nApp;
    filePathBuffer[n] = '/';
    n++;
    nApp = sprintf(&filePathBuffer[n], "%lu", (uint32_t)coords.x);
    n += nApp;
    filePathBuffer[n] = '/';
    n++;
    nApp = sprintf(&filePathBuffer[n], "%lu", (uint32_t)coords.y);
    n += nApp;
    nApp = strlen(imageFileExtesion);
    memcpy(&filePathBuffer[n], imageFileExtesion, nApp);
    filePathBuffer[n + nApp] = '\0';
    return filePathBuffer;
}


void updateMap(
    uint16_t* buffer, 
    float lat, 
    float lon, 
    uint32_t zoom, 
    char* tilesRootDir, 
    char* imageFileExtesion, 
    const uint16_t* colorPalette,
    mapBorders* borders
) {
    vec2 coords = {
        .x = lat,
        .y = lon
    };
    vec2 linearCoords = deg2float(coords, zoom);
    float leftBorder = linearCoords.x - (float)MAP_CENTER_X / (float)TILE_WIDTH;
    float topBorder = linearCoords.y - (float)MAP_CENTER_Y / (float)TILE_HEIGHT;

    if (borders) {
        borders->left = leftBorder;
        borders->top = topBorder;
        borders->lower = topBorder + (float)MAP_HEIGHT / (float)TILE_HEIGHT;
        borders->right = leftBorder + (float)MAP_WIDTH / (float)TILE_WIDTH;
    }

    linearCoords.x = leftBorder;
    linearCoords.y = topBorder;
    uint16_t curMapPixX = 0;
    uint16_t curMapPixY = 0;
    uint16_t printedWidth = 0;
    uint16_t printedHeight = 0;
    while (floorf(linearCoords.x) < leftBorder + (float)MAP_WIDTH / (float)TILE_WIDTH) {
        linearCoords.y = topBorder;
        printedWidth = 0;
        while (floorf(linearCoords.y) < topBorder + (float)MAP_HEIGHT / (float)TILE_HEIGHT) {
            char* fileName = getFileNameXY(linearCoords.x, linearCoords.y, zoom, tilesRootDir, imageFileExtesion);
            uint8_t* decomp = getFileContent(fileName);
            free(fileName);
            u16vec2 startingPoint = {
                .x = curMapPixX,
                .y = curMapPixY
            };
            u16vec2 point = pointOnMapXY(linearCoords.x, linearCoords.y);
            printedHeight = (uint16_t)TILE_HEIGHT - point.y;
            printedWidth = (uint16_t)TILE_WIDTH - point.x;
            while (point.x < (uint16_t)TILE_WIDTH && curMapPixX < (uint16_t)MAP_WIDTH) {
                while (point.y < (uint16_t)TILE_HEIGHT && curMapPixY < (uint16_t)MAP_HEIGHT) {
                    uint16_t pixel = colorPalette[decomp[point.y * TILE_WIDTH + point.x]];
                    // change endianess (to be deleted after editing lookup table)
                    pixel = ((pixel & 0xFF) << 8) | (pixel >> 8);
                    buffer[curMapPixY * MAP_WIDTH + curMapPixX] = pixel;
                    point.y++;
                    curMapPixY++;
                }
                curMapPixX++;
                point.x++;
                u16vec2 tmp = pointOnMapXY(linearCoords.x, linearCoords.y);
                point.y = tmp.y;
                curMapPixY = startingPoint.y;
            }
            curMapPixY += printedHeight;
            curMapPixX = startingPoint.x;
            linearCoords.y += (float)printedHeight / (float)TILE_HEIGHT;
            free(decomp);
        }
        curMapPixY = 0;
        curMapPixX += printedWidth;
        linearCoords.x += (float)printedWidth / (float)TILE_WIDTH;
    }
}


void UpdateWindowBuffer(
    uint16_t* buffer, 
    float lat, 
    float lon, 
    uint32_t zoom, 
    char* tilesRootDir, 
    char* imageFileExtesion, 
    const uint16_t* colorPalette, 
    mapBorders* borders
) {
    if (zoom < MIN_ZOOM || zoom > MAX_ZOOM) {
        return;
    }
    updateMap(buffer, lat, lon, zoom, tilesRootDir, imageFileExtesion, colorPalette, borders);
    // for (int i = 0; i < MAP_HEIGHT * MAP_WIDTH; i += MAP_WIDTH) {
    //     for (int j = 0; j < MAP_WIDTH / 2; j++) {
    //         uint16_t tmp = buffer[i + j];
    //         buffer[i + j] = buffer[i + MAP_WIDTH - 1 - j];
    //         buffer[i + MAP_WIDTH - 1 - j] = tmp;
    //     }
    // }
}