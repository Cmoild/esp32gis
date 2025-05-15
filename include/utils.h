#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#define GPS_BUFFER_LENGTH 100

enum GPS_message_status {
    MESSAGE_ENDED = 0,
    MESSAGE_STARTED = 1,
    CHECK_TAG = 2,
    EMPTY_STATUS = 3,
};

typedef struct {
    float lat;
    float lon;
    float altitude;
    char hoursMinutes[6]; // 00:00 + \0
    bool is_valid;
    float timestamp;
} gps_data_t;

void lstdir(char* path);

void print_memory_info();

int get_gps_line(char* buffer, enum GPS_message_status* status, char* gps_line_buffer, int gps_line_len);

gps_data_t parse_gps_line(char* line, int len, int time_zone_offset);

float calculate_speed_kmh(gps_data_t p1, gps_data_t p2);

#endif