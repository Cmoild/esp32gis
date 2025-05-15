#include <utils.h>
#include <math.h>

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define EARTH_RADIUS_M 6371000.0

void lstdir(char* path) {
    DIR *dp;
    struct dirent *ep;
    dp = opendir (path);
    if (dp != NULL)
    {
        while ((ep = readdir (dp)) != NULL)
        puts (ep->d_name);
        (void) closedir (dp);
        return;
    }
    else
    {
        perror ("Couldn't open the directory");
        return;
    }
}


void print_memory_info() {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    printf("\n[DRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_EXEC);
    printf("[IRAM]   Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);

    heap_caps_get_info(&info, MALLOC_CAP_DMA);
    printf("[DMA-capable] Free: %u bytes, Largest block: %u bytes\n", info.total_free_bytes, info.largest_free_block);
}


int get_gps_line(char* buffer, enum GPS_message_status* status, char* gps_line_buffer, int gps_line_len) {
    if (gps_line_buffer[0] && *status != MESSAGE_ENDED && *status != EMPTY_STATUS) {
        int i = 0;
        while (i < GPS_BUFFER_LENGTH && buffer[i] != '\r') {
            gps_line_buffer[i + gps_line_len] = buffer[i];
            i++;
        }
        *status = MESSAGE_ENDED;
        return gps_line_len + i;
    }

    if (!gps_line_buffer[0]) {
        gps_line_len = 0;
        int i = 0;
        char* target_tag = "$??GGA,";
        while (i < GPS_BUFFER_LENGTH) {
            if (buffer[i] == '$') {
                bool is_target = true;
                for (int j = 0; j < 7; j++) {
                    if (i + j >= GPS_BUFFER_LENGTH) {
                        *status = CHECK_TAG;
                    }
                    if ((j != 1 && j != 2) && buffer[i + j] != target_tag[j]) {
                        is_target = false;
                        break;
                    }
                }
                if (is_target) {
                    while (i + gps_line_len < 100 && buffer[i + gps_line_len] != '\n') {
                        gps_line_buffer[gps_line_len] = buffer[i + gps_line_len];
                        gps_line_len++;
                    }
                    if (buffer[i + gps_line_len] == '\n') {
                        *status = MESSAGE_ENDED;
                    }
                    else {
                        *status = MESSAGE_STARTED;
                    }
                    return gps_line_len;
                }
            }
            i++;
        }
    }

    return 0;
}


gps_data_t parse_gps_line(char* line, int len, int time_zone_offset) {
    gps_data_t data = {
        .lat = 0,
        .lon = 0,
        .altitude = 0,
        .hoursMinutes = {'0', '0', ':', '0', '0', 0},
        .is_valid = false,
    };
    if (len < 6) return data;
    if (line[0] != '$' && line[3] != 'G' && line[4] != 'G' && line[5] != 'A') return data;
    int comma_count = 0;
    bool is_valid = false;
    for (int i = 0; i < len; i++) {
        if (comma_count == 6 && !is_valid) {
            is_valid = (line[i] - '0' == 1) ? true : false;
        }
        if (line[i] == ',') comma_count++;
    }
    if (comma_count != 14) return data;
    if (!is_valid) return data;

    data.is_valid = is_valid;

    int i = 0;
    comma_count = 0;
    
    while (comma_count < 14 && i < len) {
        if (line[i] == ',') {
            comma_count++;
            i++;
        }
        switch (comma_count)
        {
        case 1:
            //parse time here
            int hrs = (line[i] - '0') * 10 + (line[i + 1]);
            hrs = (hrs + time_zone_offset) % 24;
            data.hoursMinutes[0] = hrs / 10 + '0';
            data.hoursMinutes[1] = hrs % 10 + '0';
            data.hoursMinutes[3] = line[i + 2];
            data.hoursMinutes[4] = line[i + 3];
            i += 10;
            break;

        case 2:
            float lat = (line[i] - '0') * 10 + (line[i + 1] - '0');
            char mins_buf_lat[8] = {line[i + 2], line[i + 3], line[i + 4], line[i + 5], line[i + 6], line[i + 7], line[i + 8], 0};
            lat += (float)atof(mins_buf_lat) / 60;
            i += 10;
            comma_count++;
            lat = (line[i] == 'N') ? lat : -1. * lat;
            i++;
            data.lat = lat;
            break;

        case 4:
            float lon = (line[i] - '0') * 100 + (line[i + 1] - '0') * 10 + (line[i + 2] - '0');
            char mins_buf_lon[8] = {line[i + 3], line[i + 4], line[i + 5], line[i + 6], line[i + 7], line[i + 8], line[i + 9], 0};
            lon += (float)atof(mins_buf_lon) / 60;
            i += 11;
            comma_count++;
            lon = (line[i] == 'E') ? lon : -1. * lon;
            i++;
            data.lon = lon;
            break;

        // case 9:
        //     // parse altitude here
        //     break;
        
        default:
            while (line[i] != ',' && i < len) i++;
            break;
        }
    }

    return data;
}

float calculate_speed_kmh(gps_data_t p1, gps_data_t p2) {
    float lat1_rad = DEG_TO_RAD(p1.lat);
    float lon1_rad = DEG_TO_RAD(p1.lon);
    float lat2_rad = DEG_TO_RAD(p2.lat);
    float lon2_rad = DEG_TO_RAD(p2.lon);

    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;

    float R = EARTH_RADIUS_M + (p1.altitude + p2.altitude) / 2.0;

    float a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    float horizontal_distance = R * c;

    float dz = p2.altitude - p1.altitude;

    float distance = sqrt(horizontal_distance * horizontal_distance + dz * dz);

    float dt = p2.timestamp - p1.timestamp;
    if (dt <= 0.0) return 0.0;

    return distance / dt * 3.6;
}