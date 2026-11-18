// Status shared between the navigation task (writer) and the shell (reader).
#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "gins/types.hpp"

struct Status {
    gins::Mode mode = gins::Mode::kWaitFix;
    int64_t t_gps_us = 0;
    double lat = 0, lon = 0;
    float h = 0, yaw_deg = 0, speed = 0, sig_h = 0;

    uint8_t fix_type = 0, carr_soln = 0, num_sv = 0;
    float fix_hacc = 0;
    bool pps_locked = false;
    float fix_latency_ms = 0;
    float drift_ppm = 0;

    uint32_t imu_samples = 0;
    float imu_hz = 0;
    uint32_t missed_irq = 0;
    uint32_t overruns = 0;
    float filter_cpu_pct = 0;
    uint32_t filter_max_us = 0;
    uint32_t gnss_accepted = 0, gnss_rejected = 0;

    bool logging = false;
    char file[20] = "";
    uint32_t dropped_imu = 0, dropped_rawx = 0;
    uint32_t ring_high_water = 0;
    uint32_t sd_stall_max_ms = 0;
    uint32_t bytes_written = 0;
};

namespace status {

void init();
extern Status g_status;
extern SemaphoreHandle_t g_mutex;

template <typename F>
void edit(F&& f) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    f(g_status);
    xSemaphoreGive(g_mutex);
}

inline Status get() {
    Status s;
    edit([&](Status& g) { s = g; });
    return s;
}

}  // namespace status
