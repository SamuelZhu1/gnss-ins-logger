// Navigation task (core 1, highest app priority): woken by the IMU data-ready
// interrupt at 200 Hz, it reads the sample, maps local time to GPS time,
// feeds the navigator and logs IMU, PPS and filter records.
#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class Icm42688;

namespace nav {

struct Context {
    Icm42688* imu;
    QueueHandle_t pps_queue;
    QueueHandle_t pvt_queue;
};

void task(void* ctx);
TaskHandle_t handle();
// Installs the data-ready ISR that wakes the given task.
void attach_imu_interrupt(TaskHandle_t task);

// Requests from the shell, picked up by the task.
extern std::atomic<int32_t> g_mask_request_s;
extern std::atomic<bool> g_realign_request;

}  // namespace nav
