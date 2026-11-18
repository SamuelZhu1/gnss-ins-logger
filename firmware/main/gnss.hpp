// u-blox ZED-F9P over UART: configure it, parse UBX, hand NAV-PVT to the
// navigation task and log NAV-PVT and RXM-RAWX.
#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gins/ubx.hpp"

namespace gnss {

struct PvtMsg {
    int64_t arrival_us;  // local time the message was read from the UART
    gins::ubx::NavPvt pvt;
};

// Configures the receiver and returns the queue of PvtMsg for the nav task.
QueueHandle_t init();
void task(void*);

}  // namespace gnss
