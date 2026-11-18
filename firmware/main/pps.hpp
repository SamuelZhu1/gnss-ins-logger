// PPS input: the receiver's timepulse marks the start of each GPS second.
#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace pps {
// Queue of int64_t local timestamps (us) of rising edges.
QueueHandle_t init(gpio_num_t pin);
}
