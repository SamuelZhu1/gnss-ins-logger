#include "pps.hpp"

#include "esp_attr.h"
#include "esp_timer.h"

namespace pps {
namespace {
QueueHandle_t g_queue = nullptr;

void IRAM_ATTR isr(void*) {
    const int64_t t = esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(g_queue, &t, &woken);
    if (woken) portYIELD_FROM_ISR();
}
}  // namespace

QueueHandle_t init(gpio_num_t pin) {
    g_queue = xQueueCreate(8, sizeof(int64_t));
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << pin;
    io.mode = GPIO_MODE_INPUT;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io);
    gpio_isr_handler_add(pin, isr, nullptr);
    return g_queue;
}

}  // namespace pps
