// GNSS/INS dead reckoning logger, ESP32-S3.
//
// Tasks (core, priority):
//   nav     (1, 20)  IMU data-ready -> time base -> navigator, logs IMU/PPS/FILTER
//   gnss    (0, 15)  UART -> UBX parser -> NAV-PVT queue, logs NAV-PVT/RAWX
//   logger  (0, 5)   ring buffer -> SD card in 4 KB blocks
//   shell   (0, 3)   USB Serial/JTAG commands
#include "board.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gnss.hpp"
#include "icm42688.hpp"
#include "logger.hpp"
#include "nav_task.hpp"
#include "pps.hpp"
#include "shell.hpp"
#include "status.hpp"

namespace {
const char* TAG = "main";
Icm42688 g_imu;
nav::Context g_nav_ctx;
}  // namespace

extern "C" void app_main(void) {
    status::init();
    gpio_install_isr_service(0);

    const bool card = logger::init();
    if (!card) ESP_LOGW(TAG, "no SD card: navigating without logging");

    if (g_imu.init(board::kImuSpi, board::kImuSclk, board::kImuMosi, board::kImuMiso, board::kImuCs) != ESP_OK) {
        ESP_LOGE(TAG, "IMU not found; check wiring");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    g_nav_ctx.imu = &g_imu;
    g_nav_ctx.pvt_queue = gnss::init();
    g_nav_ctx.pps_queue = pps::init(board::kPps);

    if (card) xTaskCreatePinnedToCore(logger::task, "logger", 6144, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(gnss::task, "gnss", 6144, nullptr, 15, nullptr, 0);
    TaskHandle_t nav_handle = nullptr;
    xTaskCreatePinnedToCore(nav::task, "nav", 16384, &g_nav_ctx, 20, &nav_handle, 1);
    nav::attach_imu_interrupt(nav_handle);
    xTaskCreatePinnedToCore(shell::task, "shell", 4096, nullptr, 3, nullptr, 0);

#if CONFIG_GINS_LOG_ON_BOOT
    if (card) logger::start();
#endif
    ESP_LOGI(TAG, "running");
}
