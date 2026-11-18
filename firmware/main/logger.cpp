#include "logger.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstring>

#include "board.hpp"
#include "driver/sdspi_host.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "icm42688.hpp"
#include "sdmmc_cmd.h"
#include "status.hpp"

namespace logger {
namespace {
const char* TAG = "logger";
constexpr char kMount[] = "/sdcard";
constexpr size_t kBlock = 4096;
constexpr size_t kRingBytes = CONFIG_GINS_RING_KB * 1024;

RingbufHandle_t g_ring = nullptr;
SemaphoreHandle_t g_file_mutex = nullptr;
FILE* g_file = nullptr;
std::atomic<bool> g_active{false};
bool g_card = false;
char g_name[20] = "";

std::atomic<uint32_t> g_seq[0x11];
std::atomic<uint32_t> g_dropped[0x11];
std::atomic<uint32_t> g_high_water{0};
uint32_t g_stall_max_ms = 0;
uint32_t g_bytes = 0;

bool mount() {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = board::kSdSpi;
    host.max_freq_khz = 20000;
    spi_bus_config_t bus = {};
    bus.mosi_io_num = board::kSdMosi;
    bus.miso_io_num = board::kSdMiso;
    bus.sclk_io_num = board::kSdSclk;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 8192;
    if (spi_bus_initialize(board::kSdSpi, &bus, SDSPI_DEFAULT_DMA) != ESP_OK) return false;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = board::kSdCs;
    slot.host_id = board::kSdSpi;

    esp_vfs_fat_sdmmc_mount_config_t mc = {};
    mc.format_if_mount_failed = false;
    mc.max_files = 2;
    mc.allocation_unit_size = 32 * 1024;  // bigger clusters, fewer FAT updates
    sdmmc_card_t* card = nullptr;
    const esp_err_t err = esp_vfs_fat_sdspi_mount(kMount, &host, &slot, &mc, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        return false;
    }
    sdmmc_card_print_info(stdout, card);
    return true;
}

void write_to_card(const void* data, size_t n) {
    xSemaphoreTake(g_file_mutex, portMAX_DELAY);
    if (g_file) {
        const int64_t t0 = esp_timer_get_time();
        fwrite(data, 1, n, g_file);
        const uint32_t ms = static_cast<uint32_t>((esp_timer_get_time() - t0) / 1000);
        g_bytes += static_cast<uint32_t>(n);
        if (ms > g_stall_max_ms) g_stall_max_ms = ms;
        xSemaphoreGive(g_file_mutex);
        if (ms >= 50) event(gins::logfmt::Event::kSdStallMs, ms);
        return;
    }
    xSemaphoreGive(g_file_mutex);
}

void write_header() {
    uint8_t p[64];
    gins::logfmt::Writer w(p, sizeof(p));
    char ver[16] = {};
    std::strncpy(ver, esp_app_get_description()->version, sizeof(ver) - 1);
    w.bytes(reinterpret_cast<const uint8_t*>(ver), sizeof(ver))
        .f32(Icm42688::kAccelScale).f32(Icm42688::kGyroScale)
        .f32(Icm42688::kTempScale).f32(Icm42688::kTempOffset)
        .f32(static_cast<float>(CONFIG_GINS_MOUNT_ROLL_DEG))
        .f32(static_cast<float>(CONFIG_GINS_MOUNT_PITCH_DEG))
        .f32(static_cast<float>(CONFIG_GINS_MOUNT_YAW_DEG))
        .u32(board::kImuRateHz).u32(0);
    write(gins::logfmt::Type::kHeader, p, static_cast<uint16_t>(w.size()));
}
}  // namespace

bool init() {
    g_file_mutex = xSemaphoreCreateMutex();
    g_ring = xRingbufferCreate(kRingBytes, RINGBUF_TYPE_BYTEBUF);
    g_card = mount();
    return g_card && g_ring != nullptr;
}

bool start() {
    if (!g_card || g_active) return g_active;
    for (int i = 1; i < 10000; ++i) {
        char name[20];
        std::snprintf(name, sizeof(name), "%s/R%04d.BIN", kMount, i);
        struct stat st;
        if (stat(name, &st) != 0) {
            std::strncpy(g_name, name, sizeof(g_name) - 1);
            break;
        }
    }
    xSemaphoreTake(g_file_mutex, portMAX_DELAY);
    g_file = std::fopen(g_name, "wb");
    if (g_file) setvbuf(g_file, nullptr, _IOFBF, kBlock);
    xSemaphoreGive(g_file_mutex);
    if (!g_file) {
        ESP_LOGE(TAG, "cannot open %s", g_name);
        return false;
    }
    g_active = true;
    write_header();
    event(gins::logfmt::Event::kLogStart, 0);
    ESP_LOGI(TAG, "logging to %s", g_name);
    status::edit([](Status& s) {
        s.logging = true;
        std::strncpy(s.file, g_name, sizeof(s.file) - 1);
    });
    return true;
}

void stop() {
    g_active = false;
    vTaskDelay(pdMS_TO_TICKS(300));  // let the task drain what's queued
    xSemaphoreTake(g_file_mutex, portMAX_DELAY);
    if (g_file) {
        std::fflush(g_file);
        fsync(fileno(g_file));
        std::fclose(g_file);
        g_file = nullptr;
    }
    xSemaphoreGive(g_file_mutex);
    status::edit([](Status& s) { s.logging = false; });
}

bool active() { return g_active; }

bool write(gins::logfmt::Type type, const uint8_t* payload, uint16_t len) {
    const uint8_t t = static_cast<uint8_t>(type);
    const uint32_t seq = g_seq[t].fetch_add(1);  // assigned even if dropped, so gaps show
    if (!g_active) return false;

    uint8_t buf[gins::logfmt::kMaxPayload + gins::logfmt::kOverhead];
    const size_t n = gins::logfmt::encode(type, seq, payload, len, buf, sizeof(buf));
    if (n == 0) return false;

#if CONFIG_GINS_LOG_SYNC_WRITES
    // Experiment: the first design wrote IMU records straight to the card from
    // the filter task, so every SD stall dropped samples.
    if (type == gins::logfmt::Type::kImu) {
        write_to_card(buf, n);
        return true;
    }
#endif
    const size_t free_bytes = xRingbufferGetCurFreeSize(g_ring);
    if (type == gins::logfmt::Type::kRawx && free_bytes < kRingBytes / 10) {
        g_dropped[t].fetch_add(1);  // raw measurements matter least; shed them first
        return false;
    }
    if (xRingbufferSend(g_ring, buf, n, 0) != pdTRUE) {
        g_dropped[t].fetch_add(1);
        return false;
    }
    const uint32_t used = static_cast<uint32_t>(kRingBytes - xRingbufferGetCurFreeSize(g_ring));
    uint32_t hw = g_high_water.load();
    while (used > hw && !g_high_water.compare_exchange_weak(hw, used)) {
    }
    return true;
}

void event(gins::logfmt::Event code, uint32_t value) {
    uint8_t p[16];
    gins::logfmt::Writer w(p, sizeof(p));
    w.i64(esp_timer_get_time()).u16(static_cast<uint16_t>(code)).u16(0).u32(value);
    write(gins::logfmt::Type::kEvent, p, static_cast<uint16_t>(w.size()));
}

uint32_t dropped(gins::logfmt::Type type) { return g_dropped[static_cast<uint8_t>(type)].load(); }
uint32_t high_water() { return g_high_water.load(); }

void task(void*) {
    int64_t last_sync = esp_timer_get_time();
    int64_t last_report = last_sync;
    for (;;) {
        size_t size = 0;
        void* item = xRingbufferReceiveUpTo(g_ring, &size, pdMS_TO_TICKS(100), kBlock);
        if (item) {
            write_to_card(item, size);
            vRingbufferReturnItem(g_ring, item);
        }
        const int64_t now = esp_timer_get_time();
        if (now - last_sync > 1'000'000) {
            last_sync = now;
            xSemaphoreTake(g_file_mutex, portMAX_DELAY);
            if (g_file) {
                std::fflush(g_file);
                fsync(fileno(g_file));
            }
            xSemaphoreGive(g_file_mutex);
            status::edit([](Status& s) {
                s.dropped_imu = dropped(gins::logfmt::Type::kImu);
                s.dropped_rawx = dropped(gins::logfmt::Type::kRawx);
                s.ring_high_water = high_water();
                s.sd_stall_max_ms = g_stall_max_ms;
                s.bytes_written = g_bytes;
            });
        }
        if (now - last_report > 10'000'000) {
            last_report = now;
            event(gins::logfmt::Event::kRingHighWater, high_water());
            event(gins::logfmt::Event::kDroppedImu, dropped(gins::logfmt::Type::kImu));
            event(gins::logfmt::Event::kDroppedRawx, dropped(gins::logfmt::Type::kRawx));
        }
    }
}

}  // namespace logger
