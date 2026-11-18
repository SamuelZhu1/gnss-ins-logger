#include "shell.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "gins/types.hpp"
#include "logger.hpp"
#include "nav_task.hpp"
#include "status.hpp"

namespace shell {
namespace {

void print_status() {
    const Status s = status::get();
    std::printf(
        "{\"mode\":\"%s\",\"fix\":%u,\"carr\":%u,\"sats\":%u,\"hacc\":%.3f,\"pps_locked\":%s,"
        "\"fix_latency_ms\":%.1f,\"drift_ppm\":%.2f,\"imu_hz\":%.1f,\"imu_samples\":%lu,"
        "\"missed_irq\":%lu,\"overruns\":%lu,\"filter_cpu_pct\":%.1f,\"filter_max_us\":%lu,"
        "\"gnss_accepted\":%lu,\"gnss_rejected\":%lu,\"logging\":%s,\"file\":\"%s\","
        "\"dropped_imu\":%lu,\"dropped_rawx\":%lu,\"ring_high_water\":%lu,\"sd_stall_max_ms\":%lu,"
        "\"bytes\":%lu,\"uptime_s\":%.1f}\n",
        gins::mode_name(s.mode), s.fix_type, s.carr_soln, s.num_sv, s.fix_hacc,
        s.pps_locked ? "true" : "false", s.fix_latency_ms, s.drift_ppm, s.imu_hz,
        (unsigned long)s.imu_samples, (unsigned long)s.missed_irq, (unsigned long)s.overruns,
        s.filter_cpu_pct, (unsigned long)s.filter_max_us, (unsigned long)s.gnss_accepted,
        (unsigned long)s.gnss_rejected, s.logging ? "true" : "false", s.file,
        (unsigned long)s.dropped_imu, (unsigned long)s.dropped_rawx,
        (unsigned long)s.ring_high_water, (unsigned long)s.sd_stall_max_ms,
        (unsigned long)s.bytes_written, esp_timer_get_time() * 1e-6);
}

void print_pose() {
    const Status s = status::get();
    std::printf("{\"t_gps_us\":%lld,\"lat\":%.8f,\"lon\":%.8f,\"h\":%.2f,\"yaw_deg\":%.1f,"
                "\"speed\":%.2f,\"sig_h\":%.2f,\"mode\":\"%s\"}\n",
                (long long)s.t_gps_us, s.lat, s.lon, s.h, s.yaw_deg, s.speed, s.sig_h,
                gins::mode_name(s.mode));
}

void handle(char* line) {
    line[std::strcspn(line, "\r\n")] = 0;
    if (std::strcmp(line, "status") == 0) {
        print_status();
    } else if (std::strcmp(line, "pose") == 0) {
        print_pose();
    } else if (std::strcmp(line, "log start") == 0) {
        std::printf("{\"ok\":%s}\n", logger::start() ? "true" : "false");
    } else if (std::strcmp(line, "log stop") == 0) {
        logger::stop();
        std::printf("{\"ok\":true}\n");
    } else if (std::strncmp(line, "mask gnss ", 10) == 0) {
        const int s = std::atoi(line + 10);
        if (s <= 0 || s > 600) {
            std::printf("{\"error\":\"mask length must be 1-600 s\"}\n");
            return;
        }
        nav::g_mask_request_s = s;
        std::printf("{\"ok\":true,\"mask_s\":%d}\n", s);
    } else if (std::strcmp(line, "cal static") == 0) {
        nav::g_realign_request = true;
        std::printf("{\"ok\":true,\"note\":\"hold the bike still\"}\n");
    } else if (std::strcmp(line, "version") == 0) {
        const esp_app_desc_t* d = esp_app_get_description();
        std::printf("{\"version\":\"%s\",\"idf\":\"%s\",\"built\":\"%s %s\"}\n", d->version, d->idf_ver,
                    d->date, d->time);
    } else if (line[0] != 0) {
        std::printf("{\"error\":\"unknown command\"}\n");
    }
}

}  // namespace

void task(void*) {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&cfg);
    esp_vfs_usb_serial_jtag_use_driver();  // blocking reads on stdin
    setvbuf(stdin, nullptr, _IONBF, 0);
    char line[96];
    for (;;) {
        if (std::fgets(line, sizeof(line), stdin)) {
            handle(line);
            std::fflush(stdout);
        }
    }
}

}  // namespace shell
