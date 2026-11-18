#include "nav_task.hpp"

#include <cmath>

#include "board.hpp"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gins/log_format.hpp"
#include "gins/navigator.hpp"
#include "gins/rotation.hpp"
#include "gins/time_base.hpp"
#include "gnss.hpp"
#include "icm42688.hpp"
#include "logger.hpp"
#include "status.hpp"

namespace nav {

std::atomic<int32_t> g_mask_request_s{0};
std::atomic<bool> g_realign_request{false};

namespace {
const char* TAG = "nav";
TaskHandle_t g_task = nullptr;
volatile int64_t g_irq_time = 0;

// Large objects live in static storage, not on the task stack.
gins::Navigator* g_nav = nullptr;
gins::TimeBase g_time;

void IRAM_ATTR imu_isr(void*) {
    g_irq_time = esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

gins::NavConfig make_config() {
    gins::NavConfig c;
    c.align_us = static_cast<int64_t>(CONFIG_GINS_ALIGN_S) * 1'000'000;
    c.lag_us = 100'000;
    c.output_period_us = 100'000;
    return c;
}

void on_output(const gins::NavOutput& o, void*) {
    uint8_t p[68];
    gins::logfmt::Writer w(p, sizeof(p));
    w.i64(o.t_us).f64(o.lla.lat_deg).f64(o.lla.lon_deg).f32(static_cast<float>(o.lla.h_m))
        .f32(o.v_ned.x()).f32(o.v_ned.y()).f32(o.v_ned.z())
        .f32(o.rpy.x()).f32(o.rpy.y()).f32(o.rpy.z())
        .f32(o.sigma_pos.x()).f32(o.sigma_pos.y()).f32(o.sigma_pos.z())
        .u8(static_cast<uint8_t>(o.mode)).zeros(3);
    logger::write(gins::logfmt::Type::kFilter, p, static_cast<uint16_t>(w.size()));

    status::edit([&](Status& s) {
        s.t_gps_us = o.t_us;
        s.lat = o.lla.lat_deg;
        s.lon = o.lla.lon_deg;
        s.h = static_cast<float>(o.lla.h_m);
        s.yaw_deg = static_cast<float>(o.rpy.z() / gins::kDeg);
        s.speed = std::hypot(o.v_ned.x(), o.v_ned.y());
        s.sig_h = std::hypot(o.sigma_pos.x(), o.sigma_pos.y());
    });
}

void log_imu(int64_t t_local, const ImuRaw& r) {
    uint8_t p[22];
    gins::logfmt::Writer w(p, sizeof(p));
    w.i64(t_local);
    for (int i = 0; i < 3; ++i) w.i16(r.accel[i]);
    for (int i = 0; i < 3; ++i) w.i16(r.gyro[i]);
    w.i16(r.temp);
    logger::write(gins::logfmt::Type::kImu, p, static_cast<uint16_t>(w.size()));
}

void log_pps(int64_t t_local) {
    uint8_t p[8];
    gins::logfmt::Writer w(p, sizeof(p));
    w.i64(t_local);
    logger::write(gins::logfmt::Type::kPps, p, static_cast<uint16_t>(w.size()));
}
}  // namespace

TaskHandle_t handle() { return g_task; }

void attach_imu_interrupt(TaskHandle_t task) {
    g_task = task;
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << board::kImuInt;
    io.mode = GPIO_MODE_INPUT;
    io.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io);
    gpio_isr_handler_add(board::kImuInt, imu_isr, nullptr);
}

void task(void* arg) {
    auto* ctx = static_cast<Context*>(arg);
    static gins::Navigator nav(make_config());
    g_nav = &nav;
    nav.set_output(on_output, nullptr);

    // Sensor frame to bike frame, from menuconfig.
    const gins::Mat3 r_bs = gins::quat_from_rpy(
        static_cast<gins::real>(CONFIG_GINS_MOUNT_ROLL_DEG * gins::kDeg),
        static_cast<gins::real>(CONFIG_GINS_MOUNT_PITCH_DEG * gins::kDeg),
        static_cast<gins::real>(CONFIG_GINS_MOUNT_YAW_DEG * gins::kDeg)).toRotationMatrix();

    bool have_time = false;
    gins::Mode last_mode = nav.mode();
    uint32_t samples = 0, missed = 0, overruns = 0, max_us = 0;
    int64_t busy_us = 0;
    int64_t window_start = esp_timer_get_time();
    uint32_t window_samples = 0;

    for (;;) {
        const uint32_t pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (pending == 0) {
            ++missed;  // no data-ready for 20 ms: four samples missing
            continue;
        }
        if (pending > 1) overruns += pending - 1;  // we were late and samples were overwritten
        const int64_t t_local = g_irq_time;
        const int64_t t0 = esp_timer_get_time();

        ImuRaw raw;
        if (ctx->imu->read(raw) != ESP_OK) continue;
        log_imu(t_local, raw);
        ++samples;
        ++window_samples;

        int64_t pps_t;
        while (xQueueReceive(ctx->pps_queue, &pps_t, 0) == pdTRUE) {
            g_time.on_pps(pps_t);
            log_pps(pps_t);
        }
        gnss::PvtMsg m;
        while (xQueueReceive(ctx->pvt_queue, &m, 0) == pdTRUE) {
            const int64_t epoch = g_time.on_pvt(m.arrival_us, m.pvt.itow_ms);
#if CONFIG_GINS_STAMP_FIX_ON_ARRIVAL
            const int64_t t_fix = g_time.to_gps_us(m.arrival_us);  // the timing bug, for experiments
#else
            const int64_t t_fix = epoch;
#endif
            nav.push_fix(gins::ubx::to_fix(m.pvt, t_fix));
            have_time = true;
            status::edit([&](Status& s) {
                s.fix_type = (m.pvt.flags & 0x01) ? m.pvt.fix_type : 0;
                s.carr_soln = m.pvt.carr_soln;
                s.num_sv = m.pvt.num_sv;
                s.fix_hacc = static_cast<float>(m.pvt.h_acc_mm) * 1e-3f;
            });
        }

        if (have_time) {
            gins::ImuSample s;
            s.t_us = g_time.to_gps_us(t_local);
            const gins::Vec3 a(raw.accel[0] * Icm42688::kAccelScale, raw.accel[1] * Icm42688::kAccelScale,
                               raw.accel[2] * Icm42688::kAccelScale);
            const gins::Vec3 g(raw.gyro[0] * Icm42688::kGyroScale, raw.gyro[1] * Icm42688::kGyroScale,
                               raw.gyro[2] * Icm42688::kGyroScale);
            s.accel = r_bs * a;
            s.gyro = r_bs * g;

            const int32_t mask_s = g_mask_request_s.exchange(0);
            if (mask_s > 0) {
                nav.add_gnss_mask(s.t_us, s.t_us + static_cast<int64_t>(mask_s) * 1'000'000);
                logger::event(gins::logfmt::Event::kGnssMaskStart, static_cast<uint32_t>(mask_s));
            }
            if (g_realign_request.exchange(false)) nav.restart_alignment();

            nav.push_imu(s);
            nav.process();
            if (nav.mode() != last_mode) {
                last_mode = nav.mode();
                logger::event(gins::logfmt::Event::kModeChange, static_cast<uint32_t>(last_mode));
                ESP_LOGI(TAG, "mode %s", gins::mode_name(last_mode));
            }
        }

        const int64_t work = esp_timer_get_time() - t0;
        busy_us += work;
        if (work > max_us) max_us = static_cast<uint32_t>(work);
        if (work > 5000) logger::event(gins::logfmt::Event::kFilterOverrun, static_cast<uint32_t>(work));

        const int64_t now = esp_timer_get_time();
        if (now - window_start >= 1'000'000) {
            const float secs = static_cast<float>(now - window_start) * 1e-6f;
            const auto& fd = nav.filter().diag();
            status::edit([&](Status& st) {
                st.mode = nav.mode();
                st.imu_samples = samples;
                st.imu_hz = static_cast<float>(window_samples) / secs;
                st.missed_irq = missed;
                st.overruns = overruns;
                st.filter_cpu_pct = 100.0f * static_cast<float>(busy_us) * 1e-6f / secs;
                st.filter_max_us = max_us;
                st.pps_locked = g_time.locked(now);
                st.fix_latency_ms = static_cast<float>(g_time.mean_latency_ms());
                st.drift_ppm = static_cast<float>(g_time.drift_ppm());
                st.gnss_accepted = fd.gnss_accepted;
                st.gnss_rejected = fd.gnss_rejected;
            });
            window_start = now;
            window_samples = 0;
            busy_us = 0;
            max_us = 0;
        }
    }
}

}  // namespace nav
