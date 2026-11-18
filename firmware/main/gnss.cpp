#include "gnss.hpp"

#include "board.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "gins/log_format.hpp"
#include "logger.hpp"

namespace gnss {
namespace {
const char* TAG = "gnss";
QueueHandle_t g_queue = nullptr;
constexpr int kDefaultBaud = 38400;  // ZED-F9P factory setting

void send_config() {
    using namespace gins::ubx;
    const ConfigItem items[] = {
        {key::kRateMeas, 100},          // 10 Hz
        {key::kMsgNavPvtUart1, 1},      // NAV-PVT every epoch
        {key::kMsgRxmRawxUart1, 10},    // RAWX at 1 Hz for post-processing
        {key::kUart1OutNmea, 0},        // UBX only
        {key::kTpTimegridTp1, 1},       // PPS aligned to GPS time
        {key::kUart1Baud, static_cast<uint32_t>(CONFIG_GINS_GNSS_BAUD)},
    };
    uint8_t frame[128];
    const size_t n = build_valset(items, sizeof(items) / sizeof(items[0]), frame, sizeof(frame));
    uart_write_bytes(board::kGnssUart, frame, n);
    uart_wait_tx_done(board::kGnssUart, pdMS_TO_TICKS(100));
}
}  // namespace

QueueHandle_t init() {
    g_queue = xQueueCreate(8, sizeof(PvtMsg));
    uart_config_t cfg = {};
    cfg.baud_rate = kDefaultBaud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(board::kGnssUart, 8192, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(board::kGnssUart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(board::kGnssUart, board::kGnssTx, board::kGnssRx, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    // Configure at the factory baud rate, switch, then send again in case the
    // receiver was already at the new rate (ESP32 reset without a power cycle).
    send_config();
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_set_baudrate(board::kGnssUart, CONFIG_GINS_GNSS_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_config();
    ESP_LOGI(TAG, "receiver configured: 10 Hz NAV-PVT, 1 Hz RAWX, %d baud", CONFIG_GINS_GNSS_BAUD);
    return g_queue;
}

void task(void*) {
    gins::ubx::Parser parser;
    uint8_t buf[256];
    static uint8_t rec[gins::logfmt::kMaxPayload];  // one task, keep it off the stack
    for (;;) {
        const int n = uart_read_bytes(board::kGnssUart, buf, sizeof(buf), pdMS_TO_TICKS(5));
        if (n <= 0) continue;
        const int64_t arrival = esp_timer_get_time();
        for (int i = 0; i < n; ++i) {
            if (!parser.feed(buf[i])) continue;
            if (parser.cls() == gins::ubx::kClsNav && parser.id() == gins::ubx::kIdNavPvt) {
                PvtMsg m{arrival, {}};
                if (!gins::ubx::decode_nav_pvt(parser.payload(), parser.len(), m.pvt)) continue;
                xQueueSend(g_queue, &m, 0);

                const auto& p = m.pvt;
                gins::logfmt::Writer w(rec, sizeof(rec));
                w.i64(arrival).u32(p.itow_ms).i32(p.lat_e7).i32(p.lon_e7).i32(p.height_mm)
                    .i32(p.vel_n_mm_s).i32(p.vel_e_mm_s).i32(p.vel_d_mm_s)
                    .u32(p.h_acc_mm).u32(p.v_acc_mm).u32(p.s_acc_mm_s)
                    .u8((p.flags & 0x01) ? p.fix_type : 0).u8(p.carr_soln).u8(p.num_sv).u8(0);
                logger::write(gins::logfmt::Type::kGnssPvt, rec, static_cast<uint16_t>(w.size()));
            } else if (parser.cls() == gins::ubx::kClsRxm && parser.id() == gins::ubx::kIdRxmRawx) {
                gins::logfmt::Writer w(rec, sizeof(rec));
                w.i64(arrival).bytes(parser.payload(), parser.len());
                if (w.ok()) {
                    logger::write(gins::logfmt::Type::kRawx, rec, static_cast<uint16_t>(w.size()));
                }
            }
        }
    }
}

}  // namespace gnss
