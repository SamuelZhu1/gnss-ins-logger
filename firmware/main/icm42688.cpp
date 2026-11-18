#include "icm42688.hpp"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
const char* TAG = "icm42688";

// Bank 0 registers (ICM-42688-P datasheet, section 14)
constexpr uint8_t kDeviceConfig = 0x11;
constexpr uint8_t kIntConfig = 0x14;
constexpr uint8_t kTempData1 = 0x1D;
constexpr uint8_t kPwrMgmt0 = 0x4E;
constexpr uint8_t kGyroConfig0 = 0x4F;
constexpr uint8_t kAccelConfig0 = 0x50;
constexpr uint8_t kIntConfig1 = 0x64;
constexpr uint8_t kIntSource0 = 0x65;
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kBankSel = 0x76;
constexpr uint8_t kWhoAmIValue = 0x47;

constexpr uint8_t kOdr200Hz = 0x07;
constexpr uint8_t kGyroFs500 = 2 << 5;
constexpr uint8_t kAccelFs8g = 1 << 5;

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }
}  // namespace

esp_err_t Icm42688::init(spi_host_device_t host, int sclk, int mosi, int miso, int cs) {
    spi_bus_config_t bus = {};
    bus.sclk_io_num = sclk;
    bus.mosi_io_num = mosi;
    bus.miso_io_num = miso;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 64;
    esp_err_t err = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    spi_device_interface_config_t dc = {};
    dc.address_bits = 8;           // register address, bit 7 = read
    dc.mode = 0;
    dc.clock_speed_hz = 10 * 1000 * 1000;
    dc.spics_io_num = cs;
    dc.queue_size = 2;
    err = spi_bus_add_device(host, &dc, &dev_);
    if (err != ESP_OK) return err;

    uint8_t who = 0;
    if ((err = write_reg(kBankSel, 0)) != ESP_OK) return err;
    if ((err = read_regs(kWhoAmI, &who, 1)) != ESP_OK) return err;
    if (who != kWhoAmIValue) {
        ESP_LOGE(TAG, "WHO_AM_I 0x%02x, expected 0x%02x", who, kWhoAmIValue);
        return ESP_ERR_NOT_FOUND;
    }

    write_reg(kDeviceConfig, 0x01);  // soft reset
    vTaskDelay(pdMS_TO_TICKS(2));
    write_reg(kBankSel, 0);
    write_reg(kIntConfig, 0x03);     // INT1 push-pull, active high, pulsed
    write_reg(kIntConfig1, 0x00);    // clear INT_ASYNC_RESET as the datasheet requires
    write_reg(kIntSource0, 0x08);    // data ready on INT1
    write_reg(kGyroConfig0, kGyroFs500 | kOdr200Hz);
    write_reg(kAccelConfig0, kAccelFs8g | kOdr200Hz);
    write_reg(kPwrMgmt0, 0x0F);      // gyro and accel in low-noise mode
    vTaskDelay(pdMS_TO_TICKS(50));   // gyro start-up time is about 30 ms
    ESP_LOGI(TAG, "ready: 200 Hz, +-8 g, +-500 dps");
    return ESP_OK;
}

esp_err_t Icm42688::write_reg(uint8_t reg, uint8_t val) {
    spi_transaction_t t = {};
    t.addr = reg;
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = val;
    return spi_device_polling_transmit(dev_, &t);
}

esp_err_t Icm42688::read_regs(uint8_t reg, uint8_t* out, int n) {
    spi_transaction_t t = {};
    t.addr = static_cast<uint64_t>(reg | 0x80);
    t.length = static_cast<size_t>(8 * n);
    t.rxlength = static_cast<size_t>(8 * n);
    t.rx_buffer = rx_;
    const esp_err_t err = spi_device_polling_transmit(dev_, &t);
    if (err == ESP_OK) std::memcpy(out, rx_, static_cast<size_t>(n));
    return err;
}

esp_err_t Icm42688::read(ImuRaw& out) {
    uint8_t b[14];
    const esp_err_t err = read_regs(kTempData1, b, sizeof(b));
    if (err != ESP_OK) return err;
    out.temp = be16(b);
    for (int i = 0; i < 3; ++i) {
        out.accel[i] = be16(b + 2 + 2 * i);
        out.gyro[i] = be16(b + 8 + 2 * i);
    }
    return ESP_OK;
}
