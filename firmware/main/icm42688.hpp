// ICM-42688-P 6-axis IMU over SPI: 200 Hz output data rate, data-ready on INT1.
#pragma once

#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

struct ImuRaw {
    int16_t temp;
    int16_t accel[3];
    int16_t gyro[3];
};

class Icm42688 {
public:
    esp_err_t init(spi_host_device_t host, int sclk, int mosi, int miso, int cs);
    // Burst read of temperature, accel and gyro (14 bytes from TEMP_DATA1).
    esp_err_t read(ImuRaw& out);

    // Full scale: +-8 g and +-500 dps.
    static constexpr float kAccelScale = 9.80665f / 4096.0f;              // m/s^2 per count
    static constexpr float kGyroScale = (500.0f / 32768.0f) * 0.01745329252f;  // rad/s per count
    static constexpr float kTempScale = 1.0f / 132.48f;                   // deg C per count
    static constexpr float kTempOffset = 25.0f;

private:
    esp_err_t write_reg(uint8_t reg, uint8_t val);
    esp_err_t read_regs(uint8_t reg, uint8_t* out, int n);

    spi_device_handle_t dev_ = nullptr;
    alignas(4) uint8_t rx_[16] = {};
};
