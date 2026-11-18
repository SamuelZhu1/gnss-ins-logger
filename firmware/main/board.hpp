// Pin and option mapping from menuconfig (Kconfig.projbuild).
#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "sdkconfig.h"

namespace board {

constexpr spi_host_device_t kImuSpi = SPI2_HOST;
constexpr spi_host_device_t kSdSpi = SPI3_HOST;
constexpr uart_port_t kGnssUart = UART_NUM_1;

constexpr gpio_num_t kImuSclk = static_cast<gpio_num_t>(CONFIG_GINS_IMU_SCLK);
constexpr gpio_num_t kImuMosi = static_cast<gpio_num_t>(CONFIG_GINS_IMU_MOSI);
constexpr gpio_num_t kImuMiso = static_cast<gpio_num_t>(CONFIG_GINS_IMU_MISO);
constexpr gpio_num_t kImuCs = static_cast<gpio_num_t>(CONFIG_GINS_IMU_CS);
constexpr gpio_num_t kImuInt = static_cast<gpio_num_t>(CONFIG_GINS_IMU_INT);
constexpr gpio_num_t kSdSclk = static_cast<gpio_num_t>(CONFIG_GINS_SD_SCLK);
constexpr gpio_num_t kSdMosi = static_cast<gpio_num_t>(CONFIG_GINS_SD_MOSI);
constexpr gpio_num_t kSdMiso = static_cast<gpio_num_t>(CONFIG_GINS_SD_MISO);
constexpr gpio_num_t kSdCs = static_cast<gpio_num_t>(CONFIG_GINS_SD_CS);
constexpr gpio_num_t kGnssTx = static_cast<gpio_num_t>(CONFIG_GINS_GNSS_TX);
constexpr gpio_num_t kGnssRx = static_cast<gpio_num_t>(CONFIG_GINS_GNSS_RX);
constexpr gpio_num_t kPps = static_cast<gpio_num_t>(CONFIG_GINS_PPS);

constexpr int kImuRateHz = 200;

}  // namespace board
