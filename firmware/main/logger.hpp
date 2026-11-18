// SD card logger. Producers (navigation and GNSS tasks) encode records into a
// ring buffer and never touch the card; a low-priority task drains the buffer
// to the card in 4 KB blocks, so a 100-250 ms SD write stall never blocks
// sampling. When the buffer is nearly full, RAWX records go first.
#pragma once

#include <cstdint>

#include "gins/log_format.hpp"

namespace logger {

bool init();     // mount the card and create the ring buffer
void task(void*);
bool start();    // open the next R####.BIN and write the header record
void stop();
bool active();

// Encode and queue one record. Returns false if it was dropped.
bool write(gins::logfmt::Type type, const uint8_t* payload, uint16_t len);
void event(gins::logfmt::Event code, uint32_t value);

uint32_t dropped(gins::logfmt::Type type);
uint32_t high_water();

}  // namespace logger
