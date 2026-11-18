#include "status.hpp"

namespace status {

Status g_status;
SemaphoreHandle_t g_mutex = nullptr;

void init() { g_mutex = xSemaphoreCreateMutex(); }

}  // namespace status
