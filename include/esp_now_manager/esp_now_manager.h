#pragma once

#include <cstdint>

namespace esp_now_manager
{
    void init();
    void send_test_message(uint32_t counter);
} // namespace esp_now_manager
