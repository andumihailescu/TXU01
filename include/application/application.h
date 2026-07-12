#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace application
{
    constexpr std::size_t MAC_ADDRESS_SIZE = 6;

    /**
     * MAC placeholder local/unicast. Inlocuieste-l cu MAC-ul Wi-Fi STA real
     * al RXU01 inainte de testarea comunicatiei.
     */
    inline constexpr std::array<uint8_t, MAC_ADDRESS_SIZE>
        RXU01_DUMMY_MAC{0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

    struct Config
    {
        std::array<uint8_t, MAC_ADDRESS_SIZE> rxu01_mac =
            RXU01_DUMMY_MAC;

        uint8_t esp_now_channel = 1;
        uint8_t receive_queue_depth = 8;
        uint8_t send_result_queue_depth = 8;
        uint32_t send_interval_ms = 1000;
        uint32_t send_result_timeout_ms = 200;
    };

    /**
     * Initializeaza serviciile aplicatiei si ruleaza bucla principala.
     * Functia nu revine in timpul functionarii normale.
     */
    [[noreturn]] void run(const Config &config = {});
}
