#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace application
{
    constexpr std::size_t MAC_ADDRESS_SIZE = 6;

    /** Adresa MAC Wi-Fi STA a receptorului RXU01. */
    inline constexpr std::array<uint8_t, MAC_ADDRESS_SIZE>
        RXU01_MAC{0x02, 0x52, 0x58, 0x55, 0x00, 0x01};

    /** Adresa MAC Wi-Fi STA locala a telecomenzii TXU01. */
    inline constexpr std::array<uint8_t, MAC_ADDRESS_SIZE>
        TXU01_MAC{0x02, 0x52, 0x58, 0x55, 0x00, 0x02};

    struct Config
    {
        std::array<uint8_t, MAC_ADDRESS_SIZE> txu01_mac =
            TXU01_MAC;

        std::array<uint8_t, MAC_ADDRESS_SIZE> rxu01_mac =
            RXU01_MAC;

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
