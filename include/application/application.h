#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

#include "reliable_command_sender/reliable_command_sender.h"
#include "turn_signal/turn_signal.h"
#include "turn_signal_switch/turn_signal_switch.h"
#include "warning_switch/warning_switch.h"

namespace application
{
    constexpr std::size_t MAC_ADDRESS_SIZE = 6;

    inline constexpr std::array<uint8_t, MAC_ADDRESS_SIZE>
        RXU01_MAC{0x02, 0x52, 0x58, 0x55, 0x00, 0x01};

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

        gpio_num_t left_turn_signal_gpio = GPIO_NUM_4;
        gpio_num_t right_turn_signal_gpio = GPIO_NUM_5;
        gpio_num_t warning_gpio = GPIO_NUM_6;
        uint32_t debounce_ms = 40;
        warning_switch::InputMode warning_input_mode =
            warning_switch::InputMode::MomentaryToggle;

        uint32_t loop_interval_ms = 10;
        uint32_t state_refresh_interval_ms = 100;
        uint32_t failed_sync_retry_interval_ms = 100;
        uint32_t send_result_timeout_ms = 200;
        uint32_t acknowledgement_timeout_ms = 200;
        uint8_t maximum_retries = 2;
    };

    class Application
    {
    public:
        explicit Application(Config config = {});

        esp_err_t init();
        void process();

    private:
        esp_err_t validate_config() const;
        esp_err_t initialize_wifi();
        esp_err_t initialize_esp_now();
        esp_err_t add_rxu01_peer();
        void log_startup_info();

        void start_state_command(
            const turn_signal::RequestedState &state,
            int64_t now_us);
        void handle_command_result(
            const reliable_command_sender::Result &result,
            int64_t now_us);

        Config config_;
        turn_signal_switch::TurnSignalSwitch turn_signal_switch_;
        warning_switch::WarningSwitch warning_switch_;
        reliable_command_sender::ReliableCommandSender sender_;

        bool initialized_ = false;
        bool sync_requested_ = true;
        bool command_active_ = false;
        int64_t next_sync_us_ = 0;

        turn_signal::RequestedState desired_state_{};
        turn_signal::RequestedState command_state_{};
    };
}
