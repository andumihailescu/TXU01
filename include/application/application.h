#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

#include "light_mode_selector/light_mode_selector.h"
#include "lighting/lighting_state.h"
#include "momentary_toggle/momentary_toggle.h"
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
        gpio_num_t light_mode_gpio = GPIO_NUM_7;
        gpio_num_t reverse_lights_gpio = GPIO_NUM_8;
        gpio_num_t high_beam_gpio = GPIO_NUM_9;
        gpio_num_t projectors_gpio = GPIO_NUM_10;
        gpio_num_t fog_lights_gpio = GPIO_NUM_11;
        uint32_t debounce_ms = 40;
        warning_switch::InputMode warning_input_mode =
            warning_switch::InputMode::MomentaryToggle;

        uint16_t light_mode_drl_upper_raw = 1365;
        uint16_t light_mode_positions_upper_raw = 2730;
        uint16_t light_mode_hysteresis_raw = 80;
        uint8_t light_mode_samples_per_read = 8;
        uint32_t light_mode_debounce_ms = 60;

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
        enum class CommandKind : uint8_t
        {
            None,
            Indicator,
            ExteriorLights,
            ReverseLights
        };

        struct SyncStatus
        {
            int64_t next_attempt_us = 0;
        };

        esp_err_t validate_config() const;
        esp_err_t initialize_wifi();
        esp_err_t initialize_esp_now();
        esp_err_t add_rxu01_peer();
        void log_startup_info();

        void update_inputs();
        void request_sync(SyncStatus &status);
        CommandKind select_next_command(int64_t now_us);
        bool start_command(CommandKind kind, int64_t now_us);
        bool start_indicator_command(int64_t now_us);
        bool start_exterior_lights_command(int64_t now_us);
        bool start_reverse_lights_command(int64_t now_us);
        void handle_command_result(
            const reliable_command_sender::Result &result,
            int64_t now_us);
        SyncStatus &sync_status(CommandKind kind);
        bool active_snapshot_matches_desired() const;
        void defer_failed_start(SyncStatus &status, int64_t now_us);

        Config config_;
        turn_signal_switch::TurnSignalSwitch turn_signal_switch_;
        warning_switch::WarningSwitch warning_switch_;
        light_mode_selector::LightModeSelector light_mode_selector_;
        momentary_toggle::MomentaryToggle high_beam_button_;
        momentary_toggle::MomentaryToggle projectors_button_;
        momentary_toggle::MomentaryToggle fog_lights_button_;
        momentary_toggle::MomentaryToggle reverse_lights_button_;
        reliable_command_sender::ReliableCommandSender sender_;

        bool initialized_ = false;
        esp_err_t last_light_mode_read_error_ = ESP_OK;
        CommandKind active_command_ = CommandKind::None;
        uint8_t scheduler_cursor_ = 0;

        SyncStatus indicator_sync_{};
        SyncStatus exterior_lights_sync_{};
        SyncStatus reverse_lights_sync_{};

        turn_signal::RequestedState desired_indicator_state_{};
        turn_signal::RequestedState active_indicator_state_{};
        lighting::ExteriorState desired_exterior_lights_state_{};
        lighting::ExteriorState active_exterior_lights_state_{};
        bool desired_reverse_lights_state_ = false;
        bool active_reverse_lights_state_ = false;
    };
}
