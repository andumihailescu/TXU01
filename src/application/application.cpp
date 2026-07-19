#include "application/application.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "config/software_version.h"
#include "esp_now_driver/esp_now_driver.h"
#include "vehicle_can_protocol/vehicle_can_protocol.h"
#include "wifi_manager/wifi_manager.h"

namespace application
{
    static constexpr char TAG[] = "TXU01";
    static constexpr std::size_t COMMAND_KIND_COUNT = 3;

    static bool is_valid_unicast_mac(
        const std::array<uint8_t, MAC_ADDRESS_SIZE> &mac)
    {
        bool is_zero = true;

        for (uint8_t byte : mac)
        {
            is_zero = is_zero && byte == 0;
        }

        return !is_zero && (mac[0] & 0x01U) == 0;
    }

    static turn_signal_switch::Config make_switch_config(
        const Config &config)
    {
        turn_signal_switch::Config switch_config{};
        switch_config.left_gpio = config.left_turn_signal_gpio;
        switch_config.right_gpio = config.right_turn_signal_gpio;
        switch_config.debounce_ms = config.debounce_ms;
        return switch_config;
    }

    static warning_switch::Config make_warning_switch_config(
        const Config &config)
    {
        warning_switch::Config switch_config{};
        switch_config.gpio = config.warning_gpio;
        switch_config.debounce_ms = config.debounce_ms;
        switch_config.input_mode = config.warning_input_mode;
        return switch_config;
    }

    static light_mode_selector::Config
    make_light_mode_selector_config(const Config &config)
    {
        light_mode_selector::Config selector_config{};
        selector_config.gpio = config.light_mode_gpio;
        selector_config.drl_upper_raw =
            config.light_mode_drl_upper_raw;
        selector_config.positions_upper_raw =
            config.light_mode_positions_upper_raw;
        selector_config.hysteresis_raw =
            config.light_mode_hysteresis_raw;
        selector_config.samples_per_read =
            config.light_mode_samples_per_read;
        selector_config.debounce_ms =
            config.light_mode_debounce_ms;
        return selector_config;
    }

    static momentary_toggle::Config make_toggle_config(
        gpio_num_t gpio,
        uint32_t debounce_ms)
    {
        momentary_toggle::Config toggle_config{};
        toggle_config.gpio = gpio;
        toggle_config.debounce_ms = debounce_ms;
        return toggle_config;
    }

    static reliable_command_sender::Config make_sender_config(
        const Config &config)
    {
        reliable_command_sender::Config sender_config{};
        sender_config.destination_mac = config.rxu01_mac;
        sender_config.local_mac = config.txu01_mac;
        sender_config.send_result_timeout_ms =
            config.send_result_timeout_ms;
        sender_config.acknowledgement_timeout_ms =
            config.acknowledgement_timeout_ms;
        sender_config.maximum_retries = config.maximum_retries;
        return sender_config;
    }

    static bool input_gpios_are_unique(const Config &config)
    {
        const std::array<gpio_num_t, 8> gpios = {
            config.left_turn_signal_gpio,
            config.right_turn_signal_gpio,
            config.warning_gpio,
            config.light_mode_gpio,
            config.high_beam_gpio,
            config.projectors_gpio,
            config.fog_lights_gpio,
            config.reverse_lights_gpio,
        };

        for (std::size_t index = 0; index < gpios.size(); ++index)
        {
            if (!GPIO_IS_VALID_GPIO(gpios[index]))
            {
                return false;
            }

            for (std::size_t other = index + 1;
                 other < gpios.size();
                 ++other)
            {
                if (gpios[index] == gpios[other])
                {
                    return false;
                }
            }
        }

        return true;
    }

    Application::Application(Config config)
        : config_(config),
          turn_signal_switch_(make_switch_config(config)),
          warning_switch_(make_warning_switch_config(config)),
          light_mode_selector_(make_light_mode_selector_config(config)),
          high_beam_button_(make_toggle_config(
              config.high_beam_gpio,
              config.debounce_ms)),
          projectors_button_(make_toggle_config(
              config.projectors_gpio,
              config.debounce_ms)),
          fog_lights_button_(make_toggle_config(
              config.fog_lights_gpio,
              config.debounce_ms)),
          reverse_lights_button_(make_toggle_config(
              config.reverse_lights_gpio,
              config.debounce_ms)),
          sender_(make_sender_config(config))
    {
    }

    esp_err_t Application::init()
    {
        if (initialized_)
        {
            return ESP_OK;
        }

        esp_err_t result = validate_config();

        if (result != ESP_OK)
        {
            return result;
        }

        result = initialize_wifi();

        if (result != ESP_OK)
        {
            return result;
        }

        result = initialize_esp_now();

        if (result != ESP_OK)
        {
            return result;
        }

        result = add_rxu01_peer();

        if (result != ESP_OK)
        {
            return result;
        }

        result = turn_signal_switch_.init();

        if (result != ESP_OK)
        {
            return result;
        }

        result = warning_switch_.init();

        if (result != ESP_OK)
        {
            return result;
        }

        result = light_mode_selector_.init();

        if (result != ESP_OK)
        {
            return result;
        }

        for (momentary_toggle::MomentaryToggle *button : {
                 &high_beam_button_,
                 &projectors_button_,
                 &fog_lights_button_,
                 &reverse_lights_button_})
        {
            result = button->init();

            if (result != ESP_OK)
            {
                return result;
            }
        }

        result = sender_.init();

        if (result != ESP_OK)
        {
            return result;
        }

        desired_indicator_state_ = turn_signal::make_requested_state(
            turn_signal_switch_.input(),
            warning_switch_.request());
        desired_exterior_lights_state_.mode =
            light_mode_selector_.mode();
        desired_reverse_lights_state_ =
            reverse_lights_button_.active();

        indicator_sync_ = {};
        exterior_lights_sync_ = {};
        reverse_lights_sync_ = {};
        initialized_ = true;

        log_startup_info();
        return ESP_OK;
    }

    void Application::process()
    {
        if (!initialized_)
        {
            return;
        }

        const int64_t now_us = esp_timer_get_time();
        update_inputs();
        sender_.process();

        reliable_command_sender::Result command_result{};

        if (sender_.take_result(command_result))
        {
            handle_command_result(command_result, now_us);
        }

        if (sender_.busy() || active_command_ != CommandKind::None)
        {
            return;
        }

        const CommandKind next_command =
            select_next_command(now_us);

        if (next_command != CommandKind::None)
        {
            start_command(next_command, now_us);
        }
    }

    esp_err_t Application::validate_config() const
    {
        if (config_.esp_now_channel < 1 ||
            config_.esp_now_channel > 13 ||
            !is_valid_unicast_mac(config_.txu01_mac) ||
            !is_valid_unicast_mac(config_.rxu01_mac) ||
            config_.txu01_mac == config_.rxu01_mac ||
            config_.receive_queue_depth == 0 ||
            config_.send_result_queue_depth == 0 ||
            !input_gpios_are_unique(config_) ||
            config_.debounce_ms == 0 ||
            !lighting::are_mode_thresholds_valid(
                config_.light_mode_drl_upper_raw,
                config_.light_mode_positions_upper_raw,
                config_.light_mode_hysteresis_raw) ||
            config_.light_mode_samples_per_read == 0 ||
            config_.light_mode_debounce_ms == 0 ||
            (config_.warning_input_mode !=
                 warning_switch::InputMode::MaintainedLevel &&
             config_.warning_input_mode !=
                 warning_switch::InputMode::MomentaryToggle) ||
            config_.loop_interval_ms == 0 ||
            config_.state_refresh_interval_ms == 0 ||
            config_.failed_sync_retry_interval_ms == 0 ||
            config_.send_result_timeout_ms == 0 ||
            config_.acknowledgement_timeout_ms == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        return ESP_OK;
    }

    esp_err_t Application::initialize_wifi()
    {
        wifi_manager::Config config{};
        config.channel = config_.esp_now_channel;
        config.storage = WIFI_STORAGE_RAM;
        config.power_save = WIFI_PS_NONE;
        config.use_custom_station_mac = true;

        std::memcpy(
            config.station_mac,
            config_.txu01_mac.data(),
            wifi_manager::MAC_ADDRESS_SIZE);

        return wifi_manager::init(config);
    }

    esp_err_t Application::initialize_esp_now()
    {
        esp_now_driver::Config config{};
        config.interface = WIFI_IF_STA;
        config.default_peer_channel = 0;
        config.receive_queue_depth = config_.receive_queue_depth;
        config.send_result_queue_depth =
            config_.send_result_queue_depth;

        return esp_now_driver::init(config);
    }

    esp_err_t Application::add_rxu01_peer()
    {
        esp_now_driver::PeerConfig peer{};

        std::memcpy(
            peer.mac,
            config_.rxu01_mac.data(),
            esp_now_driver::MAC_ADDRESS_SIZE);

        peer.channel = 0;
        peer.interface = WIFI_IF_STA;
        peer.encrypt = false;

        return esp_now_driver::add_peer(peer);
    }

    void Application::log_startup_info()
    {
        uint8_t local_mac[esp_now_driver::MAC_ADDRESS_SIZE]{};
        ESP_ERROR_CHECK(esp_now_driver::get_local_mac(local_mac));

        ESP_LOGI(TAG, "Telecomanda pornita");
        ESP_LOGI(
            TAG,
            "Versiune software: %u.%u.%u + CW%02u + CY%02u",
            static_cast<unsigned>(SoftwareVersionConfig::Major),
            static_cast<unsigned>(SoftwareVersionConfig::Minor),
            static_cast<unsigned>(SoftwareVersionConfig::Patch),
            static_cast<unsigned>(SoftwareVersionConfig::IsoWeek),
            static_cast<unsigned>(SoftwareVersionConfig::IsoYearShort));
        ESP_LOGI(TAG, "MAC local STA: " MACSTR, MAC2STR(local_mac));
        ESP_LOGI(
            TAG,
            "Peer RXU01: " MACSTR,
            MAC2STR(config_.rxu01_mac.data()));
        ESP_LOGI(
            TAG,
            "GPIO: stanga=%d, dreapta=%d, warning=%d, mod=%d, "
            "high=%d, proiectoare=%d, ceata=%d, marsarier=%d",
            static_cast<int>(config_.left_turn_signal_gpio),
            static_cast<int>(config_.right_turn_signal_gpio),
            static_cast<int>(config_.warning_gpio),
            static_cast<int>(config_.light_mode_gpio),
            static_cast<int>(config_.high_beam_gpio),
            static_cast<int>(config_.projectors_gpio),
            static_cast<int>(config_.fog_lights_gpio),
            static_cast<int>(config_.reverse_lights_gpio));
        ESP_LOGI(
            TAG,
            "Stare initiala: indicatoare=%s, mod=%s (ADC=%u), "
            "high=0, proiectoare=0, ceata=0, "
            "marsarier=0",
            turn_signal::to_string(
                turn_signal::resolve_effective_state(
                    desired_indicator_state_)),
            lighting::to_string(
                desired_exterior_lights_state_.mode),
            static_cast<unsigned>(light_mode_selector_.raw()));
    }

    void Application::update_inputs()
    {
        const turn_signal_switch::Update turn_update =
            turn_signal_switch_.update();
        const warning_switch::Update warning_update =
            warning_switch_.update();
        const auto next_indicator_state =
            turn_signal::make_requested_state(
                turn_update.input,
                warning_update.request);

        if (next_indicator_state != desired_indicator_state_)
        {
            desired_indicator_state_ = next_indicator_state;
            request_sync(indicator_sync_);

            ESP_LOGI(
                TAG,
                "Cereri indicatoare: maneta=%s, warning=%s, iesire=%s",
                turn_signal::to_string(desired_indicator_state_.turn),
                turn_signal::to_string(desired_indicator_state_.warning),
                turn_signal::to_string(
                    turn_signal::resolve_effective_state(
                        desired_indicator_state_)));

            if (!desired_indicator_state_.turn_input_valid)
            {
                ESP_LOGW(
                    TAG,
                    "Ambele intrari de semnalizare sunt active; "
                    "se comanda oprirea ambelor directii");
            }
        }

        const light_mode_selector::Update mode_update =
            light_mode_selector_.update();

        if (mode_update.read_result != ESP_OK)
        {
            if (mode_update.read_result != last_light_mode_read_error_)
            {
                ESP_LOGE(
                    TAG,
                    "Citirea potentiometrului a esuat: %s; "
                    "se pastreaza ultimul mod stabil",
                    esp_err_to_name(mode_update.read_result));
            }
        }
        else if (last_light_mode_read_error_ != ESP_OK)
        {
            ESP_LOGI(TAG, "Citirea potentiometrului s-a restabilit");
        }

        last_light_mode_read_error_ = mode_update.read_result;

        const momentary_toggle::Update high_beam_update =
            high_beam_button_.update();
        const momentary_toggle::Update projectors_update =
            projectors_button_.update();
        const momentary_toggle::Update fog_update =
            fog_lights_button_.update();
        const momentary_toggle::Update reverse_update =
            reverse_lights_button_.update();

        const lighting::ExteriorState next_exterior_state =
            lighting::make_panel_exterior_state(
            mode_update.mode,
            projectors_update.active,
            fog_update.active,
            high_beam_update.active);

        if (next_exterior_state != desired_exterior_lights_state_)
        {
            desired_exterior_lights_state_ = next_exterior_state;
            request_sync(exterior_lights_sync_);

            ESP_LOGI(
                TAG,
                "Lumini: mod=%s, high=%u, proiectoare=%u, "
                "ceata=%u",
                lighting::to_string(
                    desired_exterior_lights_state_.mode),
                static_cast<unsigned>(
                    desired_exterior_lights_state_.high_beam),
                static_cast<unsigned>(
                    desired_exterior_lights_state_.front_projectors),
                static_cast<unsigned>(
                    desired_exterior_lights_state_.fog_lights));
        }

        if (reverse_update.active != desired_reverse_lights_state_)
        {
            desired_reverse_lights_state_ = reverse_update.active;
            request_sync(reverse_lights_sync_);

            ESP_LOGI(
                TAG,
                "Lumini marsarier: %s",
                desired_reverse_lights_state_ ? "On" : "Off");
        }
    }

    void Application::request_sync(SyncStatus &status)
    {
        status.next_attempt_us = 0;
    }

    Application::CommandKind
    Application::select_next_command(int64_t now_us)
    {
        const std::array<CommandKind, COMMAND_KIND_COUNT> commands = {
            CommandKind::Indicator,
            CommandKind::ExteriorLights,
            CommandKind::ReverseLights,
        };

        // Every due stream gets one turn. A changed snapshot normally has
        // next_attempt_us == 0, but it cannot starve periodic recovery of
        // the other retained LMCU100 states.
        for (std::size_t offset = 0;
             offset < commands.size();
             ++offset)
        {
            const std::size_t index =
                (scheduler_cursor_ + offset) % commands.size();
            SyncStatus &status = sync_status(commands[index]);

            if (now_us >= status.next_attempt_us)
            {
                scheduler_cursor_ = static_cast<uint8_t>(
                    (index + 1U) % commands.size());
                return commands[index];
            }
        }

        return CommandKind::None;
    }

    bool Application::start_command(
        CommandKind kind,
        int64_t now_us)
    {
        switch (kind)
        {
        case CommandKind::Indicator:
            return start_indicator_command(now_us);
        case CommandKind::ExteriorLights:
            return start_exterior_lights_command(now_us);
        case CommandKind::ReverseLights:
            return start_reverse_lights_command(now_us);
        case CommandKind::None:
        default:
            return false;
        }
    }

    bool Application::start_indicator_command(int64_t now_us)
    {
        active_indicator_state_ = desired_indicator_state_;
        const auto payload = turn_signal::make_indicator_state_payload(
            active_indicator_state_);
        const uint16_t message_id = static_cast<uint16_t>(
            vehicle_can_protocol::RemoteCommandId::
                LightingSetIndicatorState);

        if (!sender_.start(
                message_id,
                payload.data(),
                static_cast<uint8_t>(payload.size())))
        {
            defer_failed_start(indicator_sync_, now_us);
            ESP_LOGE(TAG, "Senderul nu a acceptat starea indicatoarelor");
            return false;
        }

        active_command_ = CommandKind::Indicator;
        ESP_LOGI(
            TAG,
            "TX indicatoare: warning=%u, stanga=%u, dreapta=%u",
            static_cast<unsigned>(payload[0]),
            static_cast<unsigned>(payload[1]),
            static_cast<unsigned>(payload[2]));
        return true;
    }

    bool Application::start_exterior_lights_command(int64_t now_us)
    {
        active_exterior_lights_state_ =
            desired_exterior_lights_state_;
        const auto payload = lighting::make_exterior_state_payload(
            active_exterior_lights_state_);
        const uint16_t message_id = static_cast<uint16_t>(
            vehicle_can_protocol::RemoteCommandId::
                LightingSetExteriorLightsState);

        if (!sender_.start(
                message_id,
                payload.data(),
                static_cast<uint8_t>(payload.size())))
        {
            defer_failed_start(exterior_lights_sync_, now_us);
            ESP_LOGE(TAG, "Senderul nu a acceptat starea luminilor");
            return false;
        }

        active_command_ = CommandKind::ExteriorLights;
        ESP_LOGI(
            TAG,
            "TX lumini: mod=%u, proiectoare=%u, ceata=%u, high=%u",
            static_cast<unsigned>(payload[0]),
            static_cast<unsigned>(payload[1]),
            static_cast<unsigned>(payload[2]),
            static_cast<unsigned>(payload[3]));
        return true;
    }

    bool Application::start_reverse_lights_command(int64_t now_us)
    {
        active_reverse_lights_state_ =
            desired_reverse_lights_state_;
        const auto payload = lighting::make_binary_state_payload(
            active_reverse_lights_state_);
        const uint16_t message_id = static_cast<uint16_t>(
            vehicle_can_protocol::RemoteCommandId::
                LightingSetReverseLightState);

        if (!sender_.start(
                message_id,
                payload.data(),
                static_cast<uint8_t>(payload.size())))
        {
            defer_failed_start(reverse_lights_sync_, now_us);
            ESP_LOGE(
                TAG,
                "Senderul nu a acceptat starea luminilor de marsarier");
            return false;
        }

        active_command_ = CommandKind::ReverseLights;
        ESP_LOGI(
            TAG,
            "TX marsarier: activ=%u",
            static_cast<unsigned>(payload[0]));
        return true;
    }

    void Application::handle_command_result(
        const reliable_command_sender::Result &result,
        int64_t now_us)
    {
        const bool success =
            result.code ==
            reliable_command_sender::ResultCode::Success;

        if (success)
        {
            ESP_LOGI(
                TAG,
                "ACK RXU01: mesaj=0x%04X, seq=%u, incercari=%u, "
                "status=%u",
                static_cast<unsigned>(result.message_id),
                static_cast<unsigned>(result.sequence_number),
                static_cast<unsigned>(result.attempts),
                static_cast<unsigned>(result.acknowledgement_status));
        }
        else
        {
            ESP_LOGE(
                TAG,
                "Comanda 0x%04X esuata: seq=%u, rezultat=%s, "
                "ACK=%u, incercari=%u, eroare=%s",
                static_cast<unsigned>(result.message_id),
                static_cast<unsigned>(result.sequence_number),
                reliable_command_sender::to_string(result.code),
                static_cast<unsigned>(result.acknowledgement_status),
                static_cast<unsigned>(result.attempts),
                esp_err_to_name(result.send_error));
        }

        if (active_command_ == CommandKind::None)
        {
            ESP_LOGW(TAG, "Rezultat primit fara o comanda activa");
            return;
        }

        SyncStatus &status = sync_status(active_command_);

        if (success && active_snapshot_matches_desired())
        {
            status.next_attempt_us =
                now_us +
                static_cast<int64_t>(
                    config_.state_refresh_interval_ms) *
                    1000;
        }
        else if (success)
        {
            request_sync(status);
        }
        else
        {
            status.next_attempt_us =
                now_us +
                static_cast<int64_t>(
                    config_.failed_sync_retry_interval_ms) *
                    1000;
        }

        active_command_ = CommandKind::None;
    }

    Application::SyncStatus &Application::sync_status(CommandKind kind)
    {
        switch (kind)
        {
        case CommandKind::ExteriorLights:
            return exterior_lights_sync_;
        case CommandKind::ReverseLights:
            return reverse_lights_sync_;
        case CommandKind::Indicator:
        case CommandKind::None:
        default:
            return indicator_sync_;
        }
    }

    bool Application::active_snapshot_matches_desired() const
    {
        switch (active_command_)
        {
        case CommandKind::Indicator:
            return active_indicator_state_ ==
                   desired_indicator_state_;
        case CommandKind::ExteriorLights:
            return active_exterior_lights_state_ ==
                   desired_exterior_lights_state_;
        case CommandKind::ReverseLights:
            return active_reverse_lights_state_ ==
                   desired_reverse_lights_state_;
        case CommandKind::None:
        default:
            return false;
        }
    }

    void Application::defer_failed_start(
        SyncStatus &status,
        int64_t now_us)
    {
        status.next_attempt_us =
            now_us +
            static_cast<int64_t>(
                config_.failed_sync_retry_interval_ms) *
                1000;
    }
}
