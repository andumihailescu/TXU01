#include "application/application.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "esp_now_driver/esp_now_driver.h"
#include "vehicle_can_protocol/vehicle_can_protocol.h"
#include "wifi_manager/wifi_manager.h"

namespace application
{
    static constexpr char TAG[] = "TXU01";

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

    Application::Application(Config config)
        : config_(config),
          turn_signal_switch_(make_switch_config(config)),
          warning_switch_(make_warning_switch_config(config)),
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

        result = sender_.init();

        if (result != ESP_OK)
        {
            return result;
        }

        desired_state_ = turn_signal::make_requested_state(
            turn_signal_switch_.input(),
            warning_switch_.request());
        sync_requested_ = true;
        next_sync_us_ = 0;
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
        const turn_signal_switch::Update turn_update =
            turn_signal_switch_.update();
        const warning_switch::Update warning_update =
            warning_switch_.update();

        desired_state_ = turn_signal::make_requested_state(
            turn_update.input,
            warning_update.request);

        if (turn_update.changed || warning_update.changed)
        {
            sync_requested_ = true;

            ESP_LOGI(
                TAG,
                "Cereri: maneta=%s, warning=%s, iesire=%s",
                turn_signal::to_string(desired_state_.turn),
                turn_signal::to_string(desired_state_.warning),
                turn_signal::to_string(
                    turn_signal::resolve_effective_state(
                        desired_state_)));

            if (!desired_state_.turn_input_valid)
            {
                ESP_LOGW(
                    TAG,
                    "Ambele intrari de semnalizare sunt active; "
                    "se comanda oprirea ambelor directii");
            }
        }

        sender_.process();

        reliable_command_sender::Result command_result{};

        if (sender_.take_result(command_result))
        {
            handle_command_result(command_result, now_us);
        }

        if (sender_.busy() || command_active_)
        {
            return;
        }

        if (sync_requested_ || now_us >= next_sync_us_)
        {
            start_state_command(desired_state_, now_us);
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
            !GPIO_IS_VALID_GPIO(config_.left_turn_signal_gpio) ||
            !GPIO_IS_VALID_GPIO(config_.right_turn_signal_gpio) ||
            !GPIO_IS_VALID_GPIO(config_.warning_gpio) ||
            config_.left_turn_signal_gpio ==
                config_.right_turn_signal_gpio ||
            config_.left_turn_signal_gpio == config_.warning_gpio ||
            config_.right_turn_signal_gpio == config_.warning_gpio ||
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
        ESP_LOGI(TAG, "MAC local STA: " MACSTR, MAC2STR(local_mac));
        ESP_LOGI(
            TAG,
            "Peer RXU01: " MACSTR,
            MAC2STR(config_.rxu01_mac.data()));
        ESP_LOGI(
            TAG,
            "GPIO: stanga=%d, dreapta=%d, warning=%d",
            static_cast<int>(config_.left_turn_signal_gpio),
            static_cast<int>(config_.right_turn_signal_gpio),
            static_cast<int>(config_.warning_gpio));
        ESP_LOGI(
            TAG,
            "Cereri initiale: maneta=%s, warning=%s, iesire=%s",
            turn_signal::to_string(desired_state_.turn),
            turn_signal::to_string(desired_state_.warning),
            turn_signal::to_string(
                turn_signal::resolve_effective_state(desired_state_)));
    }

    void Application::start_state_command(
        const turn_signal::RequestedState &state,
        int64_t now_us)
    {
        const vehicle_can_protocol::IndicatorStatePayload payload =
            turn_signal::make_indicator_state_payload(state);

        const uint16_t message_id = static_cast<uint16_t>(
            vehicle_can_protocol::RemoteCommandId::
                LightingSetIndicatorState);

        if (!sender_.start(
                message_id,
                payload.data(),
                static_cast<uint8_t>(payload.size())))
        {
            ESP_LOGE(TAG, "Senderul nu a acceptat starea indicatoarelor");
            sync_requested_ = false;
            next_sync_us_ =
                now_us +
                static_cast<int64_t>(
                    config_.failed_sync_retry_interval_ms) *
                    1000;
            return;
        }

        command_state_ = state;
        command_active_ = true;
        sync_requested_ = false;
        next_sync_us_ =
            now_us +
            static_cast<int64_t>(
                config_.state_refresh_interval_ms) *
                1000;

        ESP_LOGI(
            TAG,
            "Stare indicatoare: warning=%u, stanga=%u, dreapta=%u",
            static_cast<unsigned>(payload[0]),
            static_cast<unsigned>(payload[1]),
            static_cast<unsigned>(payload[2]));
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
                "ACK RXU01: seq=%u, incercari=%u, status=%u",
                static_cast<unsigned>(result.sequence_number),
                static_cast<unsigned>(result.attempts),
                static_cast<unsigned>(result.acknowledgement_status));
        }
        else
        {
            ESP_LOGE(
                TAG,
                "Comanda esuata: seq=%u, rezultat=%s, "
                "ACK=%u, incercari=%u, eroare=%s",
                static_cast<unsigned>(result.sequence_number),
                reliable_command_sender::to_string(result.code),
                static_cast<unsigned>(result.acknowledgement_status),
                static_cast<unsigned>(result.attempts),
                esp_err_to_name(result.send_error));
        }

        if (!command_active_)
        {
            return;
        }

        command_active_ = false;

        if (desired_state_ != command_state_)
        {
            sync_requested_ = true;
            next_sync_us_ = 0;
            return;
        }

        sync_requested_ = false;

        if (!success)
        {
            next_sync_us_ =
                now_us +
                static_cast<int64_t>(
                    config_.failed_sync_retry_interval_ms) *
                    1000;
        }
    }
}
