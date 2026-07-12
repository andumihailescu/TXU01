#include "application/application.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_now_driver/esp_now_driver.h"
#include "remote_protocol/remote_protocol.h"
#include "wifi_manager/wifi_manager.h"

namespace
{
    constexpr char TAG[] = "REMOTE";
    constexpr uint16_t TEST_MESSAGE_ID = 0x0100;

    esp_err_t validate_config(const application::Config &config)
    {
        if (config.esp_now_channel < 1 ||
            config.esp_now_channel > 13 ||
            (config.rxu01_mac[0] & 0x01U) != 0 ||
            config.receive_queue_depth == 0 ||
            config.send_result_queue_depth == 0 ||
            config.send_interval_ms == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        return ESP_OK;
    }

    bool uses_dummy_rxu01_mac(const application::Config &config)
    {
        return config.rxu01_mac == application::RXU01_DUMMY_MAC;
    }

    void write_counter_to_payload(
        uint8_t *payload,
        uint32_t counter)
    {
        payload[0] = static_cast<uint8_t>(counter);
        payload[1] = static_cast<uint8_t>(counter >> 8U);
        payload[2] = static_cast<uint8_t>(counter >> 16U);
        payload[3] = static_cast<uint8_t>(counter >> 24U);
    }

    esp_err_t initialize_wifi(const application::Config &app_config)
    {
        wifi_manager::Config config{};
        config.channel = app_config.esp_now_channel;
        config.storage = WIFI_STORAGE_RAM;
        config.power_save = WIFI_PS_NONE;

        return wifi_manager::init(config);
    }

    esp_err_t initialize_esp_now(const application::Config &app_config)
    {
        esp_now_driver::Config config{};
        config.interface = WIFI_IF_STA;
        config.default_peer_channel = 0;
        config.receive_queue_depth = app_config.receive_queue_depth;
        config.send_result_queue_depth = app_config.send_result_queue_depth;

        return esp_now_driver::init(config);
    }

    esp_err_t add_rxu01_peer(const application::Config &config)
    {
        esp_now_driver::PeerConfig peer{};

        std::memcpy(
            peer.mac,
            config.rxu01_mac.data(),
            esp_now_driver::MAC_ADDRESS_SIZE);

        // Canalul 0 urmeaza canalul curent al interfetei Wi-Fi.
        peer.channel = 0;
        peer.interface = WIFI_IF_STA;
        peer.encrypt = false;

        return esp_now_driver::add_peer(peer);
    }

    void log_local_mac()
    {
        uint8_t local_mac[esp_now_driver::MAC_ADDRESS_SIZE]{};

        ESP_ERROR_CHECK(
            esp_now_driver::get_local_mac(local_mac));

        ESP_LOGI(TAG, "Telecomanda ESP32-S3 pornita");
        ESP_LOGI(TAG, "MAC local STA: " MACSTR, MAC2STR(local_mac));
    }

    esp_err_t send_test_message(
        const application::Config &config,
        uint32_t counter)
    {
        remote_protocol::Message message{};
        message.type = remote_protocol::MessageType::Command;
        message.flags = remote_protocol::FlagNone;
        message.sequence_number = static_cast<uint16_t>(counter);
        message.message_id = TEST_MESSAGE_ID;
        message.payload_length = 8;

        write_counter_to_payload(message.payload, counter);
        message.payload[4] = 0x11;
        message.payload[5] = 0x22;
        message.payload[6] = 0x33;
        message.payload[7] = 0x44;

        uint8_t encoded_packet[remote_protocol::MAX_PACKET_SIZE]{};
        std::size_t encoded_length = 0;

        const remote_protocol::EncodeResult encode_result =
            remote_protocol::encode(
                message,
                encoded_packet,
                sizeof(encoded_packet),
                encoded_length);

        if (encode_result != remote_protocol::EncodeResult::Ok)
        {
            ESP_LOGE(
                TAG,
                "Eroare encode: %s",
                remote_protocol::to_string(encode_result));

            return ESP_ERR_INVALID_STATE;
        }

        const esp_err_t send_result =
            esp_now_driver::send(
                config.rxu01_mac.data(),
                encoded_packet,
                encoded_length);

        if (send_result != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "Eroare esp_now_send: %s",
                esp_err_to_name(send_result));

            return send_result;
        }

        ESP_LOGI(
            TAG,
            "Cerere trimitere counter=%lu: "
            "%02X %02X %02X %02X "
            "%02X %02X %02X %02X "
            "(pachet total=%u bytes)",
            static_cast<unsigned long>(counter),
            message.payload[0],
            message.payload[1],
            message.payload[2],
            message.payload[3],
            message.payload[4],
            message.payload[5],
            message.payload[6],
            message.payload[7],
            static_cast<unsigned>(encoded_length));

        return ESP_OK;
    }

    void log_send_result(uint32_t timeout_ms)
    {
        esp_now_driver::SendResult result{};

        if (!esp_now_driver::receive_send_result(
                result,
                timeout_ms))
        {
            ESP_LOGW(TAG, "Nu a venit rezultatul transmisiei");
            return;
        }

        ESP_LOGI(
            TAG,
            "Transmitere catre " MACSTR ": %s",
            MAC2STR(result.destination_mac),
            result.status == ESP_NOW_SEND_SUCCESS
                ? "OK"
                : "FAIL");
    }

    [[noreturn]] void run_main_loop(const application::Config &config)
    {
        uint32_t counter = 0;

        while (true)
        {
            if (send_test_message(config, counter) == ESP_OK)
            {
                log_send_result(config.send_result_timeout_ms);
            }

            ++counter;
            vTaskDelay(pdMS_TO_TICKS(config.send_interval_ms));
        }
    }
}

namespace application
{
    [[noreturn]] void run(const Config &config)
    {
        ESP_ERROR_CHECK(validate_config(config));

        if (uses_dummy_rxu01_mac(config))
        {
            ESP_LOGW(
                TAG,
                "RXU01 foloseste MAC-ul dummy " MACSTR
                "; inlocuieste-l cu MAC-ul STA real",
                MAC2STR(config.rxu01_mac.data()));
        }

        ESP_ERROR_CHECK(initialize_wifi(config));
        ESP_ERROR_CHECK(initialize_esp_now(config));
        ESP_ERROR_CHECK(add_rxu01_peer(config));

        log_local_mac();
        run_main_loop(config);
    }
}
