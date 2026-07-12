#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_manager/wifi_manager.h"
#include "esp_now_driver/esp_now_driver.h"
#include "remote_protocol/remote_protocol.h"

namespace
{
    constexpr char TAG[] = "REMOTE";

    constexpr uint8_t ESPNOW_CHANNEL = 1;

    constexpr uint16_t TEST_MESSAGE_ID = 0x0100;

    void writeCounterToPayload(
        uint8_t *payload,
        uint32_t counter)
    {
        payload[0] =
            static_cast<uint8_t>(counter);

        payload[1] =
            static_cast<uint8_t>(counter >> 8U);

        payload[2] =
            static_cast<uint8_t>(counter >> 16U);

        payload[3] =
            static_cast<uint8_t>(counter >> 24U);
    }

    esp_err_t sendTestMessage(uint32_t counter)
    {
        remote_protocol::Message message{};

        message.type =
            remote_protocol::MessageType::Command;

        message.destination =
            remote_protocol::Destination::Rxu01;

        message.flags =
            remote_protocol::FlagNone;

        message.sequence_number =
            static_cast<uint16_t>(counter);

        message.message_id =
            TEST_MESSAGE_ID;

        message.payload_length = 8;

        writeCounterToPayload(
            message.payload,
            counter);

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

        if (encode_result !=
            remote_protocol::EncodeResult::Ok)
        {
            ESP_LOGE(
                TAG,
                "Eroare encode: %s",
                remote_protocol::to_string(
                    encode_result));

            return ESP_ERR_INVALID_STATE;
        }

        const esp_err_t send_result =
            esp_now_driver::send(
                esp_now_driver::BROADCAST_MAC,
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

    void logSendResult()
    {
        esp_now_driver::SendResult result{};

        /*
         * Asteptam maximum 200 ms rezultatul callbackului.
         * In acelasi timp golim coada rezultatelor TX.
         */
        if (!esp_now_driver::receive_send_result(
                result,
                200))
        {
            ESP_LOGW(
                TAG,
                "Nu a venit rezultatul transmisiei");

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
}

extern "C" void app_main(void)
{
    /*
     * 1. Pornim Wi-Fi pe canalul folosit de ESP-NOW.
     */
    wifi_manager::Config wifi_config{};

    wifi_config.channel = ESPNOW_CHANNEL;
    wifi_config.storage = WIFI_STORAGE_RAM;
    wifi_config.power_save = WIFI_PS_NONE;

    ESP_ERROR_CHECK(
        wifi_manager::init(
            wifi_config));

    /*
     * 2. Initializam driverul ESP-NOW.
     */
    esp_now_driver::Config espnow_config{};

    espnow_config.interface = WIFI_IF_STA;

    espnow_config.default_peer_channel =
        ESPNOW_CHANNEL;

    espnow_config.receive_queue_depth = 8;
    espnow_config.send_result_queue_depth = 8;

    ESP_ERROR_CHECK(
        esp_now_driver::init(
            espnow_config));

    /*
     * 3. Adaugam adresa broadcast ca peer.
     */
    esp_now_driver::PeerConfig broadcast_peer{};

    std::memcpy(
        broadcast_peer.mac,
        esp_now_driver::BROADCAST_MAC,
        esp_now_driver::MAC_ADDRESS_SIZE);

    broadcast_peer.channel =
        ESPNOW_CHANNEL;

    broadcast_peer.interface =
        WIFI_IF_STA;

    broadcast_peer.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_driver::add_peer(
            broadcast_peer));

    /*
     * 4. Afisam adresa MAC a telecomenzii.
     */
    uint8_t local_mac[esp_now_driver::MAC_ADDRESS_SIZE]{};

    ESP_ERROR_CHECK(
        esp_now_driver::get_local_mac(
            local_mac));

    ESP_LOGI(
        TAG,
        "Telecomanda ESP32-S3 pornita");

    ESP_LOGI(
        TAG,
        "MAC local STA: " MACSTR,
        MAC2STR(local_mac));

    /*
     * 5. Trimitem mesajul o data pe secunda.
     */
    uint32_t counter = 0;

    while (true)
    {
        const esp_err_t result =
            sendTestMessage(counter);

        if (result == ESP_OK)
        {
            logSendResult();
        }

        ++counter;

        vTaskDelay(
            pdMS_TO_TICKS(1000));
    }
}