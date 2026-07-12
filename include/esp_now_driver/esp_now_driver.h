#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi_types.h"

namespace esp_now_driver
{
    constexpr std::size_t MAC_ADDRESS_SIZE = ESP_NOW_ETH_ALEN;

    /**
     * Limita de 250 bytes pastreaza compatibilitatea cu ESP-NOW v1.0.
     * Protocolul aplicatiei foloseste maximum 77 bytes in configuratia curenta.
     */
    constexpr std::size_t MAX_PAYLOAD_SIZE = ESP_NOW_MAX_DATA_LEN;

    constexpr uint32_t WAIT_FOREVER = UINT32_MAX;

    inline constexpr uint8_t BROADCAST_MAC[MAC_ADDRESS_SIZE] = {
        0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF};

    struct Config
    {
        wifi_interface_t interface = WIFI_IF_STA;
        // 0 foloseste canalul curent al interfetei Wi-Fi.
        uint8_t default_peer_channel = 0;

        uint8_t receive_queue_depth = 8;
        uint8_t send_result_queue_depth = 8;
    };

    struct PeerConfig
    {
        uint8_t mac[MAC_ADDRESS_SIZE]{};

        /**
         * 0 inseamna canalul Wi-Fi curent.
         * Altfel trebuie sa fie acelasi canal cu interfata locala.
         */
        uint8_t channel = 0;

        wifi_interface_t interface = WIFI_IF_STA;
        bool encrypt = false;

        uint8_t lmk[ESP_NOW_KEY_LEN]{};
    };

    struct ReceivedPacket
    {
        uint8_t source_mac[MAC_ADDRESS_SIZE]{};
        uint8_t destination_mac[MAC_ADDRESS_SIZE]{};

        int8_t rssi = 0;

        uint16_t data_length = 0;
        uint8_t data[MAX_PAYLOAD_SIZE]{};
    };

    struct SendResult
    {
        uint8_t destination_mac[MAC_ADDRESS_SIZE]{};
        esp_now_send_status_t status = ESP_NOW_SEND_FAIL;
    };

    struct Statistics
    {
        uint32_t dropped_receive_packets = 0;
        uint32_t dropped_send_results = 0;
    };

    /**
     * Wi-Fi trebuie sa fie deja initializat si pornit.
     */
    esp_err_t init(const Config &config);

    esp_err_t deinit();

    bool is_initialized();

    esp_err_t set_primary_master_key(
        const uint8_t pmk[ESP_NOW_KEY_LEN]);

    esp_err_t add_peer(const PeerConfig &peer);

    esp_err_t remove_peer(
        const uint8_t mac[MAC_ADDRESS_SIZE]);

    bool peer_exists(
        const uint8_t mac[MAC_ADDRESS_SIZE]);

    esp_err_t send(
        const uint8_t destination_mac[MAC_ADDRESS_SIZE],
        const uint8_t *data,
        std::size_t data_length);

    bool receive(
        ReceivedPacket &packet,
        uint32_t timeout_ms = WAIT_FOREVER);

    bool receive_send_result(
        SendResult &result,
        uint32_t timeout_ms = WAIT_FOREVER);

    void clear_queues();

    Statistics get_statistics();

    void reset_statistics();

    esp_err_t get_local_mac(
        uint8_t mac[MAC_ADDRESS_SIZE]);

    esp_err_t get_esp_now_version(uint32_t &version);
}
