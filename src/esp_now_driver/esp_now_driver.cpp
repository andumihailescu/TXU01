#include "esp_now_driver/esp_now_driver.h"

#include <atomic>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_wifi.h"

namespace
{
    esp_now_driver::Config g_config{};

    QueueHandle_t g_receive_queue = nullptr;
    QueueHandle_t g_send_result_queue = nullptr;

    std::atomic<bool> g_initialized{false};

    std::atomic<uint32_t> g_dropped_receive_packets{0};
    std::atomic<uint32_t> g_dropped_send_results{0};

    TickType_t timeoutToTicks(uint32_t timeout_ms)
    {
        if (timeout_ms == esp_now_driver::WAIT_FOREVER)
        {
            return portMAX_DELAY;
        }

        return pdMS_TO_TICKS(timeout_ms);
    }

    bool isBroadcastAddress(
        const uint8_t mac[esp_now_driver::MAC_ADDRESS_SIZE])
    {
        return std::memcmp(
                   mac,
                   esp_now_driver::BROADCAST_MAC,
                   esp_now_driver::MAC_ADDRESS_SIZE) == 0;
    }

    void deleteQueues()
    {
        if (g_receive_queue != nullptr)
        {
            vQueueDelete(g_receive_queue);
            g_receive_queue = nullptr;
        }

        if (g_send_result_queue != nullptr)
        {
            vQueueDelete(g_send_result_queue);
            g_send_result_queue = nullptr;
        }
    }

    void onDataReceived(
        const esp_now_recv_info_t *info,
        const uint8_t *data,
        int data_length)
    {
        if (!g_initialized.load(std::memory_order_acquire) ||
            g_receive_queue == nullptr ||
            info == nullptr ||
            info->src_addr == nullptr ||
            data == nullptr ||
            data_length <= 0 ||
            data_length >
                static_cast<int>(esp_now_driver::MAX_PAYLOAD_SIZE))
        {
            return;
        }

        esp_now_driver::ReceivedPacket packet{};

        std::memcpy(
            packet.source_mac,
            info->src_addr,
            esp_now_driver::MAC_ADDRESS_SIZE);

        if (info->des_addr != nullptr)
        {
            std::memcpy(
                packet.destination_mac,
                info->des_addr,
                esp_now_driver::MAC_ADDRESS_SIZE);
        }

        if (info->rx_ctrl != nullptr)
        {
            packet.rssi = info->rx_ctrl->rssi;
        }

        packet.data_length =
            static_cast<uint16_t>(data_length);

        std::memcpy(
            packet.data,
            data,
            static_cast<std::size_t>(data_length));

        // Callbackul ruleaza in taskul Wi-Fi, deci nu il blocam.
        if (xQueueSend(
                g_receive_queue,
                &packet,
                0) != pdTRUE)
        {
            g_dropped_receive_packets.fetch_add(
                1,
                std::memory_order_relaxed);
        }
    }

    void onDataSent(
        const esp_now_send_info_t *tx_info,
        esp_now_send_status_t status)
    {
        if (!g_initialized.load(std::memory_order_acquire) ||
            g_send_result_queue == nullptr ||
            tx_info == nullptr ||
            tx_info->des_addr == nullptr)
        {
            return;
        }

        esp_now_driver::SendResult result{};

        std::memcpy(
            result.destination_mac,
            tx_info->des_addr,
            esp_now_driver::MAC_ADDRESS_SIZE);

        result.status = status;

        // Callbackul ruleaza in taskul Wi-Fi, deci nu il blocam.
        if (xQueueSend(
                g_send_result_queue,
                &result,
                0) != pdTRUE)
        {
            g_dropped_send_results.fetch_add(
                1,
                std::memory_order_relaxed);
        }
    }
}

namespace esp_now_driver
{
    esp_err_t init(const Config &config)
    {
        if (g_initialized.load(std::memory_order_acquire))
        {
            return ESP_OK;
        }

        if (config.receive_queue_depth == 0 ||
            config.send_result_queue_depth == 0 ||
            config.default_peer_channel > 13)
        {
            return ESP_ERR_INVALID_ARG;
        }

        g_receive_queue = xQueueCreate(
            config.receive_queue_depth,
            sizeof(ReceivedPacket));

        if (g_receive_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }

        g_send_result_queue = xQueueCreate(
            config.send_result_queue_depth,
            sizeof(SendResult));

        if (g_send_result_queue == nullptr)
        {
            deleteQueues();
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = esp_now_init();

        if (err != ESP_OK)
        {
            deleteQueues();
            return err;
        }

        err = esp_now_register_recv_cb(onDataReceived);

        if (err != ESP_OK)
        {
            esp_now_deinit();
            deleteQueues();
            return err;
        }

        err = esp_now_register_send_cb(onDataSent);

        if (err != ESP_OK)
        {
            esp_now_unregister_recv_cb();
            esp_now_deinit();
            deleteQueues();
            return err;
        }

        g_config = config;

        reset_statistics();

        g_initialized.store(
            true,
            std::memory_order_release);

        return ESP_OK;
    }

    esp_err_t deinit()
    {
        if (!g_initialized.exchange(
                false,
                std::memory_order_acq_rel))
        {
            return ESP_OK;
        }

        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();

        const esp_err_t result = esp_now_deinit();

        deleteQueues();
        g_config = {};

        return result;
    }

    bool is_initialized()
    {
        return g_initialized.load(
            std::memory_order_acquire);
    }

    esp_err_t set_primary_master_key(
        const uint8_t pmk[ESP_NOW_KEY_LEN])
    {
        if (!is_initialized())
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (pmk == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        return esp_now_set_pmk(pmk);
    }

    esp_err_t add_peer(const PeerConfig &peer)
    {
        if (!is_initialized())
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (peer.channel > 13)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (peer.encrypt &&
            isBroadcastAddress(peer.mac))
        {
            return ESP_ERR_INVALID_ARG;
        }

        esp_now_peer_info_t native_peer{};

        std::memcpy(
            native_peer.peer_addr,
            peer.mac,
            MAC_ADDRESS_SIZE);

        native_peer.channel =
            peer.channel == 0
                ? g_config.default_peer_channel
                : peer.channel;

        native_peer.ifidx = peer.interface;
        native_peer.encrypt = peer.encrypt;

        if (peer.encrypt)
        {
            std::memcpy(
                native_peer.lmk,
                peer.lmk,
                ESP_NOW_KEY_LEN);
        }

        if (esp_now_is_peer_exist(peer.mac))
        {
            return esp_now_mod_peer(&native_peer);
        }

        return esp_now_add_peer(&native_peer);
    }

    esp_err_t remove_peer(
        const uint8_t mac[MAC_ADDRESS_SIZE])
    {
        if (!is_initialized())
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (mac == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (!esp_now_is_peer_exist(mac))
        {
            return ESP_ERR_ESPNOW_NOT_FOUND;
        }

        return esp_now_del_peer(mac);
    }

    bool peer_exists(
        const uint8_t mac[MAC_ADDRESS_SIZE])
    {
        return is_initialized() &&
               mac != nullptr &&
               esp_now_is_peer_exist(mac);
    }

    esp_err_t send(
        const uint8_t destination_mac[MAC_ADDRESS_SIZE],
        const uint8_t *data,
        std::size_t data_length)
    {
        if (!is_initialized())
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (destination_mac == nullptr ||
            data == nullptr ||
            data_length == 0 ||
            data_length > MAX_PAYLOAD_SIZE)
        {
            return ESP_ERR_INVALID_ARG;
        }

        if (!esp_now_is_peer_exist(destination_mac))
        {
            return ESP_ERR_ESPNOW_NOT_FOUND;
        }

        return esp_now_send(
            destination_mac,
            data,
            data_length);
    }

    bool receive(
        ReceivedPacket &packet,
        uint32_t timeout_ms)
    {
        if (!is_initialized() ||
            g_receive_queue == nullptr)
        {
            return false;
        }

        return xQueueReceive(
                   g_receive_queue,
                   &packet,
                   timeoutToTicks(timeout_ms)) == pdTRUE;
    }

    bool receive_send_result(
        SendResult &result,
        uint32_t timeout_ms)
    {
        if (!is_initialized() ||
            g_send_result_queue == nullptr)
        {
            return false;
        }

        return xQueueReceive(
                   g_send_result_queue,
                   &result,
                   timeoutToTicks(timeout_ms)) == pdTRUE;
    }

    void clear_queues()
    {
        if (g_receive_queue != nullptr)
        {
            xQueueReset(g_receive_queue);
        }

        if (g_send_result_queue != nullptr)
        {
            xQueueReset(g_send_result_queue);
        }
    }

    Statistics get_statistics()
    {
        Statistics statistics{};

        statistics.dropped_receive_packets =
            g_dropped_receive_packets.load(
                std::memory_order_relaxed);

        statistics.dropped_send_results =
            g_dropped_send_results.load(
                std::memory_order_relaxed);

        return statistics;
    }

    void reset_statistics()
    {
        g_dropped_receive_packets.store(
            0,
            std::memory_order_relaxed);

        g_dropped_send_results.store(
            0,
            std::memory_order_relaxed);
    }

    esp_err_t get_local_mac(
        uint8_t mac[MAC_ADDRESS_SIZE])
    {
        if (!is_initialized())
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (mac == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        return esp_wifi_get_mac(
            g_config.interface,
            mac);
    }

    esp_err_t get_esp_now_version(uint32_t &version)
    {
        if (!is_initialized())
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_now_get_version(&version);
    }
}
