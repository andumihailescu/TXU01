#include "esp_now_manager/esp_now_manager.h"

#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"

namespace
{
    constexpr char TAG[] = "ESP_NOW";

    constexpr uint8_t ESPNOW_CHANNEL = 1;
    constexpr uint32_t SEND_INTERVAL_MS = 1000;

    constexpr uint8_t PEER_MAC[ESP_NOW_ETH_ALEN] = {
        0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF};

    struct EspNowMessage
    {
        uint8_t data[8];
    };

    static_assert(
        sizeof(EspNowMessage) == 8,
        "Mesajul ESP-NOW trebuie sa aiba exact 8 bytes");

    bool g_initialized = false;

    void initNvs()
    {
        esp_err_t err = nvs_flash_init();

        if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
            err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }

        ESP_ERROR_CHECK(err);
    }

    void initWifi()
    {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();

        ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_ERROR_CHECK(
            esp_wifi_set_channel(
                ESPNOW_CHANNEL,
                WIFI_SECOND_CHAN_NONE));

        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

        uint8_t local_mac[ESP_NOW_ETH_ALEN] = {};

        ESP_ERROR_CHECK(
            esp_wifi_get_mac(
                WIFI_IF_STA,
                local_mac));

        ESP_LOGI(
            TAG,
            "MAC local STA: " MACSTR,
            MAC2STR(local_mac));
    }

    void onDataSent(
        const esp_now_send_info_t *tx_info,
        esp_now_send_status_t status)
    {
        if (tx_info == nullptr ||
            tx_info->des_addr == nullptr)
        {
            ESP_LOGW(TAG, "Callback trimitere fara adresa destinatie");
            return;
        }

        ESP_LOGI(
            TAG,
            "Trimis catre " MACSTR ": %s",
            MAC2STR(tx_info->des_addr),
            status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    }

    void onDataReceived(
        const esp_now_recv_info_t *info,
        const uint8_t *data,
        int data_len)
    {
        if (info == nullptr ||
            info->src_addr == nullptr ||
            data == nullptr)
        {
            ESP_LOGW(TAG, "Pachet ESP-NOW invalid");
            return;
        }

        if (data_len != static_cast<int>(sizeof(EspNowMessage)))
        {
            ESP_LOGW(
                TAG,
                "Primit %d bytes de la " MACSTR
                ", asteptati %u",
                data_len,
                MAC2STR(info->src_addr),
                static_cast<unsigned>(sizeof(EspNowMessage)));

            return;
        }

        EspNowMessage message{};

        std::memcpy(
            message.data,
            data,
            sizeof(message.data));

        ESP_LOGI(
            TAG,
            "Primit de la " MACSTR
            ": %02X %02X %02X %02X %02X %02X %02X %02X",
            MAC2STR(info->src_addr),
            message.data[0],
            message.data[1],
            message.data[2],
            message.data[3],
            message.data[4],
            message.data[5],
            message.data[6],
            message.data[7]);
    }

    void initEspNow()
    {
        ESP_ERROR_CHECK(esp_now_init());

        ESP_ERROR_CHECK(
            esp_now_register_send_cb(onDataSent));

        ESP_ERROR_CHECK(
            esp_now_register_recv_cb(onDataReceived));

        esp_now_peer_info_t peer_info{};

        std::memcpy(
            peer_info.peer_addr,
            PEER_MAC,
            sizeof(PEER_MAC));

        peer_info.channel = ESPNOW_CHANNEL;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;

        if (!esp_now_is_peer_exist(PEER_MAC))
        {
            ESP_ERROR_CHECK(
                esp_now_add_peer(&peer_info));
        }

        ESP_LOGI(
            TAG,
            "Peer ESP-NOW configurat: " MACSTR,
            MAC2STR(PEER_MAC));
    }
} // namespace

namespace esp_now_manager
{
    void init()
    {
        if (g_initialized)
        {
            return;
        }

        initNvs();
        initWifi();
        initEspNow();
        g_initialized = true;
    }

    void send_test_message(uint32_t counter)
    {
        EspNowMessage message{};

        message.data[0] = static_cast<uint8_t>(counter);
        message.data[1] = static_cast<uint8_t>(counter >> 8);
        message.data[2] = static_cast<uint8_t>(counter >> 16);
        message.data[3] = static_cast<uint8_t>(counter >> 24);

        message.data[4] = 0x11;
        message.data[5] = 0x22;
        message.data[6] = 0x33;
        message.data[7] = 0x44;

        const esp_err_t err = esp_now_send(
            PEER_MAC,
            reinterpret_cast<const uint8_t *>(&message),
            sizeof(message));

        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "Eroare esp_now_send: %s",
                esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(
                TAG,
                "Trimis counter=%lu: "
                "%02X %02X %02X %02X %02X %02X %02X %02X",
                static_cast<unsigned long>(counter),
                message.data[0],
                message.data[1],
                message.data[2],
                message.data[3],
                message.data[4],
                message.data[5],
                message.data[6],
                message.data[7]);
        }
    }
} // namespace esp_now_manager
