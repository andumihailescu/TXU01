#include "wifi_manager/wifi_manager.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

namespace
{
    bool g_initialized = false;
    wifi_manager::Config g_config{};

    bool isValidChannel(uint8_t channel)
    {
        // Canalele 1-13 sunt valabile in regiunea europeana.
        return channel >= 1 && channel <= 13;
    }

    esp_err_t initNvs()
    {
        esp_err_t err = nvs_flash_init();

        if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
            err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            err = nvs_flash_erase();

            if (err != ESP_OK)
            {
                return err;
            }

            err = nvs_flash_init();
        }

        return err;
    }

    void cleanupWifiAfterFailedInit(bool wifi_started)
    {
        if (wifi_started)
        {
            esp_wifi_stop();
        }

        esp_wifi_deinit();
    }
}

namespace wifi_manager
{
    esp_err_t init(const Config &config)
    {
        if (g_initialized)
        {
            return ESP_OK;
        }

        if (!isValidChannel(config.channel))
        {
            return ESP_ERR_INVALID_ARG;
        }

        esp_err_t err = initNvs();

        if (err != ESP_OK)
        {
            return err;
        }

        err = esp_netif_init();

        if (err != ESP_OK &&
            err != ESP_ERR_INVALID_STATE)
        {
            return err;
        }

        err = esp_event_loop_create_default();

        if (err != ESP_OK &&
            err != ESP_ERR_INVALID_STATE)
        {
            return err;
        }

        wifi_init_config_t wifi_config =
            WIFI_INIT_CONFIG_DEFAULT();

        err = esp_wifi_init(&wifi_config);

        if (err != ESP_OK)
        {
            return err;
        }

        bool wifi_started = false;

        err = esp_wifi_set_storage(config.storage);

        if (err != ESP_OK)
        {
            cleanupWifiAfterFailedInit(wifi_started);
            return err;
        }

        err = esp_wifi_set_mode(WIFI_MODE_STA);

        if (err != ESP_OK)
        {
            cleanupWifiAfterFailedInit(wifi_started);
            return err;
        }

        if (config.use_custom_station_mac)
        {
            err = esp_wifi_set_mac(
                WIFI_IF_STA,
                config.station_mac);

            if (err != ESP_OK)
            {
                cleanupWifiAfterFailedInit(wifi_started);
                return err;
            }
        }

        err = esp_wifi_start();

        if (err != ESP_OK)
        {
            cleanupWifiAfterFailedInit(wifi_started);
            return err;
        }

        wifi_started = true;

        err = esp_wifi_set_channel(
            config.channel,
            WIFI_SECOND_CHAN_NONE);

        if (err != ESP_OK)
        {
            cleanupWifiAfterFailedInit(wifi_started);
            return err;
        }

        err = esp_wifi_set_ps(config.power_save);

        if (err != ESP_OK)
        {
            cleanupWifiAfterFailedInit(wifi_started);
            return err;
        }

        g_config = config;
        g_initialized = true;

        return ESP_OK;
    }

    esp_err_t deinit()
    {
        if (!g_initialized)
        {
            return ESP_OK;
        }

        esp_err_t first_error = ESP_OK;

        const esp_err_t stop_result = esp_wifi_stop();

        if (stop_result != ESP_OK)
        {
            first_error = stop_result;
        }

        const esp_err_t deinit_result = esp_wifi_deinit();

        if (first_error == ESP_OK &&
            deinit_result != ESP_OK)
        {
            first_error = deinit_result;
        }

        g_initialized = false;
        g_config = {};

        return first_error;
    }

    bool is_initialized()
    {
        return g_initialized;
    }

    esp_err_t set_channel(uint8_t channel)
    {
        if (!g_initialized)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (!isValidChannel(channel))
        {
            return ESP_ERR_INVALID_ARG;
        }

        const esp_err_t err = esp_wifi_set_channel(
            channel,
            WIFI_SECOND_CHAN_NONE);

        if (err == ESP_OK)
        {
            g_config.channel = channel;
        }

        return err;
    }

    uint8_t get_channel()
    {
        return g_initialized ? g_config.channel : 0;
    }

    esp_err_t get_station_mac(uint8_t mac[6])
    {
        if (!g_initialized)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (mac == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        return esp_wifi_get_mac(
            WIFI_IF_STA,
            mac);
    }
}
