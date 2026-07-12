idf_component_register(
    SRCS
        "main.cpp"
        "wifi_manager/wifi_manager.cpp"
        "esp_now_driver/esp_now_driver.cpp"
        "remote_protocol/remote_protocol.cpp"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        freertos
)