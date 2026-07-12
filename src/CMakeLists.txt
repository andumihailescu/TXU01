idf_component_register(
    SRCS
        "main.cpp"
        "esp_now_manager/esp_now_manager.cpp"
    REQUIRES esp_event esp_netif esp_wifi nvs_flash
)
