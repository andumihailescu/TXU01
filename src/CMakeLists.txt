idf_component_register(
    SRCS
        "main.cpp"
        "application/application.cpp"
        "wifi_manager/wifi_manager.cpp"
        "esp_now_driver/esp_now_driver.cpp"
        "remote_protocol/remote_protocol.cpp"
        "reliable_command_sender/acknowledgement.cpp"
        "reliable_command_sender/reliable_command_sender.cpp"
        "turn_signal_switch/turn_signal_switch.cpp"
        "warning_switch/warning_switch.cpp"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        freertos
        esp_driver_gpio
        esp_timer
        esp_hw_support
)
