#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_now_manager/esp_now_manager.h"

extern "C" void app_main(void)
{
    esp_now_manager::init();

    uint32_t counter = 0;

    while (true)
    {
        esp_now_manager::send_test_message(counter);
        ++counter;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
