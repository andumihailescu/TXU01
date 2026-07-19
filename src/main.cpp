#include "application/application.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void)
{
    const application::Config config{};
    application::Application app{config};

    ESP_ERROR_CHECK(app.init());

    while (true)
    {
        app.process();
        vTaskDelay(pdMS_TO_TICKS(config.loop_interval_ms));
    }
}
