#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

namespace momentary_toggle
{
    struct Config
    {
        gpio_num_t gpio = GPIO_NUM_NC;
        uint32_t debounce_ms = 40;
    };

    struct Update
    {
        bool active = false;
        bool changed = false;
    };

    class MomentaryToggle
    {
    public:
        explicit MomentaryToggle(Config config);

        esp_err_t init();
        Update update();
        bool active() const;

    private:
        bool read_pressed() const;

        Config config_;
        bool initialized_ = false;
        bool candidate_pressed_ = false;
        bool stable_pressed_ = false;
        bool active_ = false;
        int64_t candidate_since_us_ = 0;
    };
}
