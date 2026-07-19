#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

#include "turn_signal/turn_signal.h"

namespace warning_switch
{
    enum class InputMode
    {
        // Contact mentinut: LOW = Warning On, HIGH = Warning Off.
        MaintainedLevel,

        // Buton momentan: fiecare apasare debounced comuta starea.
        MomentaryToggle
    };

    struct Config
    {
        gpio_num_t gpio = GPIO_NUM_6;
        uint32_t debounce_ms = 40;
        InputMode input_mode = InputMode::MaintainedLevel;
    };

    struct Update
    {
        turn_signal::WarningRequest request =
            turn_signal::WarningRequest::Off;
        bool changed = false;
    };

    class WarningSwitch
    {
    public:
        explicit WarningSwitch(Config config = {});

        esp_err_t init();
        Update update();
        turn_signal::WarningRequest request() const;

    private:
        bool read_pressed() const;

        Config config_;
        bool initialized_ = false;
        bool candidate_pressed_ = false;
        bool stable_pressed_ = false;
        int64_t candidate_since_us_ = 0;
        turn_signal::WarningRequest request_ =
            turn_signal::WarningRequest::Off;
    };
}
