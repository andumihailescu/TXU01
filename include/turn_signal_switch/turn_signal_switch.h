#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

#include "turn_signal/turn_signal.h"

namespace turn_signal_switch
{
    struct Config
    {
        gpio_num_t left_gpio = GPIO_NUM_4;
        gpio_num_t right_gpio = GPIO_NUM_5;
        uint32_t debounce_ms = 40;
    };

    struct Update
    {
        turn_signal::TurnInput input{};
        bool changed = false;
    };

    class TurnSignalSwitch
    {
    public:
        explicit TurnSignalSwitch(Config config = {});

        esp_err_t init();
        Update update();
        turn_signal::TurnInput input() const;

    private:
        turn_signal::TurnInput read_input() const;

        Config config_;
        bool initialized_ = false;
        turn_signal::TurnInput candidate_input_{};
        turn_signal::TurnInput stable_input_{};
        int64_t candidate_since_us_ = 0;
    };
}
