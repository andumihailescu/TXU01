#include "turn_signal_switch/turn_signal_switch.h"

#include <cstdint>

#include "esp_timer.h"

namespace turn_signal_switch
{
    TurnSignalSwitch::TurnSignalSwitch(Config config)
        : config_(config)
    {
    }

    esp_err_t TurnSignalSwitch::init()
    {
        if (initialized_)
        {
            return ESP_OK;
        }

        if (!GPIO_IS_VALID_GPIO(config_.left_gpio) ||
            !GPIO_IS_VALID_GPIO(config_.right_gpio) ||
            config_.left_gpio == config_.right_gpio)
        {
            return ESP_ERR_INVALID_ARG;
        }

        gpio_config_t native_config{};
        native_config.pin_bit_mask =
            (uint64_t{1} << static_cast<uint32_t>(config_.left_gpio)) |
            (uint64_t{1} << static_cast<uint32_t>(config_.right_gpio));
        native_config.mode = GPIO_MODE_INPUT;
        native_config.pull_up_en = GPIO_PULLUP_ENABLE;
        native_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        native_config.intr_type = GPIO_INTR_DISABLE;

        const esp_err_t result = gpio_config(&native_config);

        if (result != ESP_OK)
        {
            return result;
        }

        // La pornire pastram iesirea fail-safe pana cand pozitia
        // comutatorului ramane stabila pe toata durata de debounce.
        stable_input_ = {};
        candidate_input_ = read_input();
        candidate_since_us_ = esp_timer_get_time();
        initialized_ = true;

        return ESP_OK;
    }

    Update TurnSignalSwitch::update()
    {
        Update result{stable_input_, false};

        if (!initialized_)
        {
            return result;
        }

        const int64_t now_us = esp_timer_get_time();
        const turn_signal::TurnInput sampled_input = read_input();

        if (sampled_input != candidate_input_)
        {
            candidate_input_ = sampled_input;
            candidate_since_us_ = now_us;
        }

        const int64_t debounce_us =
            static_cast<int64_t>(config_.debounce_ms) * 1000;

        if (candidate_input_ != stable_input_ &&
            now_us - candidate_since_us_ >= debounce_us)
        {
            stable_input_ = candidate_input_;
            result.changed = true;
        }

        result.input = stable_input_;
        return result;
    }

    turn_signal::TurnInput TurnSignalSwitch::input() const
    {
        return stable_input_;
    }

    turn_signal::TurnInput TurnSignalSwitch::read_input() const
    {
        const bool left_high =
            gpio_get_level(config_.left_gpio) != 0;
        const bool right_high =
            gpio_get_level(config_.right_gpio) != 0;

        return turn_signal::decode_turn_levels(
            left_high,
            right_high);
    }
}
