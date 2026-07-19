#include "momentary_toggle/momentary_toggle.h"

#include "esp_timer.h"

namespace momentary_toggle
{
    MomentaryToggle::MomentaryToggle(Config config)
        : config_(config)
    {
    }

    esp_err_t MomentaryToggle::init()
    {
        if (initialized_)
        {
            return ESP_OK;
        }

        if (!GPIO_IS_VALID_GPIO(config_.gpio) ||
            config_.debounce_ms == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        gpio_config_t native_config{};
        native_config.pin_bit_mask =
            uint64_t{1} << static_cast<uint32_t>(config_.gpio);
        native_config.mode = GPIO_MODE_INPUT;
        native_config.pull_up_en = GPIO_PULLUP_ENABLE;
        native_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        native_config.intr_type = GPIO_INTR_DISABLE;

        const esp_err_t result = gpio_config(&native_config);

        if (result != ESP_OK)
        {
            return result;
        }

        // A button held during boot is not a new press. Every toggle starts
        // fail-safe in the OFF state.
        stable_pressed_ = read_pressed();
        candidate_pressed_ = stable_pressed_;
        candidate_since_us_ = esp_timer_get_time();
        active_ = false;
        initialized_ = true;
        return ESP_OK;
    }

    Update MomentaryToggle::update()
    {
        Update result{active_, false};

        if (!initialized_)
        {
            return result;
        }

        const int64_t now_us = esp_timer_get_time();
        const bool sampled_pressed = read_pressed();

        if (sampled_pressed != candidate_pressed_)
        {
            candidate_pressed_ = sampled_pressed;
            candidate_since_us_ = now_us;
        }

        const int64_t debounce_us =
            static_cast<int64_t>(config_.debounce_ms) * 1000;

        if (candidate_pressed_ != stable_pressed_ &&
            now_us - candidate_since_us_ >= debounce_us)
        {
            stable_pressed_ = candidate_pressed_;

            if (stable_pressed_)
            {
                active_ = !active_;
                result.changed = true;
            }
        }

        result.active = active_;
        return result;
    }

    bool MomentaryToggle::active() const
    {
        return active_;
    }

    bool MomentaryToggle::read_pressed() const
    {
        return gpio_get_level(config_.gpio) == 0;
    }
}
