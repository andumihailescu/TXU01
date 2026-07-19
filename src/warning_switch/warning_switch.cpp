#include "warning_switch/warning_switch.h"

#include <cstdint>

#include "esp_timer.h"

namespace warning_switch
{
    WarningSwitch::WarningSwitch(Config config)
        : config_(config)
    {
    }

    esp_err_t WarningSwitch::init()
    {
        if (initialized_)
        {
            return ESP_OK;
        }

        if (!GPIO_IS_VALID_GPIO(config_.gpio) ||
            (config_.input_mode != InputMode::MaintainedLevel &&
             config_.input_mode != InputMode::MomentaryToggle))
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

        stable_pressed_ = read_pressed();
        candidate_pressed_ = stable_pressed_;
        candidate_since_us_ = esp_timer_get_time();

        if (config_.input_mode == InputMode::MaintainedLevel)
        {
            request_ = stable_pressed_
                           ? turn_signal::WarningRequest::On
                           : turn_signal::WarningRequest::Off;
        }
        else
        {
            // Un buton tinut apasat la boot nu este interpretat drept
            // o apasare noua. Starea toggle porneste fail-safe pe Off.
            request_ = turn_signal::WarningRequest::Off;
        }

        initialized_ = true;
        return ESP_OK;
    }

    Update WarningSwitch::update()
    {
        Update result{request_, false};

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

            if (config_.input_mode == InputMode::MaintainedLevel)
            {
                request_ = stable_pressed_
                               ? turn_signal::WarningRequest::On
                               : turn_signal::WarningRequest::Off;
                result.changed = true;
            }
            else if (stable_pressed_)
            {
                request_ =
                    request_ == turn_signal::WarningRequest::Off
                        ? turn_signal::WarningRequest::On
                        : turn_signal::WarningRequest::Off;
                result.changed = true;
            }
        }

        result.request = request_;
        return result;
    }

    turn_signal::WarningRequest WarningSwitch::request() const
    {
        return request_;
    }

    bool WarningSwitch::read_pressed() const
    {
        return gpio_get_level(config_.gpio) == 0;
    }
}
