#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#include "vehicle_can_protocol/vehicle_can_protocol.h"

namespace light_mode_selector
{
    struct Config
    {
        gpio_num_t gpio = GPIO_NUM_7;
        uint16_t drl_upper_raw = 1365;
        uint16_t positions_upper_raw = 2730;
        uint16_t hysteresis_raw = 80;
        uint8_t samples_per_read = 8;
        uint32_t debounce_ms = 60;
    };

    struct Update
    {
        vehicle_can_protocol::ExteriorLightMode mode =
            vehicle_can_protocol::ExteriorLightMode::Off;
        bool changed = false;
        esp_err_t read_result = ESP_OK;
        uint16_t raw = 0;
    };

    class LightModeSelector
    {
    public:
        explicit LightModeSelector(Config config = {});

        esp_err_t init();
        Update update();
        vehicle_can_protocol::ExteriorLightMode mode() const;
        uint16_t raw() const;

    private:
        esp_err_t read_average(uint16_t &raw);

        Config config_;
        adc_oneshot_unit_handle_t adc_handle_ = nullptr;
        adc_channel_t adc_channel_ = ADC_CHANNEL_0;
        bool initialized_ = false;
        vehicle_can_protocol::ExteriorLightMode candidate_mode_ =
            vehicle_can_protocol::ExteriorLightMode::Off;
        vehicle_can_protocol::ExteriorLightMode stable_mode_ =
            vehicle_can_protocol::ExteriorLightMode::Off;
        int64_t candidate_since_us_ = 0;
        uint16_t raw_ = 0;
    };
}
