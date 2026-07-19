#include "light_mode_selector/light_mode_selector.h"

#include <cstdint>

#include "esp_timer.h"

#include "lighting/lighting_state.h"

namespace light_mode_selector
{
    LightModeSelector::LightModeSelector(Config config)
        : config_(config)
    {
    }

    esp_err_t LightModeSelector::init()
    {
        if (initialized_)
        {
            return ESP_OK;
        }

        if (!GPIO_IS_VALID_GPIO(config_.gpio) ||
            !lighting::are_mode_thresholds_valid(
                config_.drl_upper_raw,
                config_.positions_upper_raw,
                config_.hysteresis_raw) ||
            config_.samples_per_read == 0 ||
            config_.debounce_ms == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        adc_unit_t adc_unit = ADC_UNIT_1;
        esp_err_t result = adc_oneshot_io_to_channel(
            static_cast<int>(config_.gpio),
            &adc_unit,
            &adc_channel_);

        // ESP-NOW uses Wi-Fi continuously. Keeping the selector on ADC1
        // avoids the ADC2/Wi-Fi arbitration limitation.
        if (result != ESP_OK || adc_unit != ADC_UNIT_1)
        {
            return ESP_ERR_INVALID_ARG;
        }

        adc_oneshot_unit_init_cfg_t unit_config{};
        unit_config.unit_id = adc_unit;
        unit_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
        unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;

        result = adc_oneshot_new_unit(
            &unit_config,
            &adc_handle_);

        if (result != ESP_OK)
        {
            return result;
        }

        adc_oneshot_chan_cfg_t channel_config{};
        channel_config.atten = ADC_ATTEN_DB_12;
        channel_config.bitwidth = ADC_BITWIDTH_12;

        result = adc_oneshot_config_channel(
            adc_handle_,
            adc_channel_,
            &channel_config);

        if (result != ESP_OK)
        {
            adc_oneshot_del_unit(adc_handle_);
            adc_handle_ = nullptr;
            return result;
        }

        result = read_average(raw_);

        if (result == ESP_OK)
        {
            stable_mode_ = lighting::decode_mode_raw(
                raw_,
                config_.drl_upper_raw,
                config_.positions_upper_raw);
        }
        else
        {
            // A transient ADC failure must not disable the other TXU01
            // controls. Remain fail-safe with exterior lights OFF and let
            // update() recover after the first stable successful reading.
            raw_ = 0;
            stable_mode_ =
                vehicle_can_protocol::ExteriorLightMode::Off;
        }

        candidate_mode_ = stable_mode_;
        candidate_since_us_ = esp_timer_get_time();
        initialized_ = true;
        return ESP_OK;
    }

    Update LightModeSelector::update()
    {
        Update result{stable_mode_, false, ESP_OK, raw_};

        if (!initialized_)
        {
            result.read_result = ESP_ERR_INVALID_STATE;
            return result;
        }

        uint16_t sampled_raw = raw_;
        result.read_result = read_average(sampled_raw);

        if (result.read_result != ESP_OK)
        {
            return result;
        }

        raw_ = sampled_raw;
        result.raw = raw_;

        const auto sampled_mode =
            lighting::decode_mode_raw_with_hysteresis(
                raw_,
                stable_mode_,
                config_.drl_upper_raw,
                config_.positions_upper_raw,
                config_.hysteresis_raw);
        const int64_t now_us = esp_timer_get_time();

        if (sampled_mode != candidate_mode_)
        {
            candidate_mode_ = sampled_mode;
            candidate_since_us_ = now_us;
        }

        const int64_t debounce_us =
            static_cast<int64_t>(config_.debounce_ms) * 1000;

        if (candidate_mode_ != stable_mode_ &&
            now_us - candidate_since_us_ >= debounce_us)
        {
            stable_mode_ = candidate_mode_;
            result.changed = true;
        }

        result.mode = stable_mode_;
        return result;
    }

    vehicle_can_protocol::ExteriorLightMode
    LightModeSelector::mode() const
    {
        return stable_mode_;
    }

    uint16_t LightModeSelector::raw() const
    {
        return raw_;
    }

    esp_err_t LightModeSelector::read_average(uint16_t &raw)
    {
        uint32_t sum = 0;

        for (uint8_t sample = 0;
             sample < config_.samples_per_read;
             ++sample)
        {
            int sample_raw = 0;
            const esp_err_t result = adc_oneshot_read(
                adc_handle_,
                adc_channel_,
                &sample_raw);

            if (result != ESP_OK)
            {
                return result;
            }

            if (sample_raw < 0 ||
                sample_raw > lighting::ADC_MAX_RAW)
            {
                return ESP_ERR_INVALID_RESPONSE;
            }

            sum += static_cast<uint32_t>(sample_raw);
        }

        raw = static_cast<uint16_t>(
            sum / config_.samples_per_read);
        return ESP_OK;
    }
}
