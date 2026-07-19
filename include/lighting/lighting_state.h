#pragma once

#include <cstdint>

#include "vehicle_can_protocol/vehicle_can_protocol.h"

namespace lighting
{
    constexpr uint16_t ADC_MAX_RAW = 4095;

    struct ExteriorState
    {
        vehicle_can_protocol::ExteriorLightMode mode =
            vehicle_can_protocol::ExteriorLightMode::Off;
        bool front_projectors = false;
        bool fog_lights = false;
        bool high_beam = false;
    };

    constexpr bool operator==(
        const ExteriorState &left,
        const ExteriorState &right)
    {
        return left.mode == right.mode &&
               left.front_projectors == right.front_projectors &&
               left.fog_lights == right.fog_lights &&
               left.high_beam == right.high_beam;
    }

    constexpr bool operator!=(
        const ExteriorState &left,
        const ExteriorState &right)
    {
        return !(left == right);
    }

    constexpr vehicle_can_protocol::ExteriorLightMode
    decode_mode_raw(
        uint16_t raw,
        uint16_t drl_upper_raw,
        uint16_t positions_upper_raw)
    {
        if (raw < drl_upper_raw)
        {
            return vehicle_can_protocol::ExteriorLightMode::Drl;
        }

        if (raw < positions_upper_raw)
        {
            return vehicle_can_protocol::ExteriorLightMode::Positions;
        }

        return vehicle_can_protocol::ExteriorLightMode::LowBeam;
    }

    constexpr bool are_mode_thresholds_valid(
        uint16_t drl_upper_raw,
        uint16_t positions_upper_raw,
        uint16_t hysteresis_raw)
    {
        if (drl_upper_raw == 0 ||
            drl_upper_raw >= positions_upper_raw ||
            positions_upper_raw > ADC_MAX_RAW ||
            hysteresis_raw >= drl_upper_raw)
        {
            return false;
        }

        return static_cast<uint32_t>(drl_upper_raw) +
                       hysteresis_raw <
                   static_cast<uint32_t>(positions_upper_raw) -
                       hysteresis_raw &&
               static_cast<uint32_t>(positions_upper_raw) +
                       hysteresis_raw <=
                   ADC_MAX_RAW;
    }

    constexpr vehicle_can_protocol::ExteriorLightMode
    decode_mode_raw_with_hysteresis(
        uint16_t raw,
        vehicle_can_protocol::ExteriorLightMode stable_mode,
        uint16_t drl_upper_raw,
        uint16_t positions_upper_raw,
        uint16_t hysteresis_raw)
    {
        using Mode = vehicle_can_protocol::ExteriorLightMode;

        const uint32_t drl_exit =
            static_cast<uint32_t>(drl_upper_raw) + hysteresis_raw;
        const uint16_t positions_to_drl =
            hysteresis_raw < drl_upper_raw
                ? static_cast<uint16_t>(drl_upper_raw - hysteresis_raw)
                : uint16_t{0};
        const uint32_t positions_to_low =
            static_cast<uint32_t>(positions_upper_raw) + hysteresis_raw;
        const uint16_t low_exit =
            hysteresis_raw < positions_upper_raw
                ? static_cast<uint16_t>(positions_upper_raw - hysteresis_raw)
                : uint16_t{0};

        if (stable_mode == Mode::Drl && raw < drl_exit)
        {
            return Mode::Drl;
        }

        if (stable_mode == Mode::Positions)
        {
            if (raw >= positions_to_drl && raw < positions_to_low)
            {
                return Mode::Positions;
            }
        }

        if (stable_mode == Mode::LowBeam && raw >= low_exit)
        {
            return Mode::LowBeam;
        }

        return decode_mode_raw(
            raw,
            drl_upper_raw,
            positions_upper_raw);
    }

    constexpr vehicle_can_protocol::ExteriorLightStatePayload
    make_exterior_state_payload(const ExteriorState &state)
    {
        return {{
            static_cast<uint8_t>(state.mode),
            static_cast<uint8_t>(state.front_projectors),
            static_cast<uint8_t>(state.fog_lights),
            static_cast<uint8_t>(state.high_beam),
        }};
    }

    constexpr ExteriorState make_panel_exterior_state(
        vehicle_can_protocol::ExteriorLightMode mode,
        bool projectors_active,
        bool fog_lights_active,
        bool high_beam_active)
    {
        // LMCU100 applies one logical fog request to its separate front and
        // rear fog outputs.
        return {
            mode,
            projectors_active,
            fog_lights_active,
            high_beam_active,
        };
    }

    constexpr vehicle_can_protocol::BinaryStatePayload
    make_binary_state_payload(bool active)
    {
        return {{static_cast<uint8_t>(active)}};
    }

    constexpr const char *to_string(
        vehicle_can_protocol::ExteriorLightMode mode)
    {
        using Mode = vehicle_can_protocol::ExteriorLightMode;

        switch (mode)
        {
        case Mode::Off:
            return "Off";
        case Mode::Drl:
            return "DRL";
        case Mode::Positions:
            return "Positions";
        case Mode::LowBeam:
            return "LowBeam";
        default:
            return "Unknown";
        }
    }
}
