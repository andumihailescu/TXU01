#pragma once

#include <cstdint>

#include "vehicle_can_protocol/vehicle_can_protocol.h"

namespace turn_signal
{
    enum class TurnRequest : uint8_t
    {
        None = 0,
        Left = 1,
        Right = 2
    };

    enum class WarningRequest : uint8_t
    {
        Off = 0,
        On = 1
    };

    enum class EffectiveState : uint8_t
    {
        Off,
        Left,
        Right,
        Hazard
    };

    struct TurnInput
    {
        TurnRequest request = TurnRequest::None;
        bool valid = false;
    };

    struct RequestedState
    {
        TurnRequest turn = TurnRequest::None;
        WarningRequest warning = WarningRequest::Off;
        bool turn_input_valid = false;
    };

    constexpr bool operator==(
        const TurnInput &left,
        const TurnInput &right)
    {
        return left.request == right.request &&
               left.valid == right.valid;
    }

    constexpr bool operator!=(
        const TurnInput &left,
        const TurnInput &right)
    {
        return !(left == right);
    }

    constexpr bool operator==(
        const RequestedState &left,
        const RequestedState &right)
    {
        return left.turn == right.turn &&
               left.warning == right.warning &&
               left.turn_input_valid == right.turn_input_valid;
    }

    constexpr bool operator!=(
        const RequestedState &left,
        const RequestedState &right)
    {
        return !(left == right);
    }

    constexpr TurnInput decode_turn_levels(
        bool left_level_high,
        bool right_level_high)
    {
        if (!left_level_high && right_level_high)
        {
            return {TurnRequest::Left, true};
        }

        if (left_level_high && !right_level_high)
        {
            return {TurnRequest::Right, true};
        }

        if (left_level_high && right_level_high)
        {
            return {TurnRequest::None, true};
        }

        // Ambele contacte active reprezinta o combinatie invalida.
        // Cererea rezultata ramane fail-safe pe None.
        return {TurnRequest::None, false};
    }

    constexpr WarningRequest decode_warning_level(bool level_high)
    {
        return level_high
                   ? WarningRequest::Off
                   : WarningRequest::On;
    }

    constexpr RequestedState make_requested_state(
        TurnInput turn,
        WarningRequest warning)
    {
        return {turn.request, warning, turn.valid};
    }

    constexpr EffectiveState resolve_effective_state(
        const RequestedState &requested)
    {
        if (requested.warning == WarningRequest::On)
        {
            return EffectiveState::Hazard;
        }

        if (!requested.turn_input_valid)
        {
            return EffectiveState::Off;
        }

        if (requested.turn == TurnRequest::Left)
        {
            return EffectiveState::Left;
        }

        if (requested.turn == TurnRequest::Right)
        {
            return EffectiveState::Right;
        }

        return EffectiveState::Off;
    }

    constexpr vehicle_can_protocol::IndicatorStatePayload
    make_indicator_state_payload(
        const RequestedState &requested)
    {
        const TurnRequest safe_turn_request =
            requested.turn_input_valid
                ? requested.turn
                : TurnRequest::None;

        return {{
            static_cast<uint8_t>(
                requested.warning == WarningRequest::On),
            static_cast<uint8_t>(
                safe_turn_request == TurnRequest::Left),
            static_cast<uint8_t>(
                safe_turn_request == TurnRequest::Right),
        }};
    }

    constexpr const char *to_string(TurnRequest request)
    {
        switch (request)
        {
        case TurnRequest::None:
            return "None";
        case TurnRequest::Left:
            return "Left";
        case TurnRequest::Right:
            return "Right";
        default:
            return "Unknown";
        }
    }

    constexpr const char *to_string(WarningRequest request)
    {
        switch (request)
        {
        case WarningRequest::Off:
            return "Off";
        case WarningRequest::On:
            return "On";
        default:
            return "Unknown";
        }
    }

    constexpr const char *to_string(EffectiveState state)
    {
        switch (state)
        {
        case EffectiveState::Off:
            return "Off";
        case EffectiveState::Left:
            return "Left";
        case EffectiveState::Right:
            return "Right";
        case EffectiveState::Hazard:
            return "Hazard";
        default:
            return "Unknown";
        }
    }
}
