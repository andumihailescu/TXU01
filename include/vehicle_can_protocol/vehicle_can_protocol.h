#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vehicle_can_protocol
{
    enum class EcuRole : uint8_t
    {
        Lighting = 0x01
    };

    constexpr uint16_t make_remote_message_id(
        EcuRole destination,
        uint8_t command)
    {
        return static_cast<uint16_t>(
            (static_cast<uint16_t>(destination) << 8U) |
            command);
    }

    enum class RemoteCommandId : uint16_t
    {
        LightingGetSoftwareVersion =
            make_remote_message_id(EcuRole::Lighting, 0x01),
        LightingSetTaillightsBrightness =
            make_remote_message_id(EcuRole::Lighting, 0x02),
        LightingSetIndicatorState =
            make_remote_message_id(EcuRole::Lighting, 0x03)
    };

    enum class BinaryState : uint8_t
    {
        Off = 0x00,
        On = 0x01
    };

    enum class IndicatorStateField : std::size_t
    {
        Warning = 0,
        Left = 1,
        Right = 2
    };

    constexpr std::size_t INDICATOR_STATE_PAYLOAD_LENGTH = 3;
    using IndicatorStatePayload =
        std::array<uint8_t, INDICATOR_STATE_PAYLOAD_LENGTH>;

    static_assert(
        static_cast<uint16_t>(
            RemoteCommandId::LightingSetIndicatorState) == 0x0103);
}
