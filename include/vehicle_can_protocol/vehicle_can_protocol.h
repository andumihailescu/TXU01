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
        LightingSetIndicatorState =
            make_remote_message_id(EcuRole::Lighting, 0x03),
        LightingSetExteriorLightsState =
            make_remote_message_id(EcuRole::Lighting, 0x04),
        LightingSetBrakeState =
            make_remote_message_id(EcuRole::Lighting, 0x05),
        LightingSetReverseLightState =
            make_remote_message_id(EcuRole::Lighting, 0x06)
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

    enum class ExteriorLightMode : uint8_t
    {
        Off = 0,
        Drl = 1,
        Positions = 2,
        LowBeam = 3
    };

    enum class ExteriorLightStateField : std::size_t
    {
        Mode = 0,
        FrontProjectors = 1,
        FogLights = 2,
        HighBeam = 3
    };

    constexpr std::size_t EXTERIOR_LIGHT_STATE_PAYLOAD_LENGTH = 4;
    using ExteriorLightStatePayload =
        std::array<uint8_t, EXTERIOR_LIGHT_STATE_PAYLOAD_LENGTH>;

    constexpr std::size_t BINARY_STATE_PAYLOAD_LENGTH = 1;
    using BinaryStatePayload =
        std::array<uint8_t, BINARY_STATE_PAYLOAD_LENGTH>;

    static_assert(
        static_cast<uint16_t>(
            RemoteCommandId::LightingSetIndicatorState) == 0x0103);
    static_assert(
        static_cast<uint16_t>(
            RemoteCommandId::LightingSetExteriorLightsState) == 0x0104);
    static_assert(
        static_cast<uint16_t>(
            RemoteCommandId::LightingSetBrakeState) == 0x0105);
    static_assert(
        static_cast<uint16_t>(
            RemoteCommandId::LightingSetReverseLightState) == 0x0106);
}
