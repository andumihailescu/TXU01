#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace SoftwareVersionConfig
{
    constexpr uint8_t Major = 1;
    constexpr uint8_t Minor = 0;
    constexpr uint8_t Patch = 0;

    // ISO-8601 build calendar information for 2026-07-19.
    constexpr uint8_t IsoWeek = 29;
    constexpr uint16_t IsoYear = 2026;
    constexpr uint8_t IsoYearShort =
        static_cast<uint8_t>(IsoYear % 100U);

    enum class ResponseField : std::size_t
    {
        Major = 0,
        Minor = 1,
        Patch = 2,
        IsoWeek = 3,
        IsoYearShort = 4
    };

    constexpr std::array<uint8_t, 5> ResponsePayload = {
        Major,
        Minor,
        Patch,
        IsoWeek,
        IsoYearShort,
    };

    static_assert(IsoWeek >= 1 && IsoWeek <= 53);
    static_assert(IsoYear >= 2000 && IsoYear <= 2099);
}
