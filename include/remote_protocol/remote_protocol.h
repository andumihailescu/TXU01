#pragma once

#include <cstddef>
#include <cstdint>

namespace remote_protocol
{
    constexpr uint8_t PROTOCOL_VERSION = 1;

    constexpr uint8_t MAGIC_BYTE_0 = 'R';
    constexpr uint8_t MAGIC_BYTE_1 = 'C';

    constexpr std::size_t MAX_PAYLOAD_SIZE = 64;

    /**
     * Wire format:
     *
     * Byte 0      : 'R'
     * Byte 1      : 'C'
     * Byte 2      : protocol version
     * Byte 3      : message type
     * Byte 4      : flags
     * Byte 5..6   : sequence number, little-endian
     * Byte 7..8   : message ID, little-endian
     * Byte 9      : payload length
     * Byte 10..N  : payload
     * Ultimii 2   : CRC16-CCITT, little-endian
     */
    constexpr std::size_t HEADER_SIZE = 10;
    constexpr std::size_t CRC_SIZE = 2;
    constexpr std::size_t MIN_PACKET_SIZE =
        HEADER_SIZE + CRC_SIZE;
    constexpr std::size_t MAX_PACKET_SIZE =
        HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;

    enum class MessageType : uint8_t
    {
        Command = 1,
        Telemetry = 2,
        Acknowledgement = 3,
        Configuration = 4,
        Heartbeat = 5
    };

    enum MessageFlags : uint8_t
    {
        FlagNone = 0,
        FlagAckRequested = 1U << 0,
        FlagIsResponse = 1U << 1
    };

    constexpr uint8_t VALID_FLAGS_MASK =
        FlagAckRequested | FlagIsResponse;

    struct Message
    {
        MessageType type = MessageType::Command;
        uint8_t flags = FlagNone;

        uint16_t sequence_number = 0;
        uint16_t message_id = 0;

        uint8_t payload_length = 0;
        uint8_t payload[MAX_PAYLOAD_SIZE]{};
    };

    enum class EncodeResult
    {
        Ok,
        NullPointer,
        InvalidMessageType,
        InvalidFlags,
        PayloadTooLarge,
        OutputTooSmall
    };

    enum class DecodeResult
    {
        Ok,
        NullPointer,
        PacketTooShort,
        InvalidMagic,
        UnsupportedVersion,
        InvalidMessageType,
        InvalidFlags,
        PayloadTooLarge,
        InvalidPacketLength,
        CrcMismatch
    };

    bool is_valid_message_type(MessageType type);

    bool are_valid_flags(uint8_t flags);

    std::size_t encoded_size(uint8_t payload_length);

    std::size_t encoded_size(const Message &message);

    uint16_t calculate_crc16(
        const uint8_t *data,
        std::size_t length);

    EncodeResult encode(
        const Message &message,
        uint8_t *output,
        std::size_t output_capacity,
        std::size_t &bytes_written);

    DecodeResult decode(
        const uint8_t *packet,
        std::size_t packet_length,
        Message &message);

    const char *to_string(EncodeResult result);

    const char *to_string(DecodeResult result);
}
