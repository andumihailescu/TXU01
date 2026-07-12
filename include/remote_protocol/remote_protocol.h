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
     * Byte 4      : destination
     * Byte 5      : flags
     * Byte 6..7   : sequence number, little-endian
     * Byte 8..9   : message ID, little-endian
     * Byte 10     : payload length
     * Byte 11..N  : payload
     * Ultimii 2   : CRC16-CCITT, little-endian
     */
    constexpr std::size_t HEADER_SIZE = 11;
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

    /**
     * Valorile sunt ID-uri logice.
     * Se pot adauga ulterior alte ECU-uri fara modificarea wire format-ului.
     */
    enum class Destination : uint8_t
    {
        Rxu01 = 0x01,
        Lmcu100 = 0x10,
        Scm110 = 0x11,
        Broadcast = 0xFF
    };

    enum MessageFlags : uint8_t
    {
        FlagNone = 0,
        FlagAckRequested = 1U << 0,
        FlagIsResponse = 1U << 1
    };

    struct Message
    {
        MessageType type = MessageType::Command;
        Destination destination = Destination::Rxu01;

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
        InvalidDestination,
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
        InvalidDestination,
        PayloadTooLarge,
        InvalidPacketLength,
        CrcMismatch
    };

    bool is_valid_message_type(MessageType type);

    bool is_valid_destination(Destination destination);

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
