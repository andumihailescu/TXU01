#include "remote_protocol/remote_protocol.h"

#include <cstring>

namespace
{
    constexpr std::size_t OFFSET_MAGIC_0 = 0;
    constexpr std::size_t OFFSET_MAGIC_1 = 1;
    constexpr std::size_t OFFSET_VERSION = 2;
    constexpr std::size_t OFFSET_TYPE = 3;
    constexpr std::size_t OFFSET_DESTINATION = 4;
    constexpr std::size_t OFFSET_FLAGS = 5;
    constexpr std::size_t OFFSET_SEQUENCE = 6;
    constexpr std::size_t OFFSET_MESSAGE_ID = 8;
    constexpr std::size_t OFFSET_PAYLOAD_LENGTH = 10;
    constexpr std::size_t OFFSET_PAYLOAD = 11;

    void writeUint16LittleEndian(
        uint8_t *destination,
        uint16_t value)
    {
        destination[0] =
            static_cast<uint8_t>(value & 0xFFU);

        destination[1] =
            static_cast<uint8_t>((value >> 8U) & 0xFFU);
    }

    uint16_t readUint16LittleEndian(
        const uint8_t *source)
    {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(source[0]) |
            (static_cast<uint16_t>(source[1]) << 8U));
    }
}

namespace remote_protocol
{
    bool is_valid_message_type(MessageType type)
    {
        switch (type)
        {
        case MessageType::Command:
        case MessageType::Telemetry:
        case MessageType::Acknowledgement:
        case MessageType::Configuration:
        case MessageType::Heartbeat:
            return true;

        default:
            return false;
        }
    }

    bool is_valid_destination(Destination destination)
    {
        // 0 este rezervat pentru "invalid".
        return static_cast<uint8_t>(destination) != 0;
    }

    std::size_t encoded_size(uint8_t payload_length)
    {
        if (payload_length > MAX_PAYLOAD_SIZE)
        {
            return 0;
        }

        return HEADER_SIZE +
               static_cast<std::size_t>(payload_length) +
               CRC_SIZE;
    }

    std::size_t encoded_size(const Message &message)
    {
        return encoded_size(message.payload_length);
    }

    uint16_t calculate_crc16(
        const uint8_t *data,
        std::size_t length)
    {
        if (data == nullptr && length != 0)
        {
            return 0;
        }

        uint16_t crc = 0xFFFFU;

        for (std::size_t i = 0; i < length; ++i)
        {
            crc ^= static_cast<uint16_t>(
                static_cast<uint16_t>(data[i]) << 8U);

            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                if ((crc & 0x8000U) != 0)
                {
                    crc = static_cast<uint16_t>(
                        (crc << 1U) ^ 0x1021U);
                }
                else
                {
                    crc = static_cast<uint16_t>(
                        crc << 1U);
                }
            }
        }

        return crc;
    }

    EncodeResult encode(
        const Message &message,
        uint8_t *output,
        std::size_t output_capacity,
        std::size_t &bytes_written)
    {
        bytes_written = 0;

        if (output == nullptr)
        {
            return EncodeResult::NullPointer;
        }

        if (!is_valid_message_type(message.type))
        {
            return EncodeResult::InvalidMessageType;
        }

        if (!is_valid_destination(message.destination))
        {
            return EncodeResult::InvalidDestination;
        }

        if (message.payload_length > MAX_PAYLOAD_SIZE)
        {
            return EncodeResult::PayloadTooLarge;
        }

        const std::size_t required_size =
            encoded_size(message);

        if (output_capacity < required_size)
        {
            return EncodeResult::OutputTooSmall;
        }

        output[OFFSET_MAGIC_0] = MAGIC_BYTE_0;
        output[OFFSET_MAGIC_1] = MAGIC_BYTE_1;
        output[OFFSET_VERSION] = PROTOCOL_VERSION;

        output[OFFSET_TYPE] =
            static_cast<uint8_t>(message.type);

        output[OFFSET_DESTINATION] =
            static_cast<uint8_t>(message.destination);

        output[OFFSET_FLAGS] = message.flags;

        writeUint16LittleEndian(
            &output[OFFSET_SEQUENCE],
            message.sequence_number);

        writeUint16LittleEndian(
            &output[OFFSET_MESSAGE_ID],
            message.message_id);

        output[OFFSET_PAYLOAD_LENGTH] =
            message.payload_length;

        if (message.payload_length > 0)
        {
            std::memcpy(
                &output[OFFSET_PAYLOAD],
                message.payload,
                message.payload_length);
        }

        const std::size_t crc_offset =
            HEADER_SIZE + message.payload_length;

        const uint16_t crc = calculate_crc16(
            output,
            crc_offset);

        writeUint16LittleEndian(
            &output[crc_offset],
            crc);

        bytes_written = required_size;

        return EncodeResult::Ok;
    }

    DecodeResult decode(
        const uint8_t *packet,
        std::size_t packet_length,
        Message &message)
    {
        if (packet == nullptr)
        {
            return DecodeResult::NullPointer;
        }

        if (packet_length < MIN_PACKET_SIZE)
        {
            return DecodeResult::PacketTooShort;
        }

        if (packet[OFFSET_MAGIC_0] != MAGIC_BYTE_0 ||
            packet[OFFSET_MAGIC_1] != MAGIC_BYTE_1)
        {
            return DecodeResult::InvalidMagic;
        }

        if (packet[OFFSET_VERSION] != PROTOCOL_VERSION)
        {
            return DecodeResult::UnsupportedVersion;
        }

        const MessageType type =
            static_cast<MessageType>(
                packet[OFFSET_TYPE]);

        if (!is_valid_message_type(type))
        {
            return DecodeResult::InvalidMessageType;
        }

        const Destination destination =
            static_cast<Destination>(
                packet[OFFSET_DESTINATION]);

        if (!is_valid_destination(destination))
        {
            return DecodeResult::InvalidDestination;
        }

        const uint8_t payload_length =
            packet[OFFSET_PAYLOAD_LENGTH];

        if (payload_length > MAX_PAYLOAD_SIZE)
        {
            return DecodeResult::PayloadTooLarge;
        }

        const std::size_t expected_length =
            encoded_size(payload_length);

        if (packet_length != expected_length)
        {
            return DecodeResult::InvalidPacketLength;
        }

        const std::size_t crc_offset =
            HEADER_SIZE + payload_length;

        const uint16_t received_crc =
            readUint16LittleEndian(
                &packet[crc_offset]);

        const uint16_t calculated_crc =
            calculate_crc16(
                packet,
                crc_offset);

        if (received_crc != calculated_crc)
        {
            return DecodeResult::CrcMismatch;
        }

        Message decoded{};

        decoded.type = type;
        decoded.destination = destination;
        decoded.flags = packet[OFFSET_FLAGS];

        decoded.sequence_number =
            readUint16LittleEndian(
                &packet[OFFSET_SEQUENCE]);

        decoded.message_id =
            readUint16LittleEndian(
                &packet[OFFSET_MESSAGE_ID]);

        decoded.payload_length = payload_length;

        if (payload_length > 0)
        {
            std::memcpy(
                decoded.payload,
                &packet[OFFSET_PAYLOAD],
                payload_length);
        }

        message = decoded;

        return DecodeResult::Ok;
    }

    const char *to_string(EncodeResult result)
    {
        switch (result)
        {
        case EncodeResult::Ok:
            return "Ok";
        case EncodeResult::NullPointer:
            return "NullPointer";
        case EncodeResult::InvalidMessageType:
            return "InvalidMessageType";
        case EncodeResult::InvalidDestination:
            return "InvalidDestination";
        case EncodeResult::PayloadTooLarge:
            return "PayloadTooLarge";
        case EncodeResult::OutputTooSmall:
            return "OutputTooSmall";
        default:
            return "Unknown";
        }
    }

    const char *to_string(DecodeResult result)
    {
        switch (result)
        {
        case DecodeResult::Ok:
            return "Ok";
        case DecodeResult::NullPointer:
            return "NullPointer";
        case DecodeResult::PacketTooShort:
            return "PacketTooShort";
        case DecodeResult::InvalidMagic:
            return "InvalidMagic";
        case DecodeResult::UnsupportedVersion:
            return "UnsupportedVersion";
        case DecodeResult::InvalidMessageType:
            return "InvalidMessageType";
        case DecodeResult::InvalidDestination:
            return "InvalidDestination";
        case DecodeResult::PayloadTooLarge:
            return "PayloadTooLarge";
        case DecodeResult::InvalidPacketLength:
            return "InvalidPacketLength";
        case DecodeResult::CrcMismatch:
            return "CrcMismatch";
        default:
            return "Unknown";
        }
    }
}
