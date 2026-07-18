#include "reliable_command_sender/acknowledgement.h"

namespace reliable_command_sender
{
    AcknowledgementMatch match_acknowledgement(
        const remote_protocol::Message &message,
        uint16_t expected_sequence,
        uint16_t expected_message_id,
        remote_protocol::AcknowledgementStatus &status)
    {
        if (message.sequence_number != expected_sequence ||
            message.message_id != expected_message_id)
        {
            return AcknowledgementMatch::NotForRequest;
        }

        if (message.type !=
                remote_protocol::MessageType::Acknowledgement ||
            message.flags != remote_protocol::FlagIsResponse ||
            message.payload_length != 1)
        {
            return AcknowledgementMatch::Invalid;
        }

        const uint8_t raw_status = message.payload[0];

        if (raw_status > static_cast<uint8_t>(
                             remote_protocol::AcknowledgementStatus::
                                 InvalidPayloadValue))
        {
            return AcknowledgementMatch::Invalid;
        }

        status = static_cast<remote_protocol::AcknowledgementStatus>(
            raw_status);

        return status ==
                       remote_protocol::AcknowledgementStatus::CanTransmitted
                   ? AcknowledgementMatch::Success
                   : AcknowledgementMatch::RemoteRejected;
    }
}
