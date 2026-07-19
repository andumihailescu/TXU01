#pragma once

#include <cstdint>

#include "remote_protocol/remote_protocol.h"

namespace reliable_command_sender
{
    enum class AcknowledgementMatch
    {
        NotForRequest,
        Invalid,
        Success,
        RemoteRejected
    };

    AcknowledgementMatch match_acknowledgement(
        const remote_protocol::Message &message,
        uint16_t expected_sequence,
        uint16_t expected_message_id,
        remote_protocol::AcknowledgementStatus &status);
}
