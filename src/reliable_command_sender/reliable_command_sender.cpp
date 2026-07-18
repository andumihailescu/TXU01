#include "reliable_command_sender/reliable_command_sender.h"

#include <cstring>

#include "esp_random.h"
#include "esp_timer.h"

namespace reliable_command_sender
{
    ReliableCommandSender::ReliableCommandSender(Config config)
        : config_(config)
    {
    }

    esp_err_t ReliableCommandSender::init()
    {
        if (initialized_)
        {
            return ESP_OK;
        }

        if ((config_.destination_mac[0] & 0x01U) != 0 ||
            (config_.local_mac[0] & 0x01U) != 0 ||
            config_.destination_mac == config_.local_mac ||
            config_.send_result_timeout_ms == 0 ||
            config_.acknowledgement_timeout_ms == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        next_sequence_ = static_cast<uint16_t>(esp_random());

        esp_now_driver::SendResult stale_send_result{};
        while (esp_now_driver::receive_send_result(
            stale_send_result,
            0))
        {
        }

        initialized_ = true;
        return ESP_OK;
    }

    bool ReliableCommandSender::start(
        uint16_t message_id,
        const uint8_t *payload,
        uint8_t payload_length)
    {
        if (!initialized_ || busy() || result_ready_ ||
            message_id == 0 ||
            payload_length > remote_protocol::MAX_PAYLOAD_SIZE ||
            (payload == nullptr && payload_length != 0))
        {
            return false;
        }

        message_ = {};
        message_.type = remote_protocol::MessageType::Command;
        message_.flags = remote_protocol::FlagAckRequested;
        message_.sequence_number = next_sequence_++;
        message_.message_id = message_id;
        message_.payload_length = payload_length;

        if (payload_length > 0)
        {
            std::memcpy(
                message_.payload,
                payload,
                payload_length);
        }

        const remote_protocol::EncodeResult encode_result =
            remote_protocol::encode(
                message_,
                encoded_packet_,
                sizeof(encoded_packet_),
                encoded_length_);

        attempts_ = 0;

        if (encode_result != remote_protocol::EncodeResult::Ok)
        {
            complete(ResultCode::EncodeFailed);
            return true;
        }

        state_ = State::RetryPending;
        process();
        return true;
    }

    void ReliableCommandSender::process()
    {
        if (!initialized_ || state_ == State::Idle)
        {
            return;
        }

        const int64_t now_us = esp_timer_get_time();

        if (state_ == State::RetryPending)
        {
            start_attempt(now_us);
        }

        if (state_ == State::WaitingDelivery)
        {
            process_delivery(now_us);
        }

        if (state_ == State::WaitingAcknowledgement)
        {
            process_acknowledgement(now_us);
        }
    }

    bool ReliableCommandSender::busy() const
    {
        return state_ != State::Idle;
    }

    bool ReliableCommandSender::take_result(Result &result)
    {
        if (!result_ready_)
        {
            return false;
        }

        result = result_;
        result_ready_ = false;
        return true;
    }

    void ReliableCommandSender::start_attempt(int64_t now_us)
    {
        ++attempts_;

        esp_now_driver::SendResult stale_send_result{};
        while (esp_now_driver::receive_send_result(
            stale_send_result,
            0))
        {
        }

        const esp_err_t send_result = esp_now_driver::send(
            config_.destination_mac.data(),
            encoded_packet_,
            encoded_length_);

        if (send_result != ESP_OK)
        {
            retry_or_complete(
                ResultCode::SendStartFailed,
                send_result);
            return;
        }

        deadline_us_ =
            now_us +
            static_cast<int64_t>(config_.send_result_timeout_ms) * 1000;
        state_ = State::WaitingDelivery;
    }

    void ReliableCommandSender::process_delivery(int64_t now_us)
    {
        esp_now_driver::SendResult send_result{};

        while (esp_now_driver::receive_send_result(send_result, 0))
        {
            if (!is_expected_destination(send_result.destination_mac))
            {
                continue;
            }

            if (send_result.status != ESP_NOW_SEND_SUCCESS)
            {
                retry_or_complete(ResultCode::DeliveryFailed);
                return;
            }

            deadline_us_ =
                now_us +
                static_cast<int64_t>(
                    config_.acknowledgement_timeout_ms) *
                    1000;
            state_ = State::WaitingAcknowledgement;
            return;
        }

        if (now_us >= deadline_us_)
        {
            retry_or_complete(ResultCode::DeliveryTimeout);
        }
    }

    void ReliableCommandSender::process_acknowledgement(int64_t now_us)
    {
        esp_now_driver::ReceivedPacket packet{};

        while (esp_now_driver::receive(packet, 0))
        {
            if (!is_expected_destination(packet.source_mac) ||
                !is_local_destination(packet.destination_mac))
            {
                continue;
            }

            remote_protocol::Message acknowledgement{};

            if (remote_protocol::decode(
                    packet.data,
                    packet.data_length,
                    acknowledgement) !=
                remote_protocol::DecodeResult::Ok)
            {
                continue;
            }

            remote_protocol::AcknowledgementStatus status =
                remote_protocol::AcknowledgementStatus::InvalidMessage;

            const AcknowledgementMatch match = match_acknowledgement(
                acknowledgement,
                message_.sequence_number,
                message_.message_id,
                status);

            switch (match)
            {
            case AcknowledgementMatch::NotForRequest:
                continue;
            case AcknowledgementMatch::Invalid:
                complete(ResultCode::InvalidAcknowledgement);
                return;
            case AcknowledgementMatch::Success:
                complete(ResultCode::Success, status);
                return;
            case AcknowledgementMatch::RemoteRejected:
                complete(ResultCode::RemoteRejected, status);
                return;
            default:
                continue;
            }
        }

        if (now_us >= deadline_us_)
        {
            retry_or_complete(ResultCode::AcknowledgementTimeout);
        }
    }

    void ReliableCommandSender::retry_or_complete(
        ResultCode exhausted_result,
        esp_err_t send_error)
    {
        if (attempts_ <= config_.maximum_retries)
        {
            state_ = State::RetryPending;
            return;
        }

        complete(
            exhausted_result,
            remote_protocol::AcknowledgementStatus::InvalidMessage,
            send_error);
    }

    void ReliableCommandSender::complete(
        ResultCode code,
        remote_protocol::AcknowledgementStatus status,
        esp_err_t send_error)
    {
        result_.code = code;
        result_.acknowledgement_status = status;
        result_.sequence_number = message_.sequence_number;
        result_.message_id = message_.message_id;
        result_.attempts = attempts_;
        result_.send_error = send_error;
        result_ready_ = true;
        state_ = State::Idle;
    }

    bool ReliableCommandSender::is_expected_destination(
        const uint8_t mac[esp_now_driver::MAC_ADDRESS_SIZE]) const
    {
        return mac != nullptr &&
               std::memcmp(
                   mac,
                   config_.destination_mac.data(),
                   esp_now_driver::MAC_ADDRESS_SIZE) == 0;
    }

    bool ReliableCommandSender::is_local_destination(
        const uint8_t mac[esp_now_driver::MAC_ADDRESS_SIZE]) const
    {
        if (mac == nullptr)
        {
            return false;
        }

        bool all_zero = true;
        for (std::size_t index = 0;
             index < esp_now_driver::MAC_ADDRESS_SIZE;
             ++index)
        {
            all_zero = all_zero && mac[index] == 0;
        }

        return all_zero ||
               std::memcmp(
                   mac,
                   config_.local_mac.data(),
                   esp_now_driver::MAC_ADDRESS_SIZE) == 0;
    }

    const char *to_string(ResultCode code)
    {
        switch (code)
        {
        case ResultCode::Success:
            return "Success";
        case ResultCode::EncodeFailed:
            return "EncodeFailed";
        case ResultCode::SendStartFailed:
            return "SendStartFailed";
        case ResultCode::DeliveryFailed:
            return "DeliveryFailed";
        case ResultCode::DeliveryTimeout:
            return "DeliveryTimeout";
        case ResultCode::AcknowledgementTimeout:
            return "AcknowledgementTimeout";
        case ResultCode::InvalidAcknowledgement:
            return "InvalidAcknowledgement";
        case ResultCode::RemoteRejected:
            return "RemoteRejected";
        default:
            return "Unknown";
        }
    }
}
