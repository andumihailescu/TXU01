#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "esp_now_driver/esp_now_driver.h"
#include "reliable_command_sender/acknowledgement.h"
#include "remote_protocol/remote_protocol.h"

namespace reliable_command_sender
{
    struct Config
    {
        std::array<uint8_t, esp_now_driver::MAC_ADDRESS_SIZE>
            destination_mac{};
        std::array<uint8_t, esp_now_driver::MAC_ADDRESS_SIZE>
            local_mac{};
        uint32_t send_result_timeout_ms = 200;
        uint32_t acknowledgement_timeout_ms = 200;
        uint8_t maximum_retries = 2;
    };

    enum class ResultCode
    {
        Success,
        EncodeFailed,
        SendStartFailed,
        DeliveryFailed,
        DeliveryTimeout,
        AcknowledgementTimeout,
        InvalidAcknowledgement,
        RemoteRejected
    };

    struct Result
    {
        ResultCode code = ResultCode::SendStartFailed;
        remote_protocol::AcknowledgementStatus acknowledgement_status =
            remote_protocol::AcknowledgementStatus::InvalidMessage;
        uint16_t sequence_number = 0;
        uint16_t message_id = 0;
        uint16_t attempts = 0;
        esp_err_t send_error = ESP_OK;
    };

    class ReliableCommandSender
    {
    public:
        explicit ReliableCommandSender(Config config);

        esp_err_t init();

        bool start(
            uint16_t message_id,
            const uint8_t *payload,
            uint8_t payload_length);

        void process();
        bool busy() const;
        bool take_result(Result &result);

    private:
        enum class State
        {
            Idle,
            RetryPending,
            WaitingDelivery,
            WaitingAcknowledgement
        };

        void start_attempt(int64_t now_us);
        void process_delivery(int64_t now_us);
        void process_acknowledgement(int64_t now_us);
        void retry_or_complete(
            ResultCode exhausted_result,
            esp_err_t send_error = ESP_OK);
        void complete(
            ResultCode code,
            remote_protocol::AcknowledgementStatus status =
                remote_protocol::AcknowledgementStatus::InvalidMessage,
            esp_err_t send_error = ESP_OK);
        bool is_expected_destination(
            const uint8_t mac[esp_now_driver::MAC_ADDRESS_SIZE]) const;
        bool is_local_destination(
            const uint8_t mac[esp_now_driver::MAC_ADDRESS_SIZE]) const;

        Config config_;
        bool initialized_ = false;
        uint16_t next_sequence_ = 0;
        State state_ = State::Idle;
        int64_t deadline_us_ = 0;
        uint16_t attempts_ = 0;

        remote_protocol::Message message_{};
        uint8_t encoded_packet_[remote_protocol::MAX_PACKET_SIZE]{};
        std::size_t encoded_length_ = 0;

        bool result_ready_ = false;
        Result result_{};
    };

    const char *to_string(ResultCode code);
}
