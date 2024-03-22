#pragma once

#include "MercuryStateHandler.hpp"

// Mercury includes
#include "system/systemlib/inc/ecmstates.hpp"

#include <linux/can.h>
#include <memory>
#include <vector>

namespace mercury::blackstar
{
    class CANMessageHandler final
    {
    public:
        CANMessageHandler() = default;
        ~CANMessageHandler() = default;

        void build(uint8_t slotNumber, std::shared_ptr<MercuryStateHandler> stateHandler);

        // Returns true if there is a response to send
        bool processFrame(const can_frame& frame, std::vector<uint8_t>& response);

        // processMessage is a monolithic function written to handle all of the
        // ECM messages so that a BlackStar Module can appear to be a Longbow ECM module
        // Where the messages are useful/meaningful to the BlackStar module they are utilised
        // otherwise they are just responded to in a way which will satisfy the MCM
        // Returns true if there is a response to send
        bool processMessage(std::vector<uint8_t>& response);

        uint8_t recipientID();

    private:
        static const uint8_t k_broadcastRecipientID { 0x00 };
        static const uint8_t k_MCMRecipientID { 0x01 };
        static const uint8_t k_ECMRecipientIDBase { 0x0A };  // ECM slot 0 recipient ID

        // Actual length of message excluding fields counted as part of the encoded message length field
        static const uint8_t k_baseMessageSize { 5u };

        // Length of a response message with no parameters as encoded into the length field
        static const uint8_t k_emptyResponseEncodedSize { 4u };

        static const uint8_t k_typeField { 0u };             // Byte position of message type field
        static const uint8_t k_lengthField { 1u };           // Byte position of length field
        static const uint8_t k_recipientField { 2u };        // Byte position of recipient field
        static const uint8_t k_commandIDLSBField { 3u };     // Byte position of command ID MSB field
        static const uint8_t k_commandIDMSBField { 4u };     // Byte position of command ID MSB field
        static const uint8_t k_parametersStartField { 5u };  // Byte position of first parameters byte

        static const uint8_t k_messageTypeCommand { 0xC0 };  // Message type for a command message

        // Returns true if message in the receive buffer appears to be complete based on the length field
        bool completeMessageReceived();
        bool messageAddressedToThisNode();
        bool messageIsCommand();
        bool messageCRCOK();
        uint8_t messageRecipientID();
        uint16_t messageCommandID();
        void populateResponse(uint16_t responseID,
                              const std::vector<uint8_t>& parameters,
                              std::vector<uint8_t>& response);
        uint16_t calculateCRC(const std::vector<uint8_t>& message);

        uint8_t m_recipientID { 0u };
        std::vector<uint8_t> m_receiveMessage;
        std::shared_ptr<MercuryStateHandler> m_stateHandler { nullptr };
    };
}
