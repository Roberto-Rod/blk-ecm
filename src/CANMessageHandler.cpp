#include "CANMessageHandler.hpp"

// Mercury includes
#include "system/systemlib/inc/commands.hpp"
#include "system/systemlib/inc/ecmstates.hpp"
#include "system/systemlib/inc/version.hpp"

#include <iostream>
#include <boost/crc.hpp>

namespace mercury
{
    namespace sys = embedded::system;

    namespace blackstar
    {
        void CANMessageHandler::build(uint8_t slotNumber, std::shared_ptr<MercuryStateHandler> stateHandler)
        {
            m_recipientID = k_ECMRecipientIDBase + slotNumber;
            m_stateHandler = stateHandler;

            std::cout << "CAN message handler using module ID 0x" << std::hex << +m_recipientID << std::endl;
        }

        bool CANMessageHandler::processFrame(const can_frame& frame, std::vector<uint8_t>& response)
        {
            // If we only wanted to keep frames with CAN ID equal to our recipient ID or broadcast
            // ID then we'd do this:
            // if (frame.can_id == m_recipientID || frame.can_id == k_broadcastRecipientID)
            // But keep frames with CAN ID equal to any ECM slot...
            // Slot 1 ECM = 0xA, Slot 5 ECM = 0xE
            if ((frame.can_id >= 0xA) && (frame.can_id <= 0xE))
            {
                for (uint8_t byte = 0; byte < frame.can_dlc; byte++)
                {
                    m_receiveMessage.push_back(frame.data[byte]);
                }
            }

            return processMessage(response);
        }

        uint8_t CANMessageHandler::recipientID()
        {
            return m_recipientID;
        }

        bool CANMessageHandler::completeMessageReceived()
        {
            bool retVal { false };

            // Inspect length field
            if (m_receiveMessage.size() > k_lengthField)
            {
                uint8_t length { m_receiveMessage.at(k_lengthField) };

                if (m_receiveMessage.size() >= static_cast<size_t>(length + k_baseMessageSize))
                {
                    retVal = true;
                }
            }

            return retVal;
        }

        bool CANMessageHandler::messageAddressedToThisNode()
        {
            bool retVal { false };

            // Message is addressed to this node if the recipient ID matches or if it is broadcast ID
            if (m_receiveMessage.size() > k_recipientField)
            {
                uint8_t recipient { m_receiveMessage.at(k_recipientField) };
                if ((recipient == m_recipientID) || (recipient == k_broadcastRecipientID))
                {
                    retVal = true;
                }
            }
            return retVal;
        }

        bool CANMessageHandler::messageIsCommand()
        {
            bool retVal { false };

            // Message is addressed to this node if the recipient ID matches or if it is broadcast ID
            if (m_receiveMessage.size() > k_typeField)
            {
                uint8_t type { m_receiveMessage.at(k_typeField) };
                if (type == k_messageTypeCommand)
                {
                    retVal = true;
                }
            }

            return retVal;
        }

        bool CANMessageHandler::messageCRCOK()
        {
            bool retVal { false };

            // CRC is in the last two bytes of the message, they are written in order
            // LSB, MSB. Reverse this order and then calculate the overall CRC including the
            // CRC, this will be 0x0000 if the CRC is correct
            if (m_receiveMessage.size() >= k_baseMessageSize)
            {
                uint8_t CRCMSB { m_receiveMessage.back() };
                m_receiveMessage.pop_back();
                uint8_t CRCLSB { m_receiveMessage.back() };
                m_receiveMessage.pop_back();
                m_receiveMessage.push_back(CRCMSB);
                m_receiveMessage.push_back(CRCLSB);
                if (calculateCRC(m_receiveMessage) == 0)
                {
                    retVal = true;
                }
            }

            return retVal;
        }

        uint8_t CANMessageHandler::messageRecipientID()
        {
            uint8_t recipientID { 0u };

            // Message is addressed to this node if the recipient ID matches or if it is broadcast ID
            if (m_receiveMessage.size() > k_recipientField)
            {
                recipientID = m_receiveMessage.at(k_recipientField);
            }
            return recipientID;
        }

        uint16_t CANMessageHandler::messageCommandID()
        {
            uint16_t retVal { 0u };

            if (m_receiveMessage.size() >= k_commandIDMSBField)
            {
                retVal |= static_cast<uint16_t>(m_receiveMessage.at(k_commandIDMSBField)) << 8;
                retVal |= static_cast<uint16_t>(m_receiveMessage.at(k_commandIDLSBField));
            }

            return retVal;
        }

        void CANMessageHandler::populateResponse(uint16_t responseID,
                                                 const std::vector<uint8_t>& parameters,
                                                 std::vector<uint8_t>& response)
        {
            response.clear();

            uint8_t messageType { k_messageTypeCommand };
            uint8_t recipientID { k_MCMRecipientID };
            // Get the command ID we are responding to
            uint16_t commandID { messageCommandID() };

            // Calculate the message length as encoded into the message
            uint8_t messageLength { k_emptyResponseEncodedSize };
            messageLength += parameters.size();
            response.push_back(messageType);
            response.push_back(messageLength);
            response.push_back(recipientID);
            response.push_back(static_cast<uint8_t>(responseID & 0xFF));
            response.push_back(static_cast<uint8_t>(responseID >> 8));
            response.push_back(static_cast<uint8_t>(commandID & 0xFF));
            response.push_back(static_cast<uint8_t>(commandID >> 8));
            // Append parameters to the response message
            response.insert(response.end(), parameters.begin(), parameters.end());
            // Finally, append the CRC to the response message
            uint16_t CRC { calculateCRC(response) };
            response.push_back(static_cast<uint8_t>(CRC & 0xFF));
            response.push_back(static_cast<uint8_t>(CRC >> 8));
        }

        uint16_t CANMessageHandler::calculateCRC(const std::vector<uint8_t>& message)
        {
            boost::crc_ccitt_type result;
            uint8_t bytes[message.size()];
            std::copy(message.begin(), message.end(), bytes);
            result.process_bytes(bytes, message.size());
            return result.checksum();
        }

        // Return true if there is a response to send
        bool CANMessageHandler::processMessage(std::vector<uint8_t>& response)
        {
            bool sendResponse { false };

            if (completeMessageReceived())
            {
                if (messageCRCOK())
                {
                    // If we wanted to make sure we are only processing messages addressed to this node
                    // then we'd do this test:
                    if (messageIsCommand())
                    {
                        uint16_t commandID { messageCommandID() };
                        uint16_t responseID { sys::Command::NotRecognised };
                        uint8_t recipientID { messageRecipientID() };
                        std::cout << "Received message, Recipient ID 0x" << +recipientID << ", Command ID 0x"
                                  << +commandID << std::endl;
                        std::vector<uint8_t> parameters;

                        // If message is addressed to this node then send a response...
                        // Commented out - not sending responses, just sniffing messages
                        // sendResponse = messageAddressedToThisNode();

                        if (commandID == sys::Command::Ping)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetState)
                        {
                            responseID = sys::Command::Ok;

                            // Return 2 byte state
                            uint16_t state { sys::EcmState::Unknown };

                            if (m_stateHandler != nullptr)
                            {
                                state = m_stateHandler->currentState();
                            }

                            parameters.push_back(static_cast<uint8_t>(state & 0xFF));
                            parameters.push_back(static_cast<uint8_t>(state >> 8));
                        }
                        else if (commandID == sys::Command::GetEcmModuleCapabilities)
                        {
                            // Return 64-bit min. freq Hz, 64-bit max. freq Hz
                            responseID = sys::Command::Ok;
                            // Values represent MB module
                            // clang-format off
                            uint64_t minFreqHz {  500000000ull };
                            uint64_t maxFreqHz { 2700000000ull };
                            // clang-format on
                            for (int i = 0; i < 8; i++)
                            {
                                parameters.push_back(static_cast<uint8_t>(minFreqHz & 0xFF));
                                minFreqHz >>= 8;
                            }
                            for (int i = 0; i < 8; i++)
                            {
                                parameters.push_back(static_cast<uint8_t>(maxFreqHz & 0xFF));
                                maxFreqHz >>= 8;
                            }
                        }
                        else if (commandID == sys::Command::GetAlarmThresholds)
                        {
                            responseID = sys::Command::Ok;

                            // TODO: extract values from ECM module
                        }
                        else if (commandID == sys::Command::Reboot)
                        {
                            responseID = sys::Command::Ok;
                            ::system("reboot");
                        }
                        else if (commandID == sys::Command::Identify)
                        {
                            // Don't do anything
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::StartJamming)
                        {
                            // Start Jamming
                            responseID = sys::Command::Ok;

                            if (m_stateHandler != nullptr)
                            {
                                m_stateHandler->startJammingCommandReceived();
                            }
                        }
                        else if (commandID == sys::Command::StopJamming)
                        {
                            // Stop Jamming
                            responseID = sys::Command::Ok;

                            if (m_stateHandler != nullptr)
                            {
                                m_stateHandler->stopJammingCommandReceived();
                            }
                        }
                        else if (commandID == sys::Command::Zeroize)
                        {
                            responseID = sys::Command::Ok;

                            if (m_stateHandler != nullptr)
                            {
                                m_stateHandler->zeroiseCommandReceived();
                            }
                        }
                        else if (commandID == sys::Command::BitTopLevel)
                        {
                            responseID = sys::Command::Ok;
                            // 1 byte response: 0 = OK, other values = not OK
                            if (m_stateHandler->healthOK())
                            {
                                parameters.push_back(0x00);
                            }
                            else
                            {
                                parameters.push_back(0xFF);
                            }
                        }
                        else if (commandID == sys::Command::DetailedBit)
                        {
                            responseID = sys::Command::Ok;

                            // TODO: extract values from ECM module
                        }
                        else if (commandID == sys::Command::UploadMission)
                        {
                            // Don't do anything
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::VerifyMissionFileCrc)
                        {
                            // Don't do anything
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetSerialNumber)
                        {
                            responseID = sys::Command::Ok;
                            // TODO: return serial number
                            parameters.push_back(0x00);
                            parameters.push_back(0x00);
                            parameters.push_back(0x00);
                            parameters.push_back(0x00);
                        }
                        else if (commandID == sys::Command::SetSerialNumber)
                        {
                            responseID = sys::Command::Ok;
                            // TODO: set serial number
                        }
                        else if (commandID == sys::Command::GetSoftwareVersionNumber)
                        {
                            // Get Software Version Number
                            responseID = sys::Command::Ok;

                            // Return 2 bytes each: field:
                            // SW major, SW minor, SW patch
                            // BL major, BL minor, BL patch

                            // Use the Mercury submodule version which will represent the
                            // command and state IDs we are using
                            // Set BL = 0.0.0
                            uint16_t major { sys::getMajorVersion() };
                            uint16_t minor { sys::getMinorVersion() };
                            uint16_t patch { sys::getBuildNumber() };
                            parameters.push_back(static_cast<uint8_t>(major & 0xFF));
                            parameters.push_back(static_cast<uint8_t>(major >> 8));
                            parameters.push_back(static_cast<uint8_t>(minor & 0xFF));
                            parameters.push_back(static_cast<uint8_t>(minor >> 8));
                            parameters.push_back(static_cast<uint8_t>(patch & 0xFF));
                            parameters.push_back(static_cast<uint8_t>(patch >> 8));
                            parameters.push_back(0x00);  // BL major LSB
                            parameters.push_back(0x00);  // BL major MSB
                            parameters.push_back(0x00);  // BL minor LSB
                            parameters.push_back(0x00);  // BL minor MSB
                            parameters.push_back(0x00);  // BL patch LSB
                            parameters.push_back(0x00);  // BL patch MSB
                        }
                        else if (commandID == sys::Command::GetFpgaVersionNumber)
                        {
                            responseID = sys::Command::Ok;

                            // Get FPGA Version Number, format as per SW version above
                            // Set FPGA version to 0.0.0 as it is N/A in BlackStar
                            // ECM sets BL version to 0.0.0 in this as it is always N/A
                            parameters.push_back(0x00);  // SW major LSB
                            parameters.push_back(0x00);  // SW major MSB
                            parameters.push_back(0x00);  // SW minor LSB
                            parameters.push_back(0x00);  // SW minor MSB
                            parameters.push_back(0x00);  // SW patch LSB
                            parameters.push_back(0x00);  // SW patch MSB
                            parameters.push_back(0x00);  // BL major LSB
                            parameters.push_back(0x00);  // BL major MSB
                            parameters.push_back(0x00);  // BL minor LSB
                            parameters.push_back(0x00);  // BL minor MSB
                            parameters.push_back(0x00);  // BL patch LSB
                            parameters.push_back(0x00);  // BL patch MSB
                        }
                        else if (commandID == sys::Command::SetTimeAndDate)
                        {
                            responseID = sys::Command::Ok;
                            // TODO: set time and date
                        }
                        else if (commandID == sys::Command::GetTimeAndDate)
                        {
                            responseID = sys::Command::Ok;
                            // TODO: return time and date
                            parameters.push_back(0x00);  // Year LSB
                            parameters.push_back(0x00);  // Year MSB
                            parameters.push_back(0x00);  // Month
                            parameters.push_back(0x00);  // Day of Month
                            parameters.push_back(0x00);  // Day of Week
                            parameters.push_back(0x00);  // Hour
                            parameters.push_back(0x00);  // Minutes
                            parameters.push_back(0x00);  // Seconds
                        }
                        else if (commandID == sys::Command::GetSoftwarePartNumber)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetReturnLoss)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetVswrThresholds)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetVswrThresholds)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetNumberOfVswrThresholds)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetNumberOfErrorLogEntries)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetErrorLogEntry)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::ClearErrorLog)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::UploadEcmMissionHeader)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::UploadEcmXchange)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::UploadEcmMissionFileLine)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::UploadEcmVswr)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetMissionFileInstallProgress)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::UploadFile)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::VerifyFileCrc)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::ResetToDefault)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetTamperDetectionStatus)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetTamperDetectionStatus)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetEcmDominance)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetEcmDominance)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetCalibrationType)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetCalibrationType)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetPowerMonitorRevision)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetPowerMonitorRevision)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::BatteryAggregateStatus)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetEcmAlarmThresholds)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetDocked)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetEcmDockFpgaVersionNumber)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::IsDocked)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetBitEnable)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EnableVSWRCalibrationMode)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EnablePaCalibrationMode)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetPaCalibrationPoint)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetLoggingLevel)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetLoggingLevel)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::Start)
                        {
                            responseID = sys::Command::Ok;

                            if (m_stateHandler != nullptr)
                            {
                                m_stateHandler->startCommandReceived();
                            }
                        }
                        else if (commandID == sys::Command::GetFanPsuFirmwareVersion)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmPairWithDock)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmUnpairWithDock)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDockPairing)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetPowerMonitorReading)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetSynthFrequency)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetSynthFrequency)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetNumberOfPaCalibrationPoints)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmCheckPaCalibrationPoint)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetBenchTest)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetBenchTest)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetTemperatureCompensation)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetTemperatureCompensation)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetSourcePortMode)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetEnhancedTiming)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetCvswrThreshold)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetCvswrThreshold)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrm)
                        {
                            responseID = sys::Command::Ok;

                            // Standard non-responsive ECM response:
                            parameters.push_back(0x00);
                            parameters.push_back(0x01);
                        }
                        else if (commandID == sys::Command::EcmSetDrm)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmShutdown)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrmBitStatus)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrmDetailedBit)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetLoggerDescriptor)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetLoggerLevel)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetLoggerLevel)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrmVersions)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrmSerialNumber)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetDrmSerialNumber)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrmMacAddress)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetDrmIpAssignment)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSetDrmIpAssignment)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmSendData)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetMissionValid)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::SetSerialNumberExt)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::GetSerialNumberExt)
                        {
                            responseID = sys::Command::Ok;
                        }
                        else if (commandID == sys::Command::EcmGetProductId)
                        {
                            responseID = sys::Command::Ok;

                            // Product type - SkyNet
                            parameters.push_back(0x02);
                            parameters.push_back(0x00);
                        }

                        populateResponse(responseID, parameters, response);
                    }
                }
                else
                {
                    std::cout << "ERROR: message failed CRC check" << std::endl;
                }

                // Clear the message vector after processing a complete message
                m_receiveMessage.clear();
            }

            return sendResponse;
        }
    }
}
