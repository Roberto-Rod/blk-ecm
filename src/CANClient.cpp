#include "CANClient.hpp"

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <iostream>

namespace mercury::blackstar
{
    void CANClient::build(const std::string& canDevice, std::shared_ptr<CANMessageHandler> messageHandler)
    {
        m_CANDevice = canDevice;
        m_messageHandler = messageHandler;
    }

    void CANClient::run()
    {
        // clang-format off
        m_CANClientThread = std::thread { [&] ()
                                          {
                                              start();
                                          }
                                        };
        // clang-format on
    }

    void CANClient::stop()
    {
        m_stopRequested = true;
        m_CANClientThread.join();
    }

    void CANClient::start()
    {
        // Attempt to connect to the CAN device
        if (connect())
        {
            // Keep looping until the thread is requested to stop
            while (!m_stopRequested)
            {
                // Read data from socket if it is still connected
                if (m_connected)
                {
                    int numberBytesRead { 0 };
                    ::can_frame recvFrame;

                    numberBytesRead = ::read(m_sockfd, &recvFrame, sizeof(recvFrame));
                    if (numberBytesRead == 0)
                    {
                        // Read returns 0 to indicate that the other side has disconnected,
                        // it returns -1 for no data when still connected
                        std::cout << "Socket disconnected";
                        break;
                    }
                    else if ((numberBytesRead > 0) && (m_messageHandler != nullptr))
                    {
                        // Process frame returns true if there is a response to send
                        std::vector<uint8_t> response;
                        if (m_messageHandler->processFrame(recvFrame, response))
                        {
                            sendMessage(response);
                        }
                    }
                }
            };
            disconnect();
        }
        std::cout << "CAN client terminating" << std::endl;
    }

    bool CANClient::connect()
    {
        // Is there is an existing connection then disconnect before attempting to connect again
        disconnect();

        // Create a socket
        if ((m_sockfd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)) >= 0)
        {
            ::ifreq request;
            ::sockaddr_can address;

            ::strcpy(request.ifr_name, m_CANDevice.c_str());
            ::ioctl(m_sockfd, SIOCGIFINDEX, &request);

            ::memset(&address, 0, sizeof(address));
            address.can_family = AF_CAN;
            address.can_ifindex = request.ifr_ifindex;

            // Attempt to bind the socket
            if (::bind(m_sockfd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) >= 0)
            {
                // If connect returned >= 0 then the connection was successful
                m_connected = true;
                std::cout << "Connected CAN socket using device " << m_CANDevice << std::endl;
            }
            else
            {
                std::cout << "ERROR: could not bind CAN socket" << std::endl;
            }
        }
        else
        {
            std::cout << "ERROR: could not create CAN socket" << std::endl;
        }

        return m_connected;
    }

    void CANClient::disconnect()
    {
        if (m_sockfd >= 0)
        {
            close(m_sockfd);
        }
        m_sockfd = -1;
        m_connected = false;
    }

    void CANClient::sendMessage(std::vector<uint8_t>& message)
    {
        if (m_messageHandler != nullptr)
        {
            ::can_frame sendFrame;
            // Mercury message format requires an ECM module to use its own
            // recipient ID as the CAN ID
            sendFrame.can_id = m_messageHandler->recipientID();

            // Reverse the message vector and then pop bytes from the back
            std::reverse(message.begin(), message.end());

            // Keep sending until all of the message bytes have been transmitted
            while (m_connected && !message.empty())
            {
                // Reset data count for next CAN frame
                sendFrame.can_dlc = 0;

                // Send 8 bytes or remaining bytes if there are less than 8
                for (uint8_t byte = 0; (byte < 8u) && !message.empty(); byte++)
                {
                    // Add byte to CAN frame
                    sendFrame.can_dlc++;
                    sendFrame.data[byte] = message.back();
                    message.pop_back();
                }

                // Send CAN frame
                if (::write(m_sockfd, &sendFrame, sizeof(sendFrame)) != sizeof(sendFrame))
                {
                    m_connected = false;
                }
            }
        }
    }
}
