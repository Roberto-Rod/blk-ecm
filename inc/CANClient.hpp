#pragma once

#include "CANMessageHandler.hpp"

#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>

namespace mercury
{
    namespace blackstar
    {
        class CANClient final
        {
        public:
            CANClient() = default;
            ~CANClient() = default;

            void build(const std::string& CANDevice, std::shared_ptr<CANMessageHandler> messageHandler);
            void run();
            void stop();

        private:
            // clang-format off
            //static constexpr std::chrono::duration k_messageTimeout { 1s };        // One second timeout waiting between message packets
            // clang-format on

            void start();
            bool connect();
            void disconnect();
            void sendMessage(std::vector<uint8_t>& message);

            int m_sockfd { -1 };
            std::thread m_CANClientThread;
            std::string m_CANDevice;
            std::shared_ptr<CANMessageHandler> m_messageHandler { nullptr };
            bool m_connected { false };
            std::atomic_bool m_stopRequested { false };
        };
    }
}
