#include "VSLBSP.hpp"
#include "BuildID.hpp"
#include "CANClient.hpp"
#include "CANMessageHandler.hpp"
#include "MercuryStateHandler.hpp"

#include <cstdlib>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <unistd.h>

namespace bs = mercury::blackstar;

const std::string k_CANDevice { "can0" };

bool keepRunning { true };

void signalHandler(int signalNumber)
{
    keepRunning = false;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);
    std::shared_ptr<bs::VSLBSP> BSP = std::make_shared<bs::VSLBSP>();

    if (BSP->initialise())
    {
        bool runMainLoop { false };

        // Check for command line switches - these cause this application to perform a single
        // action and then exit, if there is no recognised switch then main loop runs until
        // kill signal is received
        uint32_t forward_uV { 0u };
        uint32_t reverse_uV { 0u };
        bool ok { true };
        bool outputDone { false };
        switch (getopt(argc, argv, "dEediMmrs"))
        {
            case 'i':
                // 'i' option - initialise fan/PSU controller
                ok = BSP->resetFanPSUController();
                break;

            case 'r':
                // 'r' option - just get RF power readings and print to standard output
                ok = BSP->getRFPowerMonitorReadings(forward_uV, reverse_uV);
                if (ok)
                {
                    std::cout << forward_uV << "," << reverse_uV << std::endl;
                    outputDone = true;
                }
                break;

            case 'E':
                // 'E' option - enable PSU & fans
                ok = BSP->enableFans() && BSP->enablePSU() && BSP->enablePA();
                BSP->setRFLEDOff();
                break;

            case 'e':
                // 'e' option - disable PSU, fans & PA
                BSP->mutePA();
                ok = BSP->disableFans() && BSP->disablePSU() && BSP->disablePA();
                BSP->setRFLEDOff();
                break;

            case 'M':
                // 'M' option - mute PA
                BSP->mutePA();
                ok = true;
                BSP->setRFLEDOff();
                break;

            case 'm':
                // 'm' option - unmute PA
                BSP->unmutePA();
                ok = true;
                BSP->setRFLEDOn();
                break;

            case 's':
                // 's' option - report ECM slot number
                std::cout << "ECM slot number: " << +BSP->ECMSlotNumber() << std::endl;
                outputDone = true;
                break;

            default:
                runMainLoop = true;
                break;
        }

        if (!runMainLoop)
        {
            // If not running main loop then just output command success
            if (!outputDone)
            {
                if (ok)
                {
                    std::cout << "OK" << std::endl;
                }
                else
                {
                    std::cout << "ERROR" << std::endl;
                }
            }
        }
        else
        {
            // Run main loop
            // clang-format off
            std::cout << "BlackStarECM" << std::endl << "Build ID: 0x"
                      << std::setfill('0') << std::setw(8) << std::right << std::hex
                      << bs::k_buildID << std::endl;
            // clang-format on
            if (BSP->resetFanPSUController())
            {
                // Initialise hardware to safe values
                BSP->disablePA();
                BSP->setRFLEDOff();
                BSP->disablePSU();
                BSP->disableFans();
            }
            else
            {
                std::cout << "ERROR: could not initialise fan/PSU controller" << std::endl;
            }

            // Make the CAN message handler, CAN client and the Mercury state handler
            std::shared_ptr<bs::CANMessageHandler> messageHandler { std::make_shared<bs::CANMessageHandler>() };
            std::shared_ptr<bs::MercuryStateHandler> stateHandler { std::make_shared<bs::MercuryStateHandler>() };
            bs::CANClient client;

            stateHandler->build(BSP);
            messageHandler->build(BSP->ECMSlotNumber(), stateHandler);
            client.build(k_CANDevice, messageHandler);

            // Run the CAN client
            client.run();

            // Keep going until kill signal is received
            while (keepRunning)
            {
            }

            // Stop the CAN client
            client.stop();
        }
    }
    else
    {
        std::cout << "ERROR: Could not open VL_API" << std::endl;
        std::cout << "Note: root privileges (sudo) required to open I2C bus" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
