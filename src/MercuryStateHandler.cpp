#include "MercuryStateHandler.hpp"

#include <iostream>
#include <thread>
#include <chrono>

namespace mercury
{
    namespace sys = embedded::system;

    namespace blackstar
    {

        void MercuryStateHandler::build(std::shared_ptr<VSLBSP> BSP)
        {
            m_BSP = BSP;
            m_BSP->enablePSU();
            m_BSP->enableFans();
        }

        // Functions to be called by CAN message handler
        void MercuryStateHandler::startCommandReceived()
        {
            // If the state initialised or an earlier state then move to StandbyNoMission
            if (m_state <= sys::EcmState::Initialised)
            {
                m_state = sys::EcmState::StandbyNoMission;
            }
        }

        void MercuryStateHandler::startJammingCommandReceived()
        {
            std::cout << "Start Jamming Command Received" << std::endl;
            m_state = sys::EcmState::Jamming;
            m_BSP->setRFLEDOn();
            m_BSP->enablePA();
            m_BSP->unmutePA();
        }

        void MercuryStateHandler::stopJammingCommandReceived()
        {
            std::cout << "Stop Jamming Command Received" << std::endl;
            m_state = sys::EcmState::StandbyWithMission;
            m_BSP->setRFLEDOff();
            m_BSP->mutePA();
            m_BSP->disablePA();
        }

        void MercuryStateHandler::zeroiseCommandReceived()
        {
            if (sys::EcmState::isStandby(m_state))
            {
                m_state = sys::EcmState::StandbyNoMission;
                // TODO: remove application
            }
        }

        // Functions to be called by BlackStar application handler
        void MercuryStateHandler::applicationLoaded()
        {
            if (sys::EcmState::isStandbyNoMission(m_state))
            {
                m_state = sys::EcmState::StandbyWithMission;
            }
        }

        sys::EcmState::State MercuryStateHandler::currentState() const
        {
            sys::EcmState::State state { m_state };

            // If health is not OK then map the current state to the corresponding "WithError" state
            if (!m_healthOK)
            {
                if (state == sys::EcmState::StandbyNoMission)
                {
                    state = sys::EcmState::StandbyNoMissionWithError;
                }
                else if (state == sys::EcmState::StandbyWithMission)
                {
                    state = sys::EcmState::StandbyWithMissionWithError;
                }
                else if (state == sys::EcmState::Jamming)
                {
                    state = sys::EcmState::JammingWithError;
                }
            }

            return state;
        }

        bool MercuryStateHandler::healthOK() const
        {
            return m_healthOK;
        }
    }
}
