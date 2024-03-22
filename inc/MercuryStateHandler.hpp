#pragma once

#include "VSLBSP.hpp"

// Mercury includes
#include "system/systemlib/inc/ecmstates.hpp"

#include <memory>

namespace mercury
{
    namespace sys = embedded::system;

    namespace blackstar
    {
        class MercuryStateHandler final
        {
        public:
            MercuryStateHandler() = default;
            ~MercuryStateHandler() = default;

            // Build the object
            void build(std::shared_ptr<VSLBSP> BSP);

            // Functions to be called by CAN message handler
            void startCommandReceived();
            void startJammingCommandReceived();
            void stopJammingCommandReceived();
            void zeroiseCommandReceived();

            // Functions to be called by BlackStar application handler
            void applicationLoaded();

            // Get composite state ("WithError" applied if health is not OK)
            sys::EcmState::State currentState() const;
            bool healthOK() const;

        private:
            // Note - we only use the states witout "WithError" at the end of them
            // we track the health status separately and return the composite state when
            // currentState() is called
            sys::EcmState::State m_state { sys::EcmState::Started };
            bool m_healthOK { true };
            std::shared_ptr<VSLBSP> m_BSP { nullptr };
        };
    }
}
