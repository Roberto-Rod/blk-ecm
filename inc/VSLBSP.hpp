#pragma once
#include <cstdint>

namespace mercury::blackstar
{
    class VSLBSP final
    {
    public:
        VSLBSP();
        ~VSLBSP();
        
        // Initialise function opens the VersaLogic API
        bool initialise();
        
        // LED control functions
        void setRFLEDOn();
        void setRFLEDOff();
        void setAlertLEDOn();
        void setAlertLEDOff();

        // PA mute control functions
        void mutePA();
        void unmutePA();

        // Fan/PSU (I2C) control functions
        bool resetFanPSUController();
        bool enableFans();
        bool disableFans();
        bool enablePSU();
        bool disablePSU();
        bool enablePA();
        bool disablePA();

        // RF Power Monitor (SPI) functions
        bool getRFPowerMonitorReadings(uint32_t& forward_uV, uint32_t& reverse_uV);

        // ECM slot number function
        uint8_t ECMSlotNumber();

    private:
        // Fan/PSU register control
        bool readFanPSURegister(uint8_t address, uint8_t& data);
        bool writeFanPSURegister(uint8_t address, uint8_t data);
        bool setFanPSURegisterBits(uint8_t address, uint8_t mask);
        bool clearFanPSURegisterBits(uint8_t address, uint8_t mask);
    
        bool m_APIOpen { false };
    };
}
