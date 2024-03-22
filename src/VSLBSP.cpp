#include "VSLBSP.hpp"

#include <unistd.h>
#include <iostream>
#include <cstdbool>  // Include this before VL_OSALib.h as that uses _Bool

#define linux
#include "../lib/VL_OSALib.h"

namespace mercury::blackstar
{
    // clang-format off
    // LED DIO definitions
    // Ref.: VersaAPIGuide, Table 21 "EPU Board On-board DIO Channels"
    static const uint8_t k_ECMSlot0Channel  { DIO_CHANNEL_17 }; // GPIO1
    static const uint8_t k_ECMSlot1Channel  { DIO_CHANNEL_18 }; // GPIO2
    static const uint8_t k_ECMSlot2Channel  { DIO_CHANNEL_19 }; // GPIO3
    static const uint8_t k_PAMuteNChannel   { DIO_CHANNEL_20 }; // GPIO4
    static const uint8_t k_PPSSelectChannel { DIO_CHANNEL_21 }; // GPIO5
    static const uint8_t k_RFLEDChannel     { DIO_CHANNEL_22 }; // GPIO6
    static const uint8_t k_GNSSResetChannel { DIO_CHANNEL_23 }; // GPIO7
    static const uint8_t k_AlertLEDChannel  { DIO_CHANNEL_24 }; // GPIO8
    
    // Fan/PSU controller I2C definitions
    static const uint8_t k_fanPSUI2CChipAddress         { 0x15 };
    static const uint8_t k_fanPSUI2CControlRegister     { 0x00 };
    static const uint8_t k_fanPSUI2CFan1Register        { 0x01 };
    static const uint8_t k_fanPSUI2CFan2Register        { 0x02 };
    static const uint8_t k_fanPSUI2CStatus1Register     { 0x40 };
    static const uint8_t k_fanPSUI2CResetRegister       { 0x7F };
    static const uint8_t k_fanPSUI2CControlBitPSUEnable { 0x01 };
    static const uint8_t k_fanPSUI2CControlBitPAEnable  { 0x04 };
    static const uint8_t k_fanPSUI2CFanBitsFanEnable    { 0x13 };
    static const uint8_t k_fanPSUI2CResetCode           { 0x34 };
    static const useconds_t k_fanPSUResetSleepTime_us   { 100000u };
    // clang-format on

    VSLBSP::VSLBSP()
    {
    }

    VSLBSP::~VSLBSP()
    {
        VL_Close();
    }
    
    bool VSLBSP::initialise()
    {
        m_APIOpen = false;
        static const int32_t k_VLReturnSuccess { 0 };
        static const uint32_t k_SPIMode { 3u };
        static const uint32_t k_SPIBytesPerFrame { 2u };

        // Open the VersaLogic API and configure settings for this BSP
        if (VL_Open() == k_VLReturnSuccess)
        {
            bool ok { true };

            // clang-format off
            // Set outputs to default values before enabling outputs
            //VL_DIOSetChannelLevel(k_PAMuteNChannel,   DIO_CHANNEL_LOW); // Low = PA muted
            VL_DIOSetChannelLevel(k_PPSSelectChannel, DIO_CHANNEL_LOW); // Low = Internal GNSS receiver
            //VL_DIOSetChannelLevel(k_RFLEDChannel,     DIO_CHANNEL_LOW); // Low = RF LED off
            VL_DIOSetChannelLevel(k_GNSSResetChannel, DIO_CHANNEL_LOW); // Low = GNSS receiver out of reset
            VL_DIOSetChannelLevel(k_AlertLEDChannel,  DIO_CHANNEL_LOW); // Low = Alert LED off
            
            // Set the digital I/O directions
            VL_DIOSetChannelDirection(k_ECMSlot0Channel,  DIO_INPUT);
            VL_DIOSetChannelDirection(k_ECMSlot1Channel,  DIO_INPUT);
            VL_DIOSetChannelDirection(k_ECMSlot2Channel,  DIO_INPUT);
            VL_DIOSetChannelDirection(k_PAMuteNChannel,   DIO_OUTPUT);
            VL_DIOSetChannelDirection(k_PPSSelectChannel, DIO_OUTPUT);
            VL_DIOSetChannelDirection(k_RFLEDChannel,     DIO_OUTPUT);
            VL_DIOSetChannelDirection(k_GNSSResetChannel, DIO_OUTPUT);
            VL_DIOSetChannelDirection(k_AlertLEDChannel,  DIO_OUTPUT);
            // clang-format on

            // Configure the I2C bus to communicate with the fan/PSU controller
            ok = ok && (VSL_I2CIsAvailable(VL_I2C_BUS_TYPE_PRIMARY) == VL_API_OK);
            ok = ok && (VSL_I2CSetFrequency(VL_I2C_BUS_TYPE_PRIMARY, VL_I2C_FREQUENCY_100KHZ) == VL_API_OK);

            // Configure the SPI bus to read from the RF Power Monitor
            // ADC122S051 SPI parameters:
            //   Transaction length: 16 bits
            //   CPHA: 1, CPOL: 1 (SPI Mode 3)
            //   SCLK: 3.2 MHz to  8.0 MHz
            ok = ok && (VSL_SPIIsAvailable() == VL_API_OK);
            // VersaAPI SPI clock frequency options:
            // SPI_CLK_FREQ0 = 0.75 MHz
            // SPI_CLK_FREQ1 = 1.5 MHz
            // SPI_CLK_FREQ2 = 2.0 MHz
            // SPI_CLK_FREQ3 = 6.0 MHz <-- selected (compatible with ADC122S051)
            ok = ok && (VSL_SPISetFrequency(SPI_CLK_FREQ3) == VL_API_OK);
            ok = ok && (VSL_SPISetShiftDirection(SPI_DIR_LEFT) == VL_API_OK);
            ok = ok && (VSL_SPISetFrameSize(k_SPIBytesPerFrame) == VL_API_OK);
            ok = ok && (VSL_SPISetMode(k_SPIMode) == VL_API_OK);

            if (ok)
            {
                m_APIOpen = true;
            }
            else
            {
                VL_Close();
            }
        }
        
        return m_APIOpen;
    }
    
    void VSLBSP::setRFLEDOn()
    {
        if (m_APIOpen)
        {
            // LED control outputs are active (LED on) high
            VL_DIOSetChannelLevel(k_RFLEDChannel, DIO_CHANNEL_HIGH);
        }
    }
    
    void VSLBSP::setRFLEDOff()
    {
        if (m_APIOpen)
        {
            // LED control outputs are active (LED on) high
            VL_DIOSetChannelLevel(k_RFLEDChannel, DIO_CHANNEL_LOW);
        }
    }
    
    void VSLBSP::setAlertLEDOn()
    {
        if (m_APIOpen)
        {
            // LED control outputs are active (LED on) high
            VL_DIOSetChannelLevel(k_AlertLEDChannel, DIO_CHANNEL_HIGH);
        }
    }
    
    void VSLBSP::setAlertLEDOff()    
    {
        if (m_APIOpen)
        {
            // LED control outputs are active (LED on) high
            VL_DIOSetChannelLevel(k_AlertLEDChannel, DIO_CHANNEL_LOW);
        }
    }

    void VSLBSP::mutePA()
    {
        if (m_APIOpen)
        {
            VL_DIOSetChannelLevel(k_PAMuteNChannel, DIO_CHANNEL_LOW);
        }
    }

    void VSLBSP::unmutePA()
    {
        if (m_APIOpen)
        {
            VL_DIOSetChannelLevel(k_PAMuteNChannel, DIO_CHANNEL_HIGH);
        }
    }

    bool VSLBSP::resetFanPSUController()
    {
        bool ok { false };
        if (writeFanPSURegister(k_fanPSUI2CResetRegister, k_fanPSUI2CResetCode))
        {
            ok = true;
            usleep(k_fanPSUResetSleepTime_us);
        }
        return ok;
    }
    
    bool VSLBSP::enableFans()
    {
        bool ok { true };
        ok = setFanPSURegisterBits(k_fanPSUI2CFan1Register, k_fanPSUI2CFanBitsFanEnable) && ok;
        ok = setFanPSURegisterBits(k_fanPSUI2CFan1Register, k_fanPSUI2CFanBitsFanEnable) && ok;
        return ok;
    }
    
    bool VSLBSP::disableFans()
    {
        bool ok { true };
        ok = clearFanPSURegisterBits(k_fanPSUI2CFan1Register, k_fanPSUI2CFanBitsFanEnable) && ok;
        ok = clearFanPSURegisterBits(k_fanPSUI2CFan1Register, k_fanPSUI2CFanBitsFanEnable) && ok;
        return ok;
    }
    
    bool VSLBSP::enablePSU()
    {
        return setFanPSURegisterBits(k_fanPSUI2CControlRegister, k_fanPSUI2CControlBitPSUEnable);
    }
    
    bool VSLBSP::disablePSU()
    {
        return clearFanPSURegisterBits(k_fanPSUI2CControlRegister, k_fanPSUI2CControlBitPSUEnable);
    }
    
    bool VSLBSP::enablePA()
    {
        return setFanPSURegisterBits(k_fanPSUI2CControlRegister, k_fanPSUI2CControlBitPAEnable);
    }
    
    bool VSLBSP::disablePA()
    {
        return clearFanPSURegisterBits(k_fanPSUI2CControlRegister, k_fanPSUI2CControlBitPAEnable);
    }

    bool VSLBSP::getRFPowerMonitorReadings(uint32_t& forward_uV, uint32_t& reverse_uV)
    {
        // SPI data is shifted out to the left and the write data is in a uint32_t buffer
        // so shift the 1-byte control word left by 24-bits before initiating transfer
        static const uint32_t k_forwardSensorControlValue { 0x00 << 24 };  // IN1
        static const uint32_t k_reverseSensorControlValue { 0x08 << 24 };  // IN2

        bool ok { false };

        if (m_APIOpen)
        {
            uint32_t forwardADC { 0u };
            uint32_t reverseADC { 0u };
            uint32_t control { 0u };
            // The ADC122S051 ADC returns the track/hold value for the channel selected in
            // the previous SPI cycle. To ensure we don't get out of sync, send 3 transactions.
            // Note that the VSL_SPIReadDataFrame function gets the data which had been returned
            // on the last VSL_SPIWriteDataFrame transaction.
            ok = true;

            // Transaction 1: select forward, discard returned value
            if (ok)
            {
                control = k_forwardSensorControlValue;
                ok = VSL_SPIWriteDataFrame(SPI_SS_SS0, &control);
            }
            // Transaction 2: select reverse, read forward value
            if (ok)
            {
                control = k_reverseSensorControlValue;
                ok = VSL_SPIWriteDataFrame(SPI_SS_SS0, &control) && VSL_SPIReadDataFrame(&forwardADC);
            }
            // Transaction 3: select forward, read reverse value
            // Note: channel selected on third transaction is effectively "don't care",
            // leave reverse selected
            if (ok)
            {
                ok = VSL_SPIWriteDataFrame(SPI_SS_SS0, &control) && VSL_SPIReadDataFrame(&reverseADC);
            }

            // Convert the ADC readings to microvolts
            if (ok)
            {
                // 1 LSB = Va / 4096 = 5000 mV / 4096 = 1.221 mV
                static const uint32_t va_mV { 5000u };
                static const uint32_t bitRange { 4096u };
                static const uint32_t bitRangeDiv2 { bitRange / 2u };
                // Add half the denominator before dividing to round the result
                forward_uV = ((forwardADC * va_mV) + bitRangeDiv2) / bitRange;
                reverse_uV = ((reverseADC * va_mV) + bitRangeDiv2) / bitRange;
            }
        }
        return ok;
    }

    uint8_t VSLBSP::ECMSlotNumber()
    {
        uint8_t slotNumber { 0u };
        // Slot Bits 2:0
        // Slot 0 = "000"
        // Slot 1 = "001"
        // ...
        // Slot 7 = "111"
        if (VSL_DIOGetChannelLevel(k_ECMSlot2Channel) == 0x01)
        {
            slotNumber |= 0x04;
        }
        if (VSL_DIOGetChannelLevel(k_ECMSlot1Channel) == 0x01)
        {
            slotNumber |= 0x02;
        }
        if (VSL_DIOGetChannelLevel(k_ECMSlot0Channel) == 0x01)
        {
            slotNumber |= 0x01;
        }
        return slotNumber;
    }

    bool VSLBSP::readFanPSURegister(uint8_t address, uint8_t& data)
    {
        bool ok { false };
        if (m_APIOpen)
        {
            ok = (VSL_I2CReadRegister(VL_I2C_BUS_TYPE_PRIMARY, k_fanPSUI2CChipAddress, address, &data) == VL_API_OK);
            if (!ok)
            {
                std::cout << "READ FAILED" << std::endl;
            }
        }
        return ok;
    }
 
    bool VSLBSP::writeFanPSURegister(uint8_t address, uint8_t data)
    {
        bool ok { false };
        if (m_APIOpen)
        {
            // Note VersaAPIGuide v1.8.2 has an error in the description of VSL_I2CWriteRegister,
            // it states that the fourth parameter is a pointer to unsigned char but it is an unsigned char (not a pointer)
            ok = (VSL_I2CWriteRegister(VL_I2C_BUS_TYPE_PRIMARY, k_fanPSUI2CChipAddress, address, data) == VL_API_OK);
        }
        return ok;
    }
    
    bool VSLBSP::setFanPSURegisterBits(uint8_t address, uint8_t mask)
    {
        bool ok { false };
        uint8_t value { 0u };
        if (readFanPSURegister(address, value))
        {
            value |= mask;
            ok = writeFanPSURegister(address, value);
        }
        return ok;
    }
    
    bool VSLBSP::clearFanPSURegisterBits(uint8_t address, uint8_t mask)
    {
        bool ok { false };
        uint8_t value { 0u };
        if (readFanPSURegister(address, value))
        {
            value &= ~mask;
            ok = writeFanPSURegister(address, value);
        }
        return ok;
    }
}
