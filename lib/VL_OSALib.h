/******************************************************************************/
/* Name			: VL_OSALib.h												  */
/*                                                                            */
/* Contents		: Function prototypes for Operating System Agnostic calls     */
/*                                                                            */
/* Author		: VersaLogic Corporation					                  */
/*                                                                            */
/* Version		: 1.6.0                                                       */
/*                                                                            */
/* Copyright 2015-2020 VersaLogic Corporation								  */
/******************************************************************************/


#ifndef _VLAPI_H
#define _VLAPI_H

#ifdef WIN32
// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the VL_OSALIB_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// VL_OSALIB_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef VL_OSALIB_EXPORTS
#define VL_OSALIB_API __declspec(dllexport)
#else
#define VL_OSALIB_API __declspec(dllimport)
#endif

#endif

#ifndef FILE_DEVICE_XRPORT

#define FILE_DEVICE_XRPORT 0x00008005
#define XRPORT_IOCTL_INDEX 0x805
#define IOCTL_XRPORT_READ_CONFIG_REG \
CTL_CODE(FILE_DEVICE_XRPORT , \
XRPORT_IOCTL_INDEX + 8, \
METHOD_BUFFERED, \
FILE_ANY_ACCESS)

#define IOCTL_XRPORT_WRITE_CONFIG_REG \
CTL_CODE(FILE_DEVICE_XRPORT , \
XRPORT_IOCTL_INDEX + 9, \
METHOD_BUFFERED, \
FILE_ANY_ACCESS)

typedef struct
{
unsigned char bReg;
unsigned char bData;
} CONFIG_WRITE, *PCONFIG_WRITE;

#endif

#ifdef linux
#define VL_OSALIB_API

#include <sys/types.h>
#include <unistd.h>
#ifdef usb
#include <libusb-1.0/libusb.h>
#endif  // usb
#endif  // linux


#ifdef __cplusplus
extern "C"
{
#endif


/* VersaLogic API return codes. */
typedef enum {
	VL_API_OK			= 1,			// Function completed successfully.
	VL_API_ERROR,						// Unexpected problem occurred in the VersaLogic library.
	VL_API_INVALID_ARG,					// Command argument out of range.
	VL_API_NOT_SUPPORTED,				// Function not supported.
	VL_API_ARG_NOT_SUPPORTED,			// Arguement currently not supported.
	VL_API_BC_LIBRARY_INIT_ERROR,		// Baseboard Controller library initialization error.
	VL_API_BC_LIBRARY_INSTALL_ERROR,	// Baseboard Controller library installation error.
	VL_API_BC_BOARD_OPEN_ERROR,			// Baseboard could not be opened error.
	VL_API_BC_LIBRARY_CLOSE_ERROR,		// Baseboard Controller library close error.
	VL_API_I2C_REQUESTED_BUS_NOT_FOUND,	// Could not find the requested I2C bus.
	VL_API_I2C_READ_ERROR,				// Error attempting to read I2C address.
	VL_API_I2C_READ_REGISTER_ERROR,		// Error attempting to read I2C register.
	VL_API_I2C_WRITE_ERROR,				// Error attempting to write I2C address.
	VL_API_I2C_WRITE_REGISTER_ERROR,	// Error attempting to write I2C register.
	VL_API_I2C_PRIVLEDGE_ERROR,         // Privledge error. Must run with elevated privledges.
	VL_API_I2C_INFRASTRUCTURE_ERROR,    // Infrastructure error. Be sure I2C tools installed. 
    VL_API_GET_UPTIME_ERROR,			// Error attempting to get the board up time.
	VL_API_VGA_BL_VALUE_OUT_OF_RANGE,	// Specified backlight value is out of range.
	VL_API_VGA_BL_INTERFACE_ERROR,		// Backlight generic error.
	VL_API_VGA_BL_DISPLAY_NOT_FOUND,	// Backlight name not found.
	VL_API_IRQ_NUM_UNKNOWN,				// Could not determine current IRQ number.
	VL_API_BOARD_ID_ERROR,				// Could not determine board ID.
	VL_API_INVALID_DIO_CHANNEL,			// Specified DIO Channel was invalid.
	VL_API_SPI_READ_ERROR,				// Error attempting to read SPI data registers.
	VL_API_SPI_WRITE_ERROR,				// Error attempting to write SPI data registers.
	VL_API_CAN_CLOSE_DEVICE_ERROR,		// Cound not close the CAN device.
	VL_API_CAN_OPEN_DEVICE_ERROR, 		// Cound not open the CAN device.
	VL_API_CAN_OPEN_NO_DEVICE_FOUND, 	// Cound not find any USB (CAN) devices.
	VL_API_CAN_OPEN_NO_DEVICE_DESC,	 	// Cound not open the CAN device descriptor.
	VL_API_CAN_OPEN_PORT_ERROR,		 	// Cound not open the CAN device descriptor.
	VL_API_CAN_PORT_CLOSED,				// CAN port is closed.
	VL_API_CAN_TX_ERROR		 			// Failed to transmit CAN packet.
} VL_APIStatusT;

/* Helpful defines. */
#define TRUE 1								// Simple define.
#define FALSE 0								// Simple define.
#define YES 1								// Simple define.
#define NO 0								// Simple define.

/* Administative API calls */
VL_OSALIB_API unsigned long VSL_Open();
VL_OSALIB_API unsigned long VSL_Close();
VL_OSALIB_API void		    VSL_GetVersion(unsigned char *Major, unsigned char *Minor, unsigned char *Release);
VL_OSALIB_API unsigned long VSL_FindProducts();

/* Heat sink fan API calls */
VL_OSALIB_API void				VSL_FanOn();
VL_OSALIB_API void				VSL_FanOff();
VL_OSALIB_API unsigned short	VSL_FanGetRPM(unsigned char PulsePerRevolution);

/* API calls for access I/O control and information functions */
/* Set the operation mode of the ADS8668 to auto scan the selected analog channels.
 * 
 * Inputs:
 *     Enable: NO (default) => Auto scan mode disabled.
 *             YES          => Auto scan mode disabled
 *
 *     EnableOnReset: NO (default) => Auto scan mode disabled on reset.
 *                    YES          => Auto scan mode enabled on reset.
 *
 *     Channels: 
 *         0 (default) => Channel is not include in the auto scan.
 *         1           => Channel is include in the auto scan.
 *         Bit[7]      => Channel 7
 *            :              :
 *         Bit[0]      => Channel 0
 *       
 *
 * Outputs:
 *     None.
 *
 * Return Values:
 *    VL_API_OK            => Successful in enable/disabling auto scan.
 *    VL_API_NOT_SUPPORTED => Auto scan functionality not supported on this hardware.
 *
 * */
VL_OSALIB_API VL_APIStatusT	VSL_ADCConfigureAnalogAutoScanMode(int Enable,              
				                                               int EnableOnReset,      
															   unsigned char Channels);
VL_OSALIB_API void			VSL_ADCSetAnalogInputRange(unsigned char Channel, unsigned char Range);
VL_OSALIB_API double		VSL_ADCGetAnalogInput(unsigned char Channel, unsigned char Format);
VL_OSALIB_API void			VSL_DACSetAnalogOutput(unsigned char Channel, unsigned short Level);
VL_OSALIB_API unsigned char VSL_GetADCType();

VL_OSALIB_API unsigned char	VSL_DIOGetChannelLevel(unsigned char Channel);
VL_OSALIB_API void			VSL_DIOSetChannelLevel(unsigned char Channel, unsigned char Level);
VL_OSALIB_API VL_APIStatusT	VSL_DIOGetChannelDirection(unsigned char Channel, unsigned char *pDirection);
VL_OSALIB_API void			VSL_DIOSetChannelDirection(unsigned char Channel, unsigned char Direction);
#ifdef WIN32
VL_OSALIB_API unsigned int	VSL_DIOSetupInterrupts(int Pid);
#endif
#ifdef linux
VL_OSALIB_API unsigned int	VSL_DIOSetupInterrupts(pid_t Pid);
#endif
VL_OSALIB_API void          VSL_DIOEnableInterruptGeneration(int state, int numOfDIO, ...);
VL_OSALIB_API void          VSL_DIOClearInterruptStatus(int numOfDIO, ...);

#ifdef WIN32
//VL_OSALIB_API unsigned char		VSL_TMRSet(unsigned char Timer, unsigned char Mode, unsigned short Count, BOOL External);
VL_OSALIB_API unsigned char		VSL_TMRSet(unsigned char Timer, unsigned char Mode, unsigned short Count, int External);
#endif  // WIN32
#ifdef linux
VL_OSALIB_API unsigned char		VSL_TMRSet(unsigned char Timer, unsigned char Mode, unsigned short Count, unsigned char External);
#endif  // linux
VL_OSALIB_API void				VSL_TMRClear(unsigned char Timer);
VL_OSALIB_API unsigned short	VSL_TMRGet(unsigned char Timer);
#ifdef WIN32
VL_OSALIB_API unsigned int		VSL_TMRGetEvent(unsigned char Timer, int Pid);
#endif
#ifdef linux
VL_OSALIB_API unsigned int		VSL_TMRGetEvent(unsigned char Timer, pid_t Pid);
#endif
VL_OSALIB_API unsigned int		VL_TMRRegisterHandler(void *Handler);
VL_OSALIB_API VL_APIStatusT     VSL_TMRGetSettings(unsigned char Timer, 
	                                               unsigned short *pCount, 
												   unsigned char *pMode, 
												   unsigned char *pType, 
												   unsigned char *pIntStatus, 
	                                               unsigned char *pIRQNum, 
												   unsigned char *pIRQEnable, 
												   unsigned char *pIRQStatus, 
	                                               unsigned char *pClockSelect);
 VL_OSALIB_API void              VSL_TMRCascadeTimers(unsigned char action);

/* Function in VL_MCU.c */
VL_OSALIB_API void VSL_CTimerSet(unsigned char Timer,
                                      unsigned char ResetEnable,
                                      unsigned char StopEnable,
                                      unsigned long MatchValue,
                                      unsigned char OutControl,
                                      unsigned char OutInitState,
                                      unsigned char IntEnable);
VL_OSALIB_API unsigned long    VSL_CTimerGet(unsigned char Timer);
VL_OSALIB_API void	    	VSL_CTimerStart(unsigned char Timer);
VL_OSALIB_API void    		VSL_CTimerStop(unsigned char Timer);
VL_OSALIB_API void    		VSL_CTimerReset(unsigned char Timer);
VL_OSALIB_API void          VSL_CTimerClear(unsigned char Timer);

/* API calls for Watchdog control and information */
VL_OSALIB_API unsigned char	VSL_WDTStatus();							// Returns a 0x01 (Enabled) or a 0x00 (Disabled)
VL_OSALIB_API unsigned char VSL_WDTGetValue();							// Returns the value in seconds (one byte)
VL_OSALIB_API void			VSL_WDTSetValue(unsigned char Value);		// Sets the value in seconds for the watchdog
VL_OSALIB_API void 			VSL_WDTResetEnable(unsigned char State);	// State can be ENABLE or DISABLE
VL_OSALIB_API void 			VSL_WDTEnable(unsigned char State);			// State can be ENABLE or DISABLE
VL_OSALIB_API void 			VSL_GetWDTResetEnable(unsigned char *pState);	// Get State
VL_OSALIB_API void 			VSL_GetWDTEnable(unsigned char *pState);		// Get State 

/* API calls for I2C control and Information */
VL_OSALIB_API VL_APIStatusT VSL_I2CIsAvailable(unsigned long busType);	// Returns VL_API_OK on success, or 
																		//VL_API_I2C_REQUESTED_BUS_NOT_FOUND on error
VL_OSALIB_API VL_APIStatusT VSL_I2CForceBusId(char *pBusName);	// Returns VL_API_OK on success, or 
																//VL_API_I2C_REQUESTED_BUS_NOT_FOUND on error

VL_OSALIB_API VL_APIStatusT VSL_I2CReadRegister(unsigned long bType, unsigned char addr, 
												unsigned short regNum, unsigned char *data);

/* Read data from the specified register at the specified address.
 * Return values:	VL_API_OK							=> successful read
 * 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
 * 					VL_API_I2C_READ_ERROR 				=> if an error occurs during reading
 * Read data is returned in an argument
 * */
VL_OSALIB_API VL_APIStatusT VSL_I2CReadAddress(unsigned long busType, unsigned char address, 
											   unsigned char *data, unsigned long numSequentialBytes);							

/* Write data to the specified register at the specified address.
 * Returns values:	VL_API_OK							=> successful write
 * 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
 *					VL_API_I2C_WRITE_REGISTER_ERROR		=> if an error occurs during reading
 * Read data is returned in an arguement
 * */												
VL_OSALIB_API VL_APIStatusT VSL_I2CWriteRegister(unsigned long busType, unsigned char address, 
												 unsigned short registerNum, unsigned char data);


/* Write data to the specified address.
 * Returns values:	VL_API_OK							=> successful write
 * 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
 *					VL_API_I2C_WRITE_ERROR 				=> if an error occurs during writing
 * Read data is returned in an arguement
 * */												 
VL_OSALIB_API VL_APIStatusT VSL_I2CWriteAddress(unsigned long busType, unsigned char address, 
												unsigned char *pData, unsigned long numSequentialBytes);

/* Combines writing to then reading from a device at a specified address in one step (no STOP condition 
   sent after the write, read is initiated with a start condition).
* Returns values:	VL_API_OK							=> successful write
* 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
*					VL_API_I2C_WRITE_ERROR 				=> if an error occurs during writing
* Read data is returned in an arguement
* */
VL_OSALIB_API VL_APIStatusT VSL_I2CWriteReadCombined(unsigned long busType,     unsigned char address,
														unsigned char *pWriteData, unsigned long numWriteBytes,
														unsigned char *pReadData,  unsigned long numReadBytes);

/* Retrieve the maximum frequency for the I2C bus.
 * Returns values:	VL_API_OK							=> successful retreival of the maximum bus frequency
 * 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
 *					VL_API_I2C_READ_REGISTER_ERROR 		=> if an error occurs during data retreival
 * Read data is returned in an arguement
 * */												
VL_OSALIB_API VL_APIStatusT VSL_I2CGetMaxFrequency(unsigned long busType, unsigned long *data);

/* Retrieve the current frequency for the I2C bus.
 * Returns values:	VL_API_OK							=> successful retreival of the current bus frequency
 * 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
 *					VL_API_I2C_READ_REGISTER_ERROR 		=> if an error occurs during data retrieval
 * Read data is returned in an arguement
 * */
VL_OSALIB_API VL_APIStatusT VSL_I2CGetFrequency(unsigned long busType, unsigned long *data);

/* Set the current frequency for the I2C bus.
 * Returns values:	VL_API_OK							=> successful setting of the current bus frequency
 * 					VL_API_I2C_REQUESTED_BUS_NOT_FOUND	=> if I2C bus not found
 *					VL_API_I2C_WRITE_REGISTER_ERROR 	=> if an error occurs during data setting
 * Read data is returned in an arguement
 * */
VL_OSALIB_API VL_APIStatusT VSL_I2CSetFrequency(unsigned long busType, unsigned long data);


/* API calls for onboard FPGA access. */
/* Read data from the specified FPGA register.
 * Return values:	VL_API_OK		=> successful read
 * 					VL_API_ERROR	=> if an error occurs during reading
 * Read data is returned in an argument
 * */
VL_OSALIB_API VL_APIStatusT VSL_FPGAReadRegister(unsigned long register, unsigned char *pdata);							

/* Write data to the specified register.
 * Returns values:	VL_API_OK			=> successful write
 *					VL_API_ERROR		=> if an error occurs during writing
 * */												
VL_OSALIB_API VL_APIStatusT VSL_FPGAWriteRegister(unsigned long registerNum, unsigned char data);

// ***************************** VersaAPI Calls to configure, sent and receive data on the SPI bus ******************
#include <stdint.h>
#define SPI_SS_SS0	0x01
#define SPI_SS_SS1	0x02
#define SPI_CLK_FREQ0	0x00
#define SPI_CLK_FREQ1	0x10
#define SPI_CLK_FREQ2	0x20
#define SPI_CLK_FREQ3	0x30

VL_OSALIB_API VL_APIStatusT VSL_SPIIsAvailable();
VL_OSALIB_API VL_APIStatusT VSL_SPISetFrequency(unsigned int Frequency);
VL_OSALIB_API VL_APIStatusT VSL_SPISetShiftDirection(unsigned int Direction);
VL_OSALIB_API VL_APIStatusT VSL_SPISetMode(unsigned int Mode);
VL_OSALIB_API VL_APIStatusT VSL_SPIReadDataFrame(uint32_t *pData);
VL_OSALIB_API VL_APIStatusT VSL_SPIWriteDataFrame(unsigned int SlaveSelect, uint32_t *pData);
VL_OSALIB_API VL_APIStatusT VSL_SPISetFrameSize(unsigned int Size);
// *************************************************************************

/* ***************** VersaPI calls for the MPEu-C1E (CAN) ***************** */
//#include "VL_CAN.h"

#ifndef _VLAPICAN_H
#define _VLAPICAN_H

/* ***** Includes ***** */
/* ******************** */

#ifdef usb
/* ***** From nxp-can.h ***** */


#ifndef __NXP_CAN_H__
#define __NXP_CAN_H__

// These defines and structures are derived from the LPC54616 user manual V2.2.

// 35.9 RX buffer and FIFO element, pg 729
// 35.10 Tx buffer element, pg 731

// R0 and T0
#define CAN_RX_TX_ID_MASK           (0x1FFFFFFF)
#define CAN_RX_TX_ID11_SHFT         (18)
#define CAN_RX_TX_REMOTE_FRAME      (0x20000000)
#define CAN_RX_TX_EXTENDED_ID       (0x40000000)
#define CAN_RX_TX_ERR_PASSIVE       (0x80000000)

#define CAN_RX_TX_GET_ID( var )             ((var & CAN_RX_TX_ID_MASK) >> CAN_RX_TX_ID11_SHFT)
#define CAN_RX_TX_SET_ID( var, id )         ((var & ~CAN_RX_TX_ID_MASK) | (id << CAN_RX_TX_ID11_SHFT))
#define CAN_RX_TX_GET_EXID( var )           (var & CAN_RX_TX_ID_MASK)
#define CAN_RX_TX_SET_EXID( var, id )       ((var & ~CAN_RX_TX_ID_MASK) | id)

// R1 and T1
#define CAN_RX_TX_SIZE_0            (0x00000000)
#define CAN_RX_TX_SIZE_1            (0x00010000)
#define CAN_RX_TX_SIZE_2            (0x00020000)
#define CAN_RX_TX_SIZE_3            (0x00030000)
#define CAN_RX_TX_SIZE_4            (0x00040000)
#define CAN_RX_TX_SIZE_5            (0x00050000)
#define CAN_RX_TX_SIZE_6            (0x00060000)
#define CAN_RX_TX_SIZE_7            (0x00070000)
#define CAN_RX_TX_SIZE_8            (0x00080000)
#define CAN_RX_TX_SIZE_12           (0x00090000)
#define CAN_RX_TX_SIZE_16           (0x000A0000)
#define CAN_RX_TX_SIZE_20           (0x000B0000)
#define CAN_RX_TX_SIZE_24           (0x000C0000)
#define CAN_RX_TX_SIZE_32           (0x000D0000)
#define CAN_RX_TX_SIZE_48           (0x000E0000)
#define CAN_RX_TX_SIZE_64           (0x000F0000)
#define CAN_RX_TX_BR_SWITCHING      (0x00100000)
#define CAN_RX_TX_CAN_FD_FORMAT     (0x00200000)
#define CAN_RX_TX_CAN_TX_EVENT      (0x00800000)

#define CAN_RX_TX_SET_SIZE( var, size )     ((var & 0xFFF0FFFF) | size)
#define CAN_RX_TX_GET_SIZE( var )           (var & 0x000F0000)

// R1
#define CAN_RX_TIMESTAMP_MASK       (0x0000FFFF)
#define CAN_RX_TIMESTAMP_SHFT       (0)
#define CAN_RX_FILTER_INDEX_MASK    (0x7E000000)
#define CAN_RX_FILTER_INDEX_SHFT    (24)
#define CAN_RX_FILTER_NO_MATCH      (0x80000000)

// T1
#define CAN_TX_USE_TX_EVENTS        (0x00800000)
#define CAN_TX_MSG_MARKER_MASK      (0xFF000000)
#define CAN_TX_MSG_MARKER_SHFT      (24)

#define CAN_MAX_FRAME_SIZE          (64)

// Status values returned in misc when status is called or returned.
#define CAN_STS_OPENED              (0x80)      // Upper bit set for open
#define CAN_STS_OK                  (0)
#define CAN_STS_ERROR               (1)
#define CAN_STS_TX_OK               (2)         // Send when tx FIFO is ready
#define CAN_STS_TX_BUSY             (3)         // Send when tx FIFO hits high
#define CAN_STS_TX_EVENT            (4)         // Send when tx FIFO hits high
#define CAN_STS_TX_ERROR            (5)
#define CAN_STS_RATE_ERROR          (6)
#define CAN_STS_INVALID_PORT        (7)
#define CAN_STS_BUS_ERROR           (8)
#define CAN_STS_UNDEFINED           (0x7F)

// MCU firmware version information
#define CAN_MCU_FW_VER_MAJOR			(0U)
#define CAN_MCU_FW_VER_MINOR			(1U)
#define CAN_MCU_FW_VER_RELEASE			(5U)
#define VL_MCU_MAKE_VERSION(major, minor, release) (((major) << 16) | ((minor) << 8) | (release))

//
// MCAN_TX_PACKET is an image of the MCAN TX buffer element in message area.
//
typedef struct _mcan_tx_packet_ {
    uint32_t            t0;
    uint32_t            t1;
    uint8_t             mcanTxData[ CAN_MAX_FRAME_SIZE ];
} MCAN_TX_PACKET;

//
// MCAN_TX_DONE is an image of the MCAN TX event E0 and E1 data.
//
typedef struct _mcan_tx_done_ {
    uint32_t            e0;
    uint32_t            e1;
} MCAN_TX_DONE;

//
// MCAN_RX_PACKET is an image of the MCAN RX buffer element in message area.
//
typedef struct _mcan_rx_packet_ {
    uint32_t            r0;
    uint32_t            r1;
    uint8_t             mcanRxData[ CAN_MAX_FRAME_SIZE ];
} MCAN_RX_PACKET;

//
// Contains items used on open. Bit rates and loop back can only be done on open.
//
typedef struct _mcan_open_ {
    uint32_t  arbitrationRate;   // In BPS
    uint32_t  dataRate;          // In BPS
    uint8_t   loopback;          // 'true' internal loop back
    uint8_t   rxEnabled;         // 
    uint32_t  masterDivider;     //
    uint16_t  arbPreDivider;     // Arbitration Clock Pre-scaler Division Factor.
    uint8_t   arbRJumpwidth;     // Arbitration Re-sync Jump Width.
    uint8_t   arbSeg1;           // Arbitration Data Time Segment 1.
    uint8_t   arbSeg2;           // Arbitration Data Time Segment 2.
    uint16_t  dataPreDivider;    // Data Clock Pre-scaler Division Factor.
    uint8_t   dataRJumpwidth;    // Data Re-sync Jump Width.
    uint8_t   dataSeg1;          // Data Data Time Segment 1.
    uint8_t   dataSeg2;          // Data Data Time Segment 2.
} MCAN_OPEN;

//
// Contains MCU fW version information.
//
typedef struct _mcan_mcu_fw_ver_ {
    uint8_t            major;        // Hex values
    uint8_t            minor;
    uint8_t            release;
} MCAN_MCU_FW_VER;

/*! @brief MCAN protocol timing characteristic configuration structure. */
typedef struct _mcan_timing_config
{
    uint16_t preDivider; /*!< Clock Pre-scaler Division Factor. */
    uint8_t rJumpwidth;  /*!< Re-sync Jump Width. */
    uint8_t seg1;        /*!< Data Time Segment 1. */
    uint8_t seg2;        /*!< Data Time Segment 2. */
} mcan_timing_config_t;

typedef struct _mcan_timing_regs_ {
    uint32_t                arbBitRate;
    uint32_t                dataBitRate;
    uint32_t                masterDivider;
    mcan_timing_config_t    arbCfg;
    mcan_timing_config_t    dataCfg;
} MCAN_TIMING;


//
// This is the packet send across the USB interface to control CAN and
// do CAN IO.
//
#define USB_CAN_CMD_UNDEFINED       (0)
#define USB_CAN_CMD_OPEN            (1)
#define USB_CAN_CMD_CLOSE           (2)
#define USB_CAN_CMD_TX              (3)
#define USB_CAN_CMD_RX              (4)
#define USB_CAN_CMD_STS             (5)
#define USB_CAN_CMD_ID              (6)     // Blink board LED at a fast rate
#define USB_CAN_CMD_MCU_PROGRAM     (7)
#define USB_CAN_CMD_RX_CTRL         (8)     // Used to turn MCAN RX on/off with misc
#define USB_CAN_CMD_MCU_FW_VER		(9)		// Return the version of the MCU code, major, minor, release
#define USB_CAN_CMD_MCU_REG_VAL		(10)	// Return specified register's value
#define USB_CAN_CMD_OPEN_USR_TIMING (11)    // Allow user to set CAN timing parameters on the port
#define USB_CAN_CMD_MAX             (12)    // Number of commands, add command, CHANGE!

#define USB_CAN_CMD_HDR_SIZE        (4)
#define USB_CAN_CMD_UNDEF_SIZE      (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_OPEN_SIZE       (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_OPEN ))
#define USB_CAN_CMD_CLOSE_SIZE      (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_TX_SIZE         (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_TX_PACKET ))
#define USB_CAN_CMD_TX_DONE_SIZE    (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_TX_DONE ))
#define USB_CAN_CMD_RX_SIZE         (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_RX_PACKET ))
#define USB_CAN_CMD_STS_SIZE        (USB_CAN_CMD_HDR_SIZE)      // Status returned in misc.
#define USB_CAN_CMD_ID_SIZE         (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_RX_CTRL_SIZE    (USB_CAN_CMD_HDR_SIZE)
//#define USB_CAN_CMD_OVERHEAD_SIZE (12)        // Offset into VL_USB_CAN_XFER for rx/tx data
#define USB_CAN_CMD_OVERHEAD_SIZE   (16)        // Offset into VL_USB_CAN_XFER for rx/tx data
#define USB_CAN_CMD_MCU_FW_VER_SIZE  (USB_CAN_CMD_HDR_SIZE) + sizeof( MCAN_MCU_FW_VER )
#define USB_CAN_CMD_MCU_REG_VAL_SIZE (USB_CAN_CMD_HDR_SIZE) + sizeof( uint32_t )        
#define USB_CAN_CMD_MCU_PROGRAM_SIZE (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_OPEN_USR_TIMING_SIZE (uint8_t)(USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_OPEN ) + 4)

// MCU register defines (for ease of reference
#define MCU_REG_DBTP (0)
#define MCU_REG_NBTP (1)
#define MCU_REG_TDCR (2)
#define MCU_REG_CCCR (3)
#define MCU_REG_PSR  (4)
#define MCU_REG_ECR  (5)

typedef struct _usb_can_packet_ {
    uint8_t             size;
    uint8_t             cmd;
    uint8_t             port;
    uint8_t             misc;
    uint32_t			extendedStatus;
    union {
        MCAN_OPEN           openInfo;
        MCAN_RX_PACKET      rxPacket;
        MCAN_TX_PACKET      txPacket;
        MCAN_TX_DONE        txEvent;
    } u;
} VL_USB_CAN_XFER;


// User Defined function called when a packet is recieved.
void userDefinedRxFunction(VL_USB_CAN_XFER *pRxPacket, int status);

#endif // __NXP_CAN_H__

/* ************************** */


/* ***** Defines. ***** */
#define DEFAULT_DEVICE_NAME	"/dev/ttyACM0"  // Default device name.
#define CAN_STR_LEN 64						// Generic string length.
#define ON 1
#define OFF 0
#define VL_ENABLE_RX_ON_TX		(1U)
#define VL_DISABLE_RX_ON_TX 	(0U)
#define VL_ENABLE_LOOPBACK		(1U)
#define VL_DISABLE_LOOPBACK		(0U)

/* USB defines. */
#define USB_CLASS_CDC_MODEM             ( 2 )       // CDC/Comm Modem
#define USB_CLASS_CDC_DATA              ( 10 )      // CDC/Comm Data

#define MAX_NUM_USB_DEVICES 2		// Maximum number of USB devies per board.
#define NUM_USBS_PER_DEVICE ( 2 )   // Number of USB channels per device.

// lsusb -v -d 0x1fc9:  <= lsusb command to get our device info.
#define OUR_VENDOR          ( 0x1FC9 )  // NXP vendor ID

// NXP Composit 2 VCOM product code.
#define OUR_PRODUCT         ( 0x00A3 )

// NXP VCOM Example enumeration
#define EP_BULK_IN          ( 0x83 )
#define EP_BULK_OUT         ( 0x03 )
/* ******************** */

/* ***** Globals ***** */
/* ******************* */

/* ***** Types/Structures. ***** */
typedef unsigned int uint32_t;
typedef int bool;

typedef struct
{
	unsigned char szVersionString[CAN_STR_LEN];
	unsigned char szDevicePath[CAN_STR_LEN];
	int			  deviceDesc;
} CAN_HANDLE, *pCAN_HANDLE;

// Storage for endpoints used by interfaces.
struct xfaceEpAddrs {
    int                                 canPort;        // 0 or 1
    uint8_t                             control;        // Modem ep address
    uint8_t                             bulkIn;         // Data input ep address
    uint8_t                             bulkOut;        // Data output ep address
};

struct deviceAccessInfo {
    struct libusb_device_descriptor     devDescriptor;
    libusb_device                       *pLibUsbDev;
    libusb_device_handle                *pDevHandle;
    int                                 numInterfaces;
    struct xfaceEpAddrs                 epAddrs[ NUM_USBS_PER_DEVICE ];
};

struct deviceAccessInfo devicesInfoEpAddrs[MAX_NUM_USB_DEVICES];
libusb_context          *pUsbContext; 

// Define the names for two CAN boards.
typedef enum
{
	VL_CAN_BOARD0	= 0,
	VL_CAN_BOARD1	
} VL_CANBoardT;

// Define the names for two CAN ports (per board).
typedef enum
{
	VL_CAN_PORT0	= 0,
	VL_CAN_PORT1,
	VL_CAN_PORTUNSPECIFIED
} VL_CANPortT;

// Define the Port status.
typedef enum
{
	VL_CAN_PORT_CLOSED	= 0,
	VL_CAN_PORT_OPENED,
	VL_CAN_PORT_UNKNOWN
} VL_CANPortStatusT;

// Define MPEe-C1E card status.
typedef enum
{
	VL_CAN_BOARD_CLOSED = 0,
	VL_CAN_BOARD_OPENED
} VL_CANBoardStatusT;
/* ***************************** */


/* CAN configuration.
	nominalBaudRate		=> Nominal bit rate in bits/second. For
						   classical CAN and CAN-FD frames that do not
						   use bit rate switching this is the speed the
						   frames will be tranmitted at.
	dataBaudRate		=> Bit rate for the data byte and checksum in
						   bit/second. Only used for CAN-FD frames that
						   use bit rate swtiching. Set to zero to
						   disable bit rate switching. Must be higher
								   than nominalBaudRate.
	baseAddress			=> Starting address in SRAM for data structures
						   used by the CAN controller, such as message
						   buffers, FIFOs and filters.
	timestampClock_Hz	=> The frequency of the timestamp clock in Hz.
						   Used for timestamping received messages.
						   Zero disables timestamping.
	rejectStandardRTR	=> If TRUE, then all 11-bit RTR frames received
						   will be ignored.
	rejectExtendedRTR	=> If TRUE, then all 29-bit RTR frames received 
						   will be ignored.
	enableLoopBack		=> Set to TRU to enable loopback mode for self-test.
	enableNonISOMode	=> Set to TRUE to use Bosch CAN-FD frame format (not
						   recommended). Set to FALSE to use ISO CAN-FD frame
						   format.
	disableFD			=> Set to TRUE to disable CAN-FD. All transmitted
						   frames will be classical frames. Any CAN-FD frames
						   will be converted into error frames.
*/
typedef struct
{
	uint32_t	nominalBaudRate;
	uint32_t	dataBaudRate;
	uint32_t	baseAddress;
	uint32_t	timestampClock_Hz;
	bool		rejectStandardRTR;
	bool		rejectExtendedRTR;
	bool		enableLoopBack;
	bool		enableNonISOMode;
	bool		disableFD;
} CAN_CONFIG, *pCAN_CONFIG;
/* *********************** */


/* VSL_CANGetVersion()
 * Retreives the version number of the VersaLogic CAN (MPEu-C1E) library libVL_CAN.
 *
 * Return values:
 *     None
 */
VL_OSALIB_API void VSL_CANGetVersion(unsigned char *Major, unsigned char *Minor, unsigned char *Release);


/* VSL_CANUpdateMCUFwVersion()
 * Place the MCU in update mode so dfu-util can update the firmware.
 *
 * Return values:
 *     None
 */
VL_OSALIB_API VL_APIStatusT VSL_CANUpdateMCUFwVersion(VL_CANBoardT boardName);


/* VSL_CANSetPortTimingParameters()
 * Opens the specified CAN port with the specified configuration. 
 * Let's the user set all CAN Port timing parameters.
 *
 * Return values:
 *     None
 */
VL_OSALIB_API VL_APIStatusT VSL_CANOpenPortWithUserTiming(
				                                  VL_CANBoardT boardName, 
											      VL_CANPortT  portName,
                                                  uint32_t	   arbitrationBitRate,
                                                  uint32_t	   dataBitRate,
												  uint32_t     rxEnabled,
												  uint32_t     loopbackEnabled,
                                                  uint32_t	   masterDivider,
                                                  uint16_t     arbPreDivider,
                                                  uint8_t      arbResyncJumpWidth,
                                                  uint8_t      arbSeg1,
                                                  uint8_t      arbSeg2,
                                                  uint16_t     dataPreDivider,
                                                  uint8_t      dataResyncJumpWidth,
                                                  uint8_t      dataSeg1,
                                                  uint8_t      dataSeg2);

/* VSL_CANOpenPort()
 * Opens the specified CAN port with the specified configuration. 
 *
 *  Return values:
 *		VL_API_OK						=> Successfully opened the CAN port
 *		VL_API_INVALID_ARG				=> Invalid input passed in.
 *		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
 *		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
 */
 VL_OSALIB_API VL_APIStatusT VSL_CANOpenPort(VL_CANBoardT	boardName, 
											 VL_CANPortT	portName, 
											 uint32_t		arbitrationBitRate,
											 uint32_t		dataBitRate,
											 uint32_t		rxEnabled,
											 uint32_t  		loopbackEnabled);

/*  VSL_CANClosePort()
 *  Closes the pecified CAN port.
 *
 *  Return values:
 *		VSL_API_OK				=> Successfully opened the CAN port.
 */
VL_OSALIB_API VL_APIStatusT VSL_CANClosePort(	VL_CANBoardT	boardName, 
												VL_CANPortT		portName);

/*  VSL_CANTransmit()
 *  Transmit the frame.
 *	Called after VSL_CANOpenPort().
 *	This routine assumes that the MCAN_TX_PACKET (standard CAN packet) is
 *  filled-out appropriately (i.e. t0, t1 and data fields).
 *
 *  Return values:
 *		VL_API_OK						=> Successfully opened the CAN port
 *		VL_API_INVALID_ARG				=> Invalid input passed in.
 *		VL_API_CAN_OPEN_DEVICE_ERROR	=> CAN device not open/ready.
 *		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
 */
VL_OSALIB_API VL_APIStatusT VSL_CANTransmit(	VL_CANBoardT  boardName, 
												VL_CANPortT   portName,
												MCAN_TX_PACKET *pTxPkt);



/*  VSL_CANGetPortStatus()
 *  Return the port status (open, or closed) for the named port.
 *	Called after VSL_CANOpenPort().
 *
 *  Return values:
 *		VL_API_OK						=> Successfully opened the CAN port
 *		VL_API_INVALID_ARG				=> Invalid input passed in.
 *		VL_API_CAN_OPEN_DEVICE_ERROR	=> CAN device not open/ready.
 *		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
 */
VL_OSALIB_API VL_APIStatusT VSL_CANGetPortStatus(	VL_CANBoardT       boardName, 
												    VL_CANPortT        portName,
												    VL_CANPortStatusT *pPortStatus);


/*  VSL_CANGetMCUFwVersion()
 *  Retrieve the MCU firmware version.
 *	Called after VSL_CANOpenPort().
 *
 *  Return values:
 *		VL_API_OK						=> Successfully read and returned the 
 *                                         MCU firmware version.
 */
VL_OSALIB_API VL_APIStatusT VSL_CANGetMCUFwVersion(    VL_CANBoardT  boardName,
                                                        uint32_t *pMcuFwVer);


/*  VSL_CANPrintPktHeader()
 *  Prints to stdout a VL CAN packet header.
 *
 *  Return values:
 *		VL_API_OK	=> Always.
 */
VL_OSALIB_API VL_APIStatusT VSL_CANPrintPktHeader(	char *prependString,
                                                    VL_USB_CAN_XFER *pPacket);

/* Break the PSR MCU register values into usable pieces.
 *  Return values:
 *		VL_API_OK						=> On success.
 *		VL_API_ERROR					=> When the register value cannot be retrieved.
 * */
VL_OSALIB_API VL_APIStatusT VSL_CANGetProtocolRegisterStatus(VL_CANBoardT boardName,
				                                             VL_CANPortT portName,
				                                             uint32_t *LEC,  uint32_t *ACT, 
			                                                 uint32_t *EP,   uint32_t *EW,
															 uint32_t *BO,   uint32_t *DLEC,
															 uint32_t *RESI, uint32_t *RBRS,
															 uint32_t *RFDF, uint32_t *PXE,
															 uint32_t *TDCV);


/* Split the ECR MCU register values into usable pieces.
 *  Return values:
 *		VL_API_OK						=> On success.
 *		VL_API_ERROR					=> When the register value cannot be retrieved.
 * */
VL_OSALIB_API VL_APIStatusT VSL_CANGetErrorCounterRegisterStatus(VL_CANBoardT boardName,
				                                                 VL_CANPortT portName,
				                                                 uint32_t *TEC,  uint32_t *REC, 
			                                                     uint32_t *RP,   uint32_t *CEL);

#endif  // _VLAPI_H_
/* ************************************************************************ */
#endif // usb


/* API calls for LCD Backlight control and Information */
/* Set the current LCD backlight level. Level range is between 0 (darkest) - 100 (brightest).
 * Return values:	VL_API_OK						=> successful setting of the backlight value
 *					VL_API_VGA_BL_INTERFACE_ERRROR	=> Error attempting to set backlight value
 *					VL_API_VGA_BL_VALUE_OUT_OF_RANGE =>	Attempting to set backlight value to 
 *														a value not in the range 0 <= value <= 100
 * */
VL_OSALIB_API VL_APIStatusT VSL_LCDSetBacklight(unsigned long newBLValue);

/* Get the current LCD backlight level. Level range is between 0 (darkest) - 100 (brightest).
 * Return values:	VL_API_OK						=> successfully retrieved of the backlight value
 *					VL_API_VGA_BL_INTERFACE_ERRROR	=> Error attempting to set backlight value
 *					VL_API_VGA_BL_VALUE_OUT_OF_RANGE =>	Retrieved backlight value not in the 
 *														range 0 <= value <= 100
 * */
VL_OSALIB_API VL_APIStatusT VSL_LCDGetBacklight(unsigned long *pNewBLValue);

/* Get the Maximum LCD backlight level. Level range is between 0 (darkest) - 100 (brightest).
 * Return values:	VL_API_OK						=> successfully retrieved of the backlight value
 *					VL_API_VGA_BL_INTERFACE_ERRROR	=> Error attempting to set backlight value
 *					VL_API_VGA_BL_VALUE_OUT_OF_RANGE =>	Retrieved backlight value not in the 
 *														range 0 <= value <= 100
 * */
VL_OSALIB_API VL_APIStatusT VSL_LCDSetDisplayName(char *pDisplayName);

/* Get the Maximum LCD backlight level. Level range is between 0 (darkest) - 100 (brightest).
 * Return values:	VL_API_OK						=> successfully retrieved of the backlight value
 *					VL_API_VGA_BL_INTERFACE_ERRROR	=> Error attempting to set backlight value
 *					VL_API_VGA_BL_VALUE_OUT_OF_RANGE =>	Retrieved backlight value not in the 
 *														range 0 <= value <= 100
 * */
VL_OSALIB_API VL_APIStatusT VSL_LCDGetBacklightMaximum(unsigned long *pNewBLValue);


/* Retrieve the current running time of the board in hours.
 * Returns values:	VL_API_OK					=> successful retreival of the current bus frequency
 * 					VL_API_GET_UPTIME_ERROR		=> if an error occurs during data retrieval
 * Read data is returned in an arguement
 * */
VL_OSALIB_API VL_APIStatusT VSL_GetUptime(unsigned long *data);

// Internal function calls to support ARM boards
#ifdef linux
VL_OSALIB_API unsigned char epc270xDIOGetChannelDirection(unsigned char Channel);
VL_OSALIB_API void epc270xWDTSetValue(unsigned char Value);
VL_OSALIB_API VL_APIStatusT epc270xDIOSetChannelLevel(unsigned char Channel, unsigned char Level);
VL_OSALIB_API VL_APIStatusT determineBoardArch();
VL_OSALIB_API VL_APIStatusT epc270xDIOSetChannelDirection(unsigned char Channel, unsigned char Direction);
VL_OSALIB_API unsigned char epc270xWDTGetValue();
VL_OSALIB_API void enableARMDIOs(int shouldExport);
VL_OSALIB_API void epc270xWDTResetEnable(unsigned char State);
VL_OSALIB_API unsigned char epc270xWDTStatus();
VL_OSALIB_API unsigned char epc270xDIOGetChannelLevel(unsigned char Channel);
VL_OSALIB_API unsigned char epc270xGetWDTResetEnable();
#endif  // linux

// For accessing MCU over I2C
VL_OSALIB_API unsigned char mcuDIOGetChannelDirection(unsigned char Channel);
VL_OSALIB_API VL_APIStatusT mcuDIOSetChannelLevel(unsigned char Channel, unsigned char Level);
VL_OSALIB_API VL_APIStatusT mcuDIOSetChannelDirection(unsigned char Channel, unsigned char Direction);
VL_OSALIB_API unsigned char mcuDIOGetChannelLevel(unsigned char Channel);


/* Specific VersaLogic board IDs, read from bits 0-6 at FPGA offset 0 */
#define VL_BOARD_EPIC_25	 0x0B
#define VL_BOARD_EBX_41		 0x0C
#define VL_BOARD_EBX_37		 0x05
#define VL_BOARD_EBX_17EA	 0x09
#define VL_BOARD_EBX_17EB	 0x09
#define VL_BOARD_EBX_18		 0x10
#define VL_BOARD_EPM_19		 0x12
#define VL_BOARD_EPM_21		 0x03
#define VL_BOARD_EPM_24		 0x07
#define VL_BOARD_EPIC_36	 0x04
#define VL_BOARD_EPM_30		 0x0F
#define VL_BOARD_EPM_31		 0x11
#define VL_BOARD_MPE_A1A2    0x0E
#define	VSL_ADC_TYPE_A2		 0x02
#define	VSL_ADC_TYPE_A1		 0x01
#define VL_BOARD_EPM_42		 0x15
#define VL_BOARD_EPU_3311	 0x13
#define VL_BOARD_EPU_3312	 0x14
#define VL_BOARD_EBX_38		 0x17
#define VL_BOARD_EPM_43		 0x16
#define VL_BOARD_EPU_4562	 0x18
#define VL_BOARD_EPU_4460	 0x19
#define VL_BOARD_BOSIND1	 0x1A
#define VL_BOARD_EPC_2700	 0x1B
#define VL_BOARD_EPC_2701	 0x1C
#define VL_BOARD_ESU_5070    0x1D
#define VL_BOARD_EPC_2702	 0x1F
#define VL_BOARD_UNKNOWN     0x00
#define VL_BOARD_EPM_39		 0x1E
#define VL_BOARD_EPU_4011    0x21    // Defined in Q01488
#define VL_BOARD_EPU_4012    0x22    // Defined in Q01488
#define VL_BOARD_EPME_51N    0x24    // CPU board
#define VL_BOARD_EPME_51S    0x25    // Carrier (I/O) board, or EPMe-51S-01
#define VL_BOARD_EPME_51S_02 0x26    // Carrier (I/O) board
#define VL_BOARD_EPU_5121    0x27    // Swift ... black bird carrier + new CPU board
#define VL_BOARD_EPME_51     0xAA    // ...EPME_51N + ...EPME_51S
#define VL_BOARD_EPU_5120    0xAB    // ...EPME_51N + ...EPME_51S-02


// Helpful channel defines
#define SPI_AI_CHANNEL_1		0x00
#define SPI_AI_CHANNEL_2		0x40
#define SPI_AI_CHANNEL_3		0x10
#define SPI_AI_CHANNEL_4		0x50
#define SPI_AI_CHANNEL_5		0x20
#define SPI_AI_CHANNEL_6		0x60
#define SPI_AI_CHANNEL_7		0x30
#define SPI_AI_CHANNEL_8		0x70
#define SPI_AI_CHANNEL_9		0x01
#define SPI_AI_CHANNEL_10		0x41
#define SPI_AI_CHANNEL_11		0x11
#define SPI_AI_CHANNEL_12		0x51
#define SPI_AI_CHANNEL_13		0x21
#define SPI_AI_CHANNEL_14		0x61
#define SPI_AI_CHANNEL_15		0x31
#define SPI_AI_CHANNEL_16		0x71

#define SPI_AI_DIFF_CHANNEL_1POS	0x80
#define SPI_AI_DIFF_CHANNEL_3POS	0xc0
#define SPI_AI_DIFF_CHANNEL_5POS	0x90
#define SPI_AI_DIFF_CHANNEL_7POS	0xd0
#define SPI_AI_DIFF_CHANNEL_2POS	0xa0
#define SPI_AI_DIFF_CHANNEL_4POS	0xe0
#define SPI_AI_DIFF_CHANNEL_6POS	0xb0
#define SPI_AI_DIFF_CHANNEL_8POS	0xf0
#define SPI_AI_DIFF_CHANNEL_9POS	0x81
#define SPI_AI_DIFF_CHANNEL_11POS	0xc1
#define SPI_AI_DIFF_CHANNEL_13POS	0x91
#define SPI_AI_DIFF_CHANNEL_15POS	0xd1
#define SPI_AI_DIFF_CHANNEL_10POS	0xa1
#define SPI_AI_DIFF_CHANNEL_12POS	0xe1
#define SPI_AI_DIFF_CHANNEL_14POS	0xb1
#define SPI_AI_DIFF_CHANNEL_16POS	0xf1

#define SPX_AI_SS0_CHANNEL_1		0x04
#define SPX_AI_SS0_CHANNEL_2		0x14
#define SPX_AI_SS0_CHANNEL_3		0x24
#define SPX_AI_SS0_CHANNEL_4		0x34
#define SPX_AI_SS0_CHANNEL_5		0x44
#define SPX_AI_SS0_CHANNEL_6		0x54
#define SPX_AI_SS0_CHANNEL_7		0x64
#define SPX_AI_SS0_CHANNEL_8		0x74

#define SPX_AI_SS1_CHANNEL_1		0x05
#define SPX_AI_SS1_CHANNEL_2		0x15
#define SPX_AI_SS1_CHANNEL_3		0x25
#define SPX_AI_SS1_CHANNEL_4		0x35
#define SPX_AI_SS1_CHANNEL_5		0x45
#define SPX_AI_SS1_CHANNEL_6		0x55
#define SPX_AI_SS1_CHANNEL_7		0x65
#define SPX_AI_SS1_CHANNEL_8		0x75

#define SPX_AI_SS2_CHANNEL_1		0x06
#define SPX_AI_SS2_CHANNEL_2		0x16
#define SPX_AI_SS2_CHANNEL_3		0x26
#define SPX_AI_SS2_CHANNEL_4		0x36
#define SPX_AI_SS2_CHANNEL_5		0x46
#define SPX_AI_SS2_CHANNEL_6		0x56
#define SPX_AI_SS2_CHANNEL_7		0x66
#define SPX_AI_SS2_CHANNEL_8		0x76

#define SPX_AI_SS3_CHANNEL_1		0x07
#define SPX_AI_SS3_CHANNEL_2		0x17
#define SPX_AI_SS3_CHANNEL_3		0x27
#define SPX_AI_SS3_CHANNEL_4		0x37
#define SPX_AI_SS3_CHANNEL_5		0x47
#define SPX_AI_SS3_CHANNEL_6		0x57
#define SPX_AI_SS3_CHANNEL_7		0x67
#define SPX_AI_SS3_CHANNEL_8		0x77

#define SPX_AD7490_AI_SS0_CHANNEL_1		0x0a
#define SPX_AD7490_AI_SS0_CHANNEL_2		0x1a
#define SPX_AD7490_AI_SS0_CHANNEL_3		0x2a
#define SPX_AD7490_AI_SS0_CHANNEL_4		0x3a
#define SPX_AD7490_AI_SS0_CHANNEL_5		0x4a
#define SPX_AD7490_AI_SS0_CHANNEL_6		0x5a
#define SPX_AD7490_AI_SS0_CHANNEL_7		0x6a
#define SPX_AD7490_AI_SS0_CHANNEL_8		0x7a
#define SPX_AD7490_AI_SS0_CHANNEL_9		0x8a
#define SPX_AD7490_AI_SS0_CHANNEL_10	0x9a
#define SPX_AD7490_AI_SS0_CHANNEL_11	0xaa
#define SPX_AD7490_AI_SS0_CHANNEL_12	0xba
#define SPX_AD7490_AI_SS0_CHANNEL_13	0xca
#define SPX_AD7490_AI_SS0_CHANNEL_14	0xda
#define SPX_AD7490_AI_SS0_CHANNEL_15	0xea
#define SPX_AD7490_AI_SS0_CHANNEL_16	0xfa

#define SPX_AD7490_AI_SS1_CHANNEL_1		0x0b
#define SPX_AD7490_AI_SS1_CHANNEL_2		0x1b
#define SPX_AD7490_AI_SS1_CHANNEL_3		0x2b
#define SPX_AD7490_AI_SS1_CHANNEL_4		0x3b
#define SPX_AD7490_AI_SS1_CHANNEL_5		0x4b
#define SPX_AD7490_AI_SS1_CHANNEL_6		0x5b
#define SPX_AD7490_AI_SS1_CHANNEL_7		0x6b
#define SPX_AD7490_AI_SS1_CHANNEL_8		0x7b
#define SPX_AD7490_AI_SS1_CHANNEL_9		0x8b
#define SPX_AD7490_AI_SS1_CHANNEL_10	0x9b
#define SPX_AD7490_AI_SS1_CHANNEL_11	0xab
#define SPX_AD7490_AI_SS1_CHANNEL_12	0xbb
#define SPX_AD7490_AI_SS1_CHANNEL_13	0xcb
#define SPX_AD7490_AI_SS1_CHANNEL_14	0xdb
#define SPX_AD7490_AI_SS1_CHANNEL_15	0xeb
#define SPX_AD7490_AI_SS1_CHANNEL_16	0xfb

#define SPX_AD7490_AI_SS2_CHANNEL_1		0x0c
#define SPX_AD7490_AI_SS2_CHANNEL_2		0x1c
#define SPX_AD7490_AI_SS2_CHANNEL_3		0x2c
#define SPX_AD7490_AI_SS2_CHANNEL_4		0x3c
#define SPX_AD7490_AI_SS2_CHANNEL_5		0x4c
#define SPX_AD7490_AI_SS2_CHANNEL_6		0x5c
#define SPX_AD7490_AI_SS2_CHANNEL_7		0x6c
#define SPX_AD7490_AI_SS2_CHANNEL_8		0x7c
#define SPX_AD7490_AI_SS2_CHANNEL_9		0x8c
#define SPX_AD7490_AI_SS2_CHANNEL_10	0x9c
#define SPX_AD7490_AI_SS2_CHANNEL_11	0xac
#define SPX_AD7490_AI_SS2_CHANNEL_12	0xbc
#define SPX_AD7490_AI_SS2_CHANNEL_13	0xcc
#define SPX_AD7490_AI_SS2_CHANNEL_14	0xdc
#define SPX_AD7490_AI_SS2_CHANNEL_15	0xec
#define SPX_AD7490_AI_SS2_CHANNEL_16	0xfc

#define SPX_AD7490_AI_SS3_CHANNEL_1		0x0d
#define SPX_AD7490_AI_SS3_CHANNEL_2		0x1d
#define SPX_AD7490_AI_SS3_CHANNEL_3		0x2d
#define SPX_AD7490_AI_SS3_CHANNEL_4		0x3d
#define SPX_AD7490_AI_SS3_CHANNEL_5		0x4d
#define SPX_AD7490_AI_SS3_CHANNEL_6		0x5d
#define SPX_AD7490_AI_SS3_CHANNEL_7		0x6d
#define SPX_AD7490_AI_SS3_CHANNEL_8		0x7d
#define SPX_AD7490_AI_SS3_CHANNEL_9		0x8d
#define SPX_AD7490_AI_SS3_CHANNEL_10	0x9d
#define SPX_AD7490_AI_SS3_CHANNEL_11	0xad
#define SPX_AD7490_AI_SS3_CHANNEL_12	0xbd
#define SPX_AD7490_AI_SS3_CHANNEL_13	0xcd
#define SPX_AD7490_AI_SS3_CHANNEL_14	0xdd
#define SPX_AD7490_AI_SS3_CHANNEL_15	0xed
#define SPX_AD7490_AI_SS3_CHANNEL_16	0xfd

#define SPX_AD7927_AI_SS0_CHANNEL_1     0x02
#define SPX_AD7927_AI_SS0_CHANNEL_2		0x12
#define SPX_AD7927_AI_SS0_CHANNEL_3		0x22
#define SPX_AD7927_AI_SS0_CHANNEL_4		0x32
#define SPX_AD7927_AI_SS0_CHANNEL_5		0x42
#define SPX_AD7927_AI_SS0_CHANNEL_6		0x52
#define SPX_AD7927_AI_SS0_CHANNEL_7		0x62
#define SPX_AD7927_AI_SS0_CHANNEL_8		0x72

#define SPX_AD7927_AI_SS1_CHANNEL_1		0x03
#define SPX_AD7927_AI_SS1_CHANNEL_2		0x13
#define SPX_AD7927_AI_SS1_CHANNEL_3		0x23
#define SPX_AD7927_AI_SS1_CHANNEL_4		0x33
#define SPX_AD7927_AI_SS1_CHANNEL_5		0x43
#define SPX_AD7927_AI_SS1_CHANNEL_6		0x53
#define SPX_AD7927_AI_SS1_CHANNEL_7		0x63
#define SPX_AD7927_AI_SS1_CHANNEL_8		0x73

#define SPX_AD7927_AI_SS2_CHANNEL_1		0x08
#define SPX_AD7927_AI_SS2_CHANNEL_2		0x18
#define SPX_AD7927_AI_SS2_CHANNEL_3		0x28
#define SPX_AD7927_AI_SS2_CHANNEL_4		0x38
#define SPX_AD7927_AI_SS2_CHANNEL_5		0x48
#define SPX_AD7927_AI_SS2_CHANNEL_6		0x58
#define SPX_AD7927_AI_SS2_CHANNEL_7		0x68
#define SPX_AD7927_AI_SS2_CHANNEL_8		0x78

#define SPX_AD7927_AI_SS3_CHANNEL_1		0x09
#define SPX_AD7927_AI_SS3_CHANNEL_2		0x19
#define SPX_AD7927_AI_SS3_CHANNEL_3		0x29
#define SPX_AD7927_AI_SS3_CHANNEL_4		0x39
#define SPX_AD7927_AI_SS3_CHANNEL_5		0x49
#define SPX_AD7927_AI_SS3_CHANNEL_6		0x59
#define SPX_AD7927_AI_SS3_CHANNEL_7		0x69
#define SPX_AD7927_AI_SS3_CHANNEL_8		0x79

#define SPI_AO_CHANNEL_1		0x00
#define SPI_AO_CHANNEL_2		0x01
#define SPI_AO_CHANNEL_3		0x02
#define SPI_AO_CHANNEL_4		0x03
#define SPI_AO_CHANNEL_5		0x04
#define SPI_AO_CHANNEL_6		0x05
#define SPI_AO_CHANNEL_7		0x06
#define SPI_AO_CHANNEL_8		0x07

#define SPX_AO_SS0_CHANNEL_1		0x40
#define SPX_AO_SS0_CHANNEL_2		0x41
#define SPX_AO_SS0_CHANNEL_3		0x42
#define SPX_AO_SS0_CHANNEL_4		0x43
#define SPX_AO_SS0_CHANNEL_5		0x44
#define SPX_AO_SS0_CHANNEL_6		0x45
#define SPX_AO_SS0_CHANNEL_7		0x46
#define SPX_AO_SS0_CHANNEL_8		0x47
#define SPX_AO_SS1_CHANNEL_1		0x50
#define SPX_AO_SS1_CHANNEL_2		0x51
#define SPX_AO_SS1_CHANNEL_3		0x52
#define SPX_AO_SS1_CHANNEL_4		0x53
#define SPX_AO_SS1_CHANNEL_5		0x54
#define SPX_AO_SS1_CHANNEL_6		0x55
#define SPX_AO_SS1_CHANNEL_7		0x56
#define SPX_AO_SS1_CHANNEL_8		0x57
#define SPX_AO_SS2_CHANNEL_1		0x60
#define SPX_AO_SS2_CHANNEL_2		0x61
#define SPX_AO_SS2_CHANNEL_3		0x62
#define SPX_AO_SS2_CHANNEL_4		0x63
#define SPX_AO_SS2_CHANNEL_5		0x64
#define SPX_AO_SS2_CHANNEL_6		0x65
#define SPX_AO_SS2_CHANNEL_7		0x66
#define SPX_AO_SS2_CHANNEL_8		0x67
#define SPX_AO_SS3_CHANNEL_1		0x70
#define SPX_AO_SS3_CHANNEL_2		0x71
#define SPX_AO_SS3_CHANNEL_3		0x72
#define SPX_AO_SS3_CHANNEL_4		0x73
#define SPX_AO_SS3_CHANNEL_5		0x74
#define SPX_AO_SS3_CHANNEL_6		0x75
#define SPX_AO_SS3_CHANNEL_7		0x76
#define SPX_AO_SS3_CHANNEL_8		0x77

#define PCI_AI_CHANNEL_1		0x0E
#define PCI_AI_CHANNEL_2		0x4E
#define PCI_AI_CHANNEL_3		0x1E
#define PCI_AI_CHANNEL_4		0x5E
#define PCI_AI_CHANNEL_5		0x2E
#define PCI_AI_CHANNEL_6		0x6E
#define PCI_AI_CHANNEL_7		0x3E
#define PCI_AI_CHANNEL_8		0x7E

#define PCI_AI_DIFF_CHANNEL_1POS	0x0F
#define PCI_AI_DIFF_CHANNEL_3POS	0x1F
#define PCI_AI_DIFF_CHANNEL_5POS	0x2F
#define PCI_AI_DIFF_CHANNEL_7POS	0x3F

#define PCI_AI_DIFF_CHANNEL_2POS	0x4F
#define PCI_AI_DIFF_CHANNEL_4POS	0x5F
#define PCI_AI_DIFF_CHANNEL_6POS	0x6F
#define PCI_AI_DIFF_CHANNEL_8POS	0x7F

#define CHANNEL_UNKNOWN			0xFF
#define DIO_CHANNEL_1			0x00
#define DIO_CHANNEL_2			0x01
#define DIO_CHANNEL_3			0x02
#define DIO_CHANNEL_4			0x03
#define DIO_CHANNEL_5			0x04
#define DIO_CHANNEL_6			0x05
#define DIO_CHANNEL_7			0x06
#define DIO_CHANNEL_8			0x07
#define DIO_CHANNEL_9			0x08
#define DIO_CHANNEL_10			0x09
#define DIO_CHANNEL_11			0x0a
#define DIO_CHANNEL_12			0x0b
#define DIO_CHANNEL_13			0x0c
#define DIO_CHANNEL_14			0x0d
#define DIO_CHANNEL_15			0x0e
#define DIO_CHANNEL_16			0x0f

#define DIO_CHANNEL_17			0x10
#define DIO_CHANNEL_18			0x11
#define DIO_CHANNEL_19			0x12
#define DIO_CHANNEL_20			0x13
#define DIO_CHANNEL_21			0x14
#define DIO_CHANNEL_22			0x15
#define DIO_CHANNEL_23			0x16
#define DIO_CHANNEL_24			0x17
#define DIO_CHANNEL_25			0x18
#define DIO_CHANNEL_26			0x19
#define DIO_CHANNEL_27			0x1a
#define DIO_CHANNEL_28			0x1b
#define DIO_CHANNEL_29			0x1c
#define DIO_CHANNEL_30			0x1d
#define DIO_CHANNEL_31			0x1e
#define DIO_CHANNEL_32			0x1f

#define DIO_CHANNEL_33			0x20
#define DIO_CHANNEL_34			0x21
#define DIO_CHANNEL_35			0x22
#define DIO_CHANNEL_36			0x23
#define DIO_CHANNEL_37			0x24
#define DIO_CHANNEL_38			0x25
#define DIO_CHANNEL_39			0x26
#define DIO_CHANNEL_40			0x27

#define DIO_SS0_CHANNEL_1		0x40
#define DIO_SS0_CHANNEL_2		0x41
#define DIO_SS0_CHANNEL_3		0x42
#define DIO_SS0_CHANNEL_4		0x43
#define DIO_SS0_CHANNEL_5		0x44
#define DIO_SS0_CHANNEL_6		0x45
#define DIO_SS0_CHANNEL_7		0x46
#define DIO_SS0_CHANNEL_8		0x47
#define DIO_SS0_CHANNEL_9		0x48
#define DIO_SS0_CHANNEL_10		0x49
#define DIO_SS0_CHANNEL_11		0x4a
#define DIO_SS0_CHANNEL_12		0x4b
#define DIO_SS0_CHANNEL_13		0x4c
#define DIO_SS0_CHANNEL_14		0x4d
#define DIO_SS0_CHANNEL_15		0x4e
#define DIO_SS0_CHANNEL_16		0x4f

#define DIO_SS1_CHANNEL_1		0x50
#define DIO_SS1_CHANNEL_2		0x51
#define DIO_SS1_CHANNEL_3		0x52
#define DIO_SS1_CHANNEL_4		0x53
#define DIO_SS1_CHANNEL_5		0x54
#define DIO_SS1_CHANNEL_6		0x55
#define DIO_SS1_CHANNEL_7		0x56
#define DIO_SS1_CHANNEL_8		0x57
#define DIO_SS1_CHANNEL_9		0x58
#define DIO_SS1_CHANNEL_10		0x59
#define DIO_SS1_CHANNEL_11		0x5a
#define DIO_SS1_CHANNEL_12		0x5b
#define DIO_SS1_CHANNEL_13		0x5c
#define DIO_SS1_CHANNEL_14		0x5d
#define DIO_SS1_CHANNEL_15		0x5e
#define DIO_SS1_CHANNEL_16		0x5f

#define DIO_SS2_CHANNEL_1		0x60
#define DIO_SS2_CHANNEL_2		0x61
#define DIO_SS2_CHANNEL_3		0x62
#define DIO_SS2_CHANNEL_4		0x63
#define DIO_SS2_CHANNEL_5		0x64
#define DIO_SS2_CHANNEL_6		0x65
#define DIO_SS2_CHANNEL_7		0x66
#define DIO_SS2_CHANNEL_8		0x67
#define DIO_SS2_CHANNEL_9		0x68
#define DIO_SS2_CHANNEL_10		0x69
#define DIO_SS2_CHANNEL_11		0x6a
#define DIO_SS2_CHANNEL_12		0x6b
#define DIO_SS2_CHANNEL_13		0x6c
#define DIO_SS2_CHANNEL_14		0x6d
#define DIO_SS2_CHANNEL_15		0x6e
#define DIO_SS2_CHANNEL_16		0x6f

#define DIO_SS3_CHANNEL_1		0x70
#define DIO_SS3_CHANNEL_2		0x71
#define DIO_SS3_CHANNEL_3		0x72
#define DIO_SS3_CHANNEL_4		0x73
#define DIO_SS3_CHANNEL_5		0x74
#define DIO_SS3_CHANNEL_6		0x75
#define DIO_SS3_CHANNEL_7		0x76
#define DIO_SS3_CHANNEL_8		0x77
#define DIO_SS3_CHANNEL_9		0x78
#define DIO_SS3_CHANNEL_10		0x79
#define DIO_SS3_CHANNEL_11		0x7a
#define DIO_SS3_CHANNEL_12		0x7b
#define DIO_SS3_CHANNEL_13		0x7c
#define DIO_SS3_CHANNEL_14		0x7d
#define DIO_SS3_CHANNEL_15		0x7e
#define DIO_SS3_CHANNEL_16		0x7f

#define DIO_AX_CHANNEL_1		0x80
#define DIO_AX_CHANNEL_2		0x81
#define DIO_AX_CHANNEL_3		0x82
#define AX_CHANNEL_MASK			0x0F

#define DIO_U2_CHANNEL_1		0xA0
#define DIO_U2_CHANNEL_2		0xA1
#define DIO_U2_CHANNEL_3		0xA2
#define DIO_U2_CHANNEL_4		0xA3
#define DIO_U2_CHANNEL_5		0xA4
#define DIO_U2_CHANNEL_6		0xA5
#define DIO_U2_CHANNEL_7		0xA6
#define DIO_U2_CHANNEL_8		0xA7
#define DIO_U2_CHANNEL_9		0xA8
#define DIO_U2_CHANNEL_10		0xA9
#define DIO_U2_CHANNEL_11		0xAA
#define DIO_U2_CHANNEL_12		0xAB
#define DIO_U2_CHANNEL_13		0xAC
#define U2_CHANNEL_MASK			0x0F

#define EXAR_DVID				0x84

#define DIO_U2_BASE				0x080
#define DIO_U2_DREV				0x08C
#define DIO_U2_DVID				0x08D
#define DIO_U2_REGB				0x08E
#define DIO_U2_MPIOINT_7_0		0x08F
#define DIO_U2_MPIOINT_15_8		0x095
#define DIO_U2_MPIOLVL_7_0		0x090
#define DIO_U2_MPIOLVL_15_8		0x096
#define DIO_U2_MPIO3T_7_0		0x091
#define DIO_U2_MPIO3T_15_8		0x097
#define DIO_U2_MPIOINV_7_0		0x092
#define DIO_U2_MPIOINV_15_8		0x098
#define DIO_U2_MPIOSEL_7_0		0x093
#define DIO_U2_MPIOSEL_15_8		0x099
#define DIO_U2_MPIOOD_7_0		0x094
#define DIO_U2_MPIOOD_15_8		0x09A

#define SPI_RANGE_PM5V		0x00  // Valid for all boards.
#define SPI_RANGE_PM10V		0x04  // Valid for all boards.
#define SPI_RANGE_PM2P5V	0x02  // Valid for boards using TI ADS8668.
#define SPI_RANGE_PM1P25V	0x03  // Valid for boards using TI ADS8668.
#define SPI_RANGE_PM0P625V	0x0B  // Valid for boards using TI ADS8668.
#define SPI_RANGE_0_5V		0x08  // Valid for all boards.
#define SPI_RANGE_0_10V		0x0C  // Valid for all boards.
#define SPI_RANGE_0_2P5V    0x07  // Valid for boards using TI ADS8668.
#define SPI_RANGE_0_1P25V   0x0F  // Valid for boards using TI ADS8668.
#define SPI_RANGE_UNDEF     0xFF  // Not valid range.

#define ADS8668_AUTO_SEQ_EN 0x01  // Enable auto scan sequence mode for the TI ADS8668.

#define PCI_RANGE_PM5V		0x00
#define PCI_RANGE_PM10V		0x04
#define PCI_RANGE_0_5V		0x08
#define PCI_RANGE_0_10V		0x0C

#define TMR8254_DEFAULT_BASE	0x3C // always 16 bytes past the base address of the FPGA registers
#define TMR8254_COUNTER0	TMR8254_DEFAULT_BASE // always 16 bytes past the base address of the FPGA registers
#define TMR8254_COUNTER1	TMR8254_DEFAULT_BASE+1
#define TMR8254_COUNTER2	TMR8254_DEFAULT_BASE+2
#define TMR8254_CONTROL		0x3F

#define TMR8254_OB_DEFAULT_BASE	0x08 // always 16 bytes past the base address of the FPGA registers
#define TMR8254_OB_COUNTER0	TMR8254_OB_DEFAULT_BASE // always 16 bytes past the base address of the FPGA registers
#define TMR8254_OB_COUNTER1	TMR8254_OB_DEFAULT_BASE+1
#define TMR8254_OB_COUNTER2	TMR8254_OB_DEFAULT_BASE+2
//#define TMR8254_OB_CONTROL	0xBF
#define TMR8254_OB_CONTROL	0xB

#define TMR8254_CONTROL_EPME_51         0x1F
#define TMR8254_DEFAULT_BASE_EPME_51    0x1C 
#define TMR8254_COUNTER0_EPME_51        TMR8254_DEFAULT_BASE_EPME_51

#define TMR8254_MODE0		0x00
#define TMR8254_MODE1		0x02
#define TMR8254_MODE2		0x04
#define TMR8254_MODE3		0x06
#define TMR8254_MODE4		0x08
#define TMR8254_MODE5		0x0a


/* Defines for backwards compatability. User should use the VSL_* commands. */
#define VL_Open 					VSL_Open
#define VL_Close 					VSL_Close
#define VL_GetVersion				VSL_GetVersion
#define VL_FindProduct				VSL_FindProducts
#define VL_DIOGetChannelLevel		VSL_DIOGetChannelLevel
#define VL_DIOSetChannelLevel		VSL_DIOSetChannelLevel
#define VL_DIOSetChannelDirection	VSL_DIOSetChannelDirection
#define	VL_ADCSetAnalogInputRange	VSL_ADCSetAnalogInputRange
#define	VL_ADCGetAnalogInput		VSL_ADCGetAnalogInput
#define	VL_DACSetAnalogOutput		VSL_DACSetAnalogOutput
#define	VL_GetADCType				VSL_GetADCType
#define	VL_TMRSet					VSL_TMRSet
#define	VL_TMRClear					VSL_TMRClear
#define VL_TMRGet					VSL_TMRGet
#define VL_TMRGetEvent				VSL_TMRGetEvent
#define VL_FanGetRPM				VSL_FanGetRPM
#define VL_FanOn					VSL_FanOn
#define	VL_FanOff					VSL_FanOff
#define VL_WDTStatus				VSL_WDTStatus
#define VL_WDTGetValue				VSL_WDTGetValue
#define VL_WDTSetValue				VSL_WDTSetValue
#define VL_WDTResetEnable			VSL_WDTResetEnable
#define VL_WDTEnable				VSL_WDTEnable

/* I2C Bus types. */
#ifndef CGOS_I2C_TYPE_PRIMARY
#define CGOS_I2C_TYPE_PRIMARY 0x00010000  // primary I2C bus
#endif  // CGOS_I2C_TYPE_PRIMARY

// Only VL_I2C_BUS_TYPE_PRIMARY is supported at this time.
#define VL_I2C_BUS_TYPE_PRIMARY	CGOS_I2C_TYPE_PRIMARY
#define VL_I2C_BUS_TYPE_SMB 	CGOS_I2C_TYPE_SMB
#define VL_I2C_BUS_TYPE_DDC 	CGOS_I2C_TYPE_DDC
#define VL_I2C_BUS_TYPE_UNKNOWN CGOS_I2C_TYPE_UNKNOWN

/* I2C Bus frequencies. */
#define VL_I2C_FREQUENCY_100KHZ	100000
#define VL_I2C_FREQUENCY_400KHZ	400000

/*============================ BELOW this line are APIs for internal VersaLogic Use only ======================== */

/* ***** Debug enabling code. ***** */
#define	VL_DEBUG_OFF	0	// No debug messages printed.
#define VL_DEBUG_1		1	// Some debug messages printed.
#define VL_DEBUG_2		2 	// More debug messages printed.

#define VL_ARCH_X86			0	// x86 architecture
#define VL_ARCH_ARM			1	// ARM architecture
#define VL_ARCH_UNKNOWN		2	// Unknown architecture

int	gAPIDebugLevel;	// Global debug level. Set by VSL_DebugLevelSet().
int gBoardArchType; // Global board architecture type;
char gARMBoardType;	// Global ARM board type, only valid if 
					// gBoardArchType == VL_ARCH_ARM

// Function to set the debug level.
VL_OSALIB_API VL_APIStatusT VSL_DebugLevelSet(int debugLevel);
/* ******************************** */

/* Baseboard controller calls. */
VL_APIStatusT BC_InitLibraryAndBoard();		// Initizlize the baseboard controller library.
VL_APIStatusT BC_CloseLibraryAndBoard();	// Close the baseboard controller library.

/* VGA Backlight controller calls. */
VL_APIStatusT BC_VGASetBLValue(unsigned long newBLValue);	// Set the backlight value.
VL_APIStatusT BC_VGAGetBLValue(unsigned long  *pCurBLValue);	// Get the backlight value.

/* CPU Fan calls. */
VL_APIStatusT BC_FanRPMGet(unsigned short *RPMReading);  // Get the current RPMs for the CPU fan


/* BIOS bank calls. */
// Refer to the BIOS and Hardware Reference Manuals for details.
VL_OSALIB_API VL_APIStatusT 	VSL_GetActiveBIOSBank(unsigned char *BBank);	// BBank = BIOS_PRIMARY_BANK
																				// BBank = BIOS_SECONDARY_BANK
VL_OSALIB_API VL_APIStatusT 	VSL_SetActiveBIOSBank(unsigned char BBank);		// BBank = Bank to become active
VL_OSALIB_API VL_APIStatusT 	VSL_GetBIOSSelectionConfig(unsigned char *BIOSSwitchPos,  // BIOS_SWITCH_ON | BIOS_SWITCH_OFF
                                                           unsigned char *BIOSOverRide,	  // 0 | 1
                                                           unsigned char *BIOSSel);       // 0 | 1


/* Raw OS agnostic I/O routines - used if you decide to access the ADC/DAC/DIO devices yourself */
VL_OSALIB_API unsigned char	 OSAInput8(unsigned short);

#ifdef WIN32
VL_OSALIB_API unsigned char	readFromFPGA(HANDLE, unsigned short);
#endif  // WIN32
#ifdef linux
VL_OSALIB_API unsigned char	readFromFPGA(int, unsigned short);
#endif // linux

VL_OSALIB_API void			OSAOutput8(unsigned short, unsigned char);
#ifdef WIN32
VL_OSALIB_API void writeToFPGA(HANDLE, unsigned short, unsigned char);
#endif  // WIN32
#ifdef linux
VL_OSALIB_API void writeToFPGA(int, unsigned short, unsigned char);
#endif  // linux
VL_OSALIB_API void			OSAWaitSPIBusyClear();
VL_OSALIB_API void			OSAWaitPCIBusyClear();
VL_OSALIB_API void			OSAWaitADCBusyClear();
VL_OSALIB_API void			OSAWaitPCIADCBusyClear();
VL_OSALIB_API unsigned char	OSADIORegRead (unsigned char Device, unsigned char Address);
VL_OSALIB_API void			OSADIORegWrite(unsigned char Device, unsigned char Address, unsigned char Value);


/* SPI access routine - these routines will do all of the work talking to the SPI/SPX devices for you */
VL_OSALIB_API unsigned char	SPXDIOGetChannelLevel(unsigned char Channel);
VL_OSALIB_API void			SPXDIOSetChannelLevel(unsigned char Channel, unsigned char Level);
VL_OSALIB_API void			SPXDIOSetChannelDirection(unsigned char Channel, unsigned char Direction);
VL_OSALIB_API unsigned char	PCIAXDIOGetChannelLevel(unsigned char Channel);
VL_OSALIB_API void			PCIAXDIOSetChannelLevel(unsigned char Channel, unsigned char Level);
VL_OSALIB_API void			PCIAXDIOSetChannelDirection(unsigned char Channel, unsigned char Direction);
VL_OSALIB_API unsigned char	PCIU2DIOGetChannelLevel(unsigned char Channel);
VL_OSALIB_API void			PCIU2DIOSetChannelLevel(unsigned char Channel, unsigned char Level);
VL_OSALIB_API void			PCIU2DIOSetChannelDirection(unsigned char Channel, unsigned char Direction);

/* Bengal specific functions */
// Bengal DIO functions
VL_OSALIB_API void			BENGALDIOSetChannelDirection(unsigned char Channel, 
														 unsigned char Direction,
														 unsigned char BoardType);
VL_OSALIB_API void			BENGALDIOSetChannelLevel(unsigned char Channel, 
													 unsigned char Level,
													 unsigned char boardType);
VL_OSALIB_API unsigned char	BENGALDIOGetChannelLevel(unsigned char Channel,
													 unsigned char boardType);
VL_OSALIB_API void			BENGALDIOSetChannelPolarity(unsigned char Channel, unsigned char Polarity);
VL_OSALIB_API unsigned char	BENGALDIOGetChannelPolarity(unsigned char Channel);

// Bengal Watchdog Functions
VL_OSALIB_API unsigned char	BENGALWDTStatus();
VL_OSALIB_API unsigned char BENGALWDTGetValue();
VL_OSALIB_API void			BENGALWDTSetValue(unsigned char Value);
VL_OSALIB_API void 			BENGALWDTResetDisable();
VL_OSALIB_API void 			BENGALWDTResetEnable();
VL_OSALIB_API void 			BENGALWDTDisable();
VL_OSALIB_API void 			BENGALWDTEnable();


/* General support structures and routines used to keep your code OS agnostic */
typedef struct BoardNamePrintKey
{
	unsigned char	BoardID;
	char			BoardName[24];
} BOARDNAMEPRINTKEY;

typedef struct BaseBoardInfo
{
	unsigned char	ProductCode;
	char			BoardName[24];
	unsigned char	BoardAttributes;	// Bit 0: Beta (1 => Beta; 0 => Production; 
										// Bit 2: Extended Temp (0 => Standard; 1 => Extented); 
										// Bit 1: Custom (0 => Stand; 1 => Custom); 
										// Bits 3-7 => Rev level.
	short			NumDIOs;
	short			NumTimers;
	unsigned char	WDTSupported;		// 0 => No; 1 => Yes.
    short			NumAnalogIn;		
    short			NumAnalogOut;		
	short			FanSupported;		// 0 => No; 1 => Yes.
	short			NumSerialPorts;
	unsigned char	BIOSInfo;			// Bit 7: BIOS Jumper Position: 1 => Primary; 0 => Secondary;
										// Bit 6: BIOS Override: 1 => BIOS follows BIOS_JMP setting;
										//						 0 => BIOS follows BIOS_SEL setting;
										// Bit 5: BIOS Select:	1 => Primary; 0 => secondary;
} BASE_BOARD_INFO;


BASE_BOARD_INFO	gBoardInfo;		// Storage for the base board information.
BASE_BOARD_INFO gPCIBoardInfo;	// Storage for one PCI board information (need more).
BASE_BOARD_INFO gU2BoardInfo;	// Storage for the U2 board information.
BASE_BOARD_INFO gSouthBoardInfo;// Storage for one south board information. Used for the
								// EPMe-42/EPM-43 initially.


// Helper function to log/print product information for base boards, etc.
VL_OSALIB_API void VSL_GetProductInfo(unsigned long ProductList, 
									  unsigned char *Name, 
									  unsigned char *Attributes, 
									  short *DIOs, 
									  short *Timers, 
									  unsigned char *WDT,
									  short *AIn,
									  short *AOut,
									  short *Serial,
									  short *FanSupport,
									  unsigned char *BIOSInfo);
									  
VL_OSALIB_API void VSL_GetPCIAXProductInfo(unsigned long ProductList, 
										   unsigned char *Name, 
										   unsigned char *Attributes);


// Defines are pulled from VL_OSALib.h
VL_OSALIB_API void OSASleep(unsigned long);
#ifdef WIN32
int U2_cfgRead(unsigned char bReg, unsigned char *pbValue );
int U2_cfgWrite(unsigned char bReg, unsigned char bValue );
#else
_Bool U2_cfgRead( unsigned char bReg, unsigned char *pbValue );
_Bool U2_cfgWrite( unsigned char bReg, unsigned char bValue );
#endif

VL_OSALIB_API double	ADS8668AnalogInput(unsigned char Channel);
VL_OSALIB_API void		ADS8668SetAnalogInputRange(	unsigned char Channel,
													unsigned char Range);

VL_OSALIB_API VL_APIStatusT ADS8668ConfigureAnalogAutoSelectMode(int enable, 
				                                                 int enableOnReset, 
																 unsigned char channels);

VL_OSALIB_API double	OSASPIAnalogInput(unsigned char ClkIdle,
										  unsigned char Irq,
										  unsigned char DataEdge,
										  unsigned char Auto,
										  unsigned char FrameLength,
										  unsigned char ClkSpeed,
										  unsigned char Range,
										  unsigned char Channel,
										  unsigned char ShiftDirection);

VL_OSALIB_API unsigned short	OSAPCIAnalogInput(unsigned char ClkIdle,
										  unsigned char Irq,
										  unsigned char DataEdge,
										  unsigned char Auto,
										  unsigned char FrameLength,
										  unsigned char ClkSpeed,
										  unsigned char Range,
										  unsigned char Channel,
										  unsigned char ShiftDirection);

VL_OSALIB_API void		OSASPIAnalogOutput(unsigned char  ClkIdle,
										   unsigned char  Irq,
										   unsigned char  DataEdge,
										   unsigned char  Auto,
										   unsigned char  FrameLength,
										   unsigned char  ClkSpeed,
										   unsigned char  Channel,
										   unsigned short ADOutput,
										   unsigned char  ShiftDirection);

VL_OSALIB_API unsigned long	long OSAReadMSR(unsigned long MSRNumber);
VL_OSALIB_API void OSAReadCPUID(int *CPUInfo, unsigned long InfoType);
VL_OSALIB_API void GetBoardInfo(unsigned char *pName, 
								unsigned char *pBeta, 
								unsigned char *pExtTemp, 
								unsigned char *pRevLevel, 
								unsigned char *pCustom);

// private VersaLogic routine for debug
VL_OSALIB_API void	OSADIODumpRegs();
#ifdef WIN32
HANDLE FindMPEe_U2(void);
#else
unsigned long FindMPEe_U2(void);
#endif  // WIN32
VL_OSALIB_API unsigned char	GetDriverIRQ(void);

/* Some bit defines for talking to the ADC/DAC/DIO devices */
#define VSL_PCI_DRV			0x02
#define VSL_SPI_DRV			0x01

#define DIO_OUTPUT			0x00
#define DIO_INPUT			0x01
#define DIO_UNKNOWN			0x02

#define DIO_INTERRUPT_HIGH	0x00
#define DIO_INTERRUPT_LOW	0x01
#define DIO_INTERRUPT_UNKNOWN	0x02

// Bengal I/O
#define BENGAL_DIO_INPUT	0x00
#define BENGAL_DIO_OUTPUT	0x01

#define TIMER_TEST_ON		0x40
#define DIO_IODIRA_OFF		0x00
#define DIO_IODIRB_OFF		0x01
#define DIO_GPIOA_OFF		0x12
#define DIO_GPIOB_OFF		0x13
#define DIO_INTFA_OFF		0x0E
#define DIO_INTFB_OFF		0x0F
#define DIO_CAPA_OFF		0x10
#define DIO_CAPB_OFF		0x11
#define DIO_INTCONA_OFF		0x08
#define DIO_INTCONB_OFF		0x09
#define DIO_INTENA_OFF		0x04
#define DIO_INTENB_OFF		0x05
#define DIO_IOCONA_OFF		0x0A
#define DIO_IOCONB_OFF		0x0B

#define DIO_DEV0			0x40
#define DIO_DEV1			0x42

#define DIO_DIRA_OFF		0x00
#define DIO_DIRB_OFF		0x01

#define DIO_WRITE			0x00
#define DIO_READ			0x01

#define DIO_CHANNEL_LOW		0x00
#define DIO_CHANNEL_HIGH	0x01
#define DIO_CHANNEL_UNKOWN	0x02

#define DIO_ICON_OFF		0x0A

#define	DIO_HA_ENABLE		0x08

#define SPI_IDLE_HIGH		0x80
#define SPI_IDLE_LOW		0x00
#define SPI_EDGE_FALLING	0x40
#define SPI_EDGE_RISING		0x00

#define SPI_FRAME_8			0x00
#define SPI_FRAME_16		0x10
#define SPI_FRAME_24		0x20
#define SPI_FRAME_32		0x30

#define SPI_SS_MANUAL		0x08
#define SPI_SS_AUTO			0x00

#define SPI_IAD1857E		0x05
#define SPI_IAD1857E_HIGH	0x03
#define SPI_I23S17			0x06

#define SPI_CS_ADS8668		0x05

#define SPI_DA2634E			0x07
#define SPI_DA2634E_HIGH	0x04
#define SPX_CS_SS0			0x01
#define SPX_CS_SS1			0x02
#define SPX_CS_SS2			0x03
#define SPX_CS_SS3			0x04

#define DIO_CS_SS0			0x40
#define DIO_CS_SS1			0x50
#define DIO_CS_SS2			0x60
#define DIO_CS_SS3			0x70

#define AO_CS_SS0			0x40
#define AO_CS_SS1			0x50
#define AO_CS_SS2			0x60
#define AO_CS_SS3			0x70

#define AI_CS_SS0			0x04
#define AI_CS_SS1			0x05
#define AI_CS_SS2			0x06
#define AI_CS_SS3			0x07

#define SPX_AD7490_SS0		0x0a
#define SPX_AD7490_SS1		0x0b
#define SPX_AD7490_SS2		0x0c
#define SPX_AD7490_SS3		0x0d

#define SPX_AD7927_SS0		0x02
#define SPX_AD7927_SS1		0x03
#define SPX_AD7927_SS2		0x08
#define SPX_AD7927_SS3		0x09

#define SPI_BUSY			0x01
#define ADC_BUSY			0x04
#define SPI_START_CONVRS	0x01

#define AD7490_LOAD_CTRL	0x80
#define AD7490_PWR_NORM		0x03
#define AD7490_RANGE_SINGLE	0x20
#define AD7490_RANGE_DOUBLE	0x00
#define AD7490_CODE_BINARY	0x10

#define SPI_CLK_1MHz		0x00
#define SPI_CLK_2MHz		0x10
#define SPI_CLK_4MHz		0x20
#define SPI_CLK_8MHz		0x30

#define SPI_DIR_LEFT		0x00
#define SPI_DIR_RIGHT		0x04

#define IRQ_CONTROL_OFF		0x03
#define IRQ_STATUS_OFF		0x04
#define IRQ_STATUS_OFF		0x04
#define SPI_CONTROL_OFF		0x08
#define SPI_STATUS_OFF		0x09
#define SPI_DATA0_OFF		0x0A
#define SPI_DATA1_OFF		0x0B
#define SPI_DATA2_OFF		0x0C
#define SPI_DATA3_OFF		0x0D
#define SPI_CUSTOM_OFF		0x0E
#define SPI_ADCONSTAT_OFF	0x0F
#define TMR_TCNTRL		    0x05
#define TMR_TBASEMS		    0x06
#define TMR_TBASELS		    0x07
#define TMR_TISR		    0x04  // See IRQ_STATUS_OFF
#define TMR_TIMR		    0x03  // See IRQ_CONTROL_OFF
#define IRQ3			    0x00  // IRQ3
#define IRQ4			    0x01  // IRQ4
#define IRQ5			    0x02  // IRQ5
#define IRQ10			    0x03  // IRQ10
#define IRQ6			    0x04  // IRQ6
#define IRQ7			    0x05  // IRQ7
#define IRQ9			    0x06  // IRQ9
#define IRQ11			    0x07  // IRQ11
#define ENABLE_INT_GEN_ON   0x01  // Turn on interrupt generation
#define ENABLE_INT_GEN_OFF  0x00  // Turn on interrupt generation

// Bengal DIO Register Offsets
#define BENGAL_DIO_DIR_LOWER	0x14	// Lower and upper 8 bits Direction Register (IN/OUT)
#define BENGAL_DIO_DIR_UPPER	0x15
#define BENGAL_DIO_POL_LOWER	0x16	// Lower and upper 8 bits Polarity Control
#define BENGAL_DIO_POL_UPPER	0x17
#define BENGAL_DIO_OUT_LOWER	0x18	// Lower and upper 8 bits Output Value
#define BENGAL_DIO_OUT_UPPER	0x19
#define BENGAL_DIO_IN_LOWER		0x1A	// Lower and upper 8 bits Input Value
#define BENGAL_DIO_IN_UPPER		0x1B
#define BENGAL_DIO_MASK_LOWER	0x1C	// Lower and upper 8 bits Interrupt Mask Register
#define BENGAL_DIO_MASK_UPPER	0x1D
#define BENGAL_DIO_STAT_LOWER	0x1E	// Lower and upper 8 bits Interrupt Status Register
#define BENGAL_DIO_STAT_UPPER	0x1F
#define BENGAL_DIO_CTRL			0x20	// Digital I/O Control Register

// Bengal Auxiliary Registers
#define BENGAL_AUX_DIR			0x21
#define BENGAL_AUX_POL			0x22
#define BENGAL_AUX_OUT			0x23
#define BENGAL_AUX_IN			0x24	// BATTLOW also included in Bengal register
#define BENGAL_AUX_ICTL			0x25	// Interrupt and Control
#define BENGAL_AUX_STAT			0x26
#define BENGAL_AUX_MODE			0x27

// Bengal Watchdog
#define BENGAL_WTDG_CTRL		0x28
#define BENGAL_WTDG_VAL			0x29
#define ENABLE					0x01
#define DISABLE					0x02
#define WDT_DISABLED			0x00
#define WDT_ENABLED				0x01
#define WDT_UNKNOWN				0xff

// Bengal FANTACH Register
#define	BENGAL_FAN_CTRL			0x2c
#define BENGAL_FAN_TACH_LOWER	0x2e
#define BENGAL_FAN_TACH_UPPER	0x2f

// Bengal Temperature Interrupt Mask, Control and Status
#define BENGAL_TEMP_CTRL			0x30	// IRQ and MASK
#define BENGAL_TEMP_STAT			0x31

// Fox Register Offsets
#define FOX_GPIO_DIR		0x21	// 8 bit Direction Register
#define FOX_GPIO_POL		0x22	// 8 bit Polarity Register
#define FOX_GPIO_OUT		0x23	// 8 bit Output Register
#define FOX_GPIO_IN			0x24	// 8 bit Input Register
#define FOX_GPIO_MASK		0x25	// 8 bit Input Register
#define FOX_GPIO_STAT		0x26	// 8 bit Input Register

// EPU-331X Register Offsets
#define EPU_331X_GPIO_DIR		0x21	// 8 bit Direction Register
#define EPU_331X_GPIO_POL		0x22	// 8 bit Polarity Register
#define EPU_331X_GPIO_OUT		0x23	// 8 bit Output Register
#define EPU_331X_GPIO_IN		0x24	// 8 bit Input Register
#define EPU_331X_GPIO_MASK		0x25	// 8 bit Input Register
#define EPU_331X_GPIO_STAT		0x26	// 8 bit Input Register

// EPME-51N Register Offsets - CPU board
#define EPME_51N_PCR_OFF         0x00    // Offset to PCR 
#define EPME_51N_PSR_OFF         0x01    // Offset to PSR 
#define EPME_51N_SCR_OFF         0x02    // Offset to SCR 
#define EPME_51N_WDT_CTL_OFF     0x28    // Offset to Watchdog control 
#define EPME_51N_WDT_VAL_OFF     0x29    // Offset to Watchdog Value

// EPME-51S Register Offsets - Carrier (I/O) board
#define EPME_51S_FPGA_OFF           0x40                        // Offset to the 51S FPGA
#define EPME_51S_PCR_OFF            EPME_51S_FPGA_OFF + 0x00    // Offset to PCR 
#define EPME_51S_PSR_OFF            EPME_51S_FPGA_OFF + 0x01    // Offset to PSR 
#define EPME_51S_SCR_OFF            EPME_51S_FPGA_OFF + 0x02    // Offset to SCR 
#define EPME_51S_TICR_OFF           EPME_51S_FPGA_OFF + 0x03    // Offset to TICR 
#define EPME_51S_TISR_OFF           EPME_51S_FPGA_OFF + 0x04    // Offset to TISR 
#define EPME_51S_TCR_OFF            EPME_51S_FPGA_OFF + 0x05    // Offset to TCR 
#define EPME_51S_DIO1_GPIO_DIR_OFF   EPME_51S_FPGA_OFF + 0x14    // Offset to Aux GPIO direction
#define EPME_51S_DIO1_GPIO_POL_OFF   EPME_51S_FPGA_OFF + 0x16    // Offset to Aux GPIO direction
#define EPME_51S_DIO1_GPIO_OUT_OFF   EPME_51S_FPGA_OFF + 0x18    // Offset to Aux GPIO direction
#define EPME_51S_DIO1_GPIO_IN_OFF    EPME_51S_FPGA_OFF + 0x1a    // Offset to Aux GPIO direction
#define EPME_51S_DIO1_GPIO_IMASK_OFF EPME_51S_FPGA_OFF + 0x1c    // Offset to Aux ICR register
#define EPME_51S_DIO1_GPIO_ISTAT_OFF EPME_51S_FPGA_OFF + 0x2e    // Offset to Aux GPIO direction
#define EPME_51S_DIO2_GPIO_DIR_OFF   EPME_51S_FPGA_OFF + 0x15    // Offset to Aux GPIO direction
#define EPME_51S_DIO2_GPIO_POL_OFF   EPME_51S_FPGA_OFF + 0x17    // Offset to Aux GPIO direction
#define EPME_51S_DIO2_GPIO_OUT_OFF   EPME_51S_FPGA_OFF + 0x19    // Offset to Aux GPIO direction
#define EPME_51S_DIO2_GPIO_IN_OFF    EPME_51S_FPGA_OFF + 0x1b    // Offset to Aux GPIO direction
#define EPME_51S_DIO2_GPIO_IMASK_OFF EPME_51S_FPGA_OFF + 0x1d    // Offset to Aux ICR register
#define EPME_51S_DIO2_GPIO_ISTAT_OFF EPME_51S_FPGA_OFF + 0x1f    // Offset to Aux GPIO direction
#define EPME_51S_AUX_GPIO_DIR_OFF   EPME_51S_FPGA_OFF + 0x21    // Offset to Aux GPIO direction
#define EPME_51S_AUX_GPIO_POL_OFF   EPME_51S_FPGA_OFF + 0x22    // Offset to Aux GPIO direction
#define EPME_51S_AUX_GPIO_OUT_OFF   EPME_51S_FPGA_OFF + 0x23    // Offset to Aux GPIO direction
#define EPME_51S_AUX_GPIO_IN_OFF    EPME_51S_FPGA_OFF + 0x24    // Offset to Aux GPIO direction
#define EPME_51S_AUX_GPIO_IMASK_OFF EPME_51S_FPGA_OFF + 0x25    // Offset to Aux ICR register
#define EPME_51S_AUX_GPIO_ISTAT_OFF EPME_51S_FPGA_OFF + 0x26    // Offset to Aux GPIO direction
#define EPME_51S_AUX_GPIO_MODE_OFF  EPME_51S_FPGA_OFF + 0x27    // Offset to Aux GPIO direction
#define EPME_51S_SPIMISC_OFF        EPME_51S_FPGA_OFF + 0x0E    // Offset to SPIMISC register
                                                                // This is where SERIRQEN is located.
#define EPME_51S_WDT_CTL_OFF        EPME_51S_FPGA_OFF + 0x28    // Offset to Watchdog control 
#define EPME_51S_WDT_VAL_OFF        EPME_51S_FPGA_OFF + 0x29    // Offset to Watchdog Value

// Boad BIOS information offsets
#define BIOS_INFO			0x2
#define BIOS_JMP_MASK		0x80
#define BIOS_OVERRIDE_MASK	0x40
#define BIOS_SELECT_MASK	0x20
#define BIOS_PRIMARY_BANK	0x0
#define BIOS_SECONDARY_BANK	0x1
#define BIOS_SWITCH_ON      0x0
#define BIOS_SWITCH_OFF     0x1

// Board DIO offsets
#define DIO_AUX_INTERRUPT_MODE_OFF		0x25
#define DIO_AUX_INTERRUPT_STATUS_OFF	0x26
#define DIO_AUXMODE2_INTERRUPT_CTRL_OFF	0x2B

// EPMe-42/EPM-43 information
#define EPM_43_FPGA_OFFSET	0x40
#define EPM_43S_PSTRING		"A-EPM43S"

#define PCI_IRQ_STATUS_OFF	0x8004
#define PCI_GPIO_CFG		0x8005
#define PCI_GPIO_OUT		0x8006
#define PCI_GPIO_IN			0x8007
#define PCI_CONTROL_OFF		0x8008
#define PCI_STATUS_OFF		0x8009
#define PCI_DATA0_OFF		0x800A
#define PCI_DATA1_OFF		0x800B
#define PCI_DATA2_OFF		0x800C
#define PCI_DATA3_OFF		0x800D
#define PCI_CUSTOM_OFF		0x800E
#define PCI_ADCONSTAT_OFF	0x800F

#define SPI_SER_IRQ_EN		0x04

#define AI_RAW				0x00
#define AI_VOLTS			0x01

#define TMR8254_CNT0		0x00
#define TMR8254_CNT1		0x40
#define TMR8254_CNT2		0x80
#define TMR8254_CNTRB		0xc0

#define TMR8254_CNT_LATCH		0x00
#define TMR8254_STS_LATCH_OFF	0x10
#define TMR8254_16BIT			0x10
#define TMR8254_LATCH		0x00
#define TMR8254_RW_LSB		0x10
#define TMR8254_RW_MSB		0x20
#define TMR8254_RW_BOTH		0x30

#define TMR8254_TM0SEL_EX	0x04
#define TMR8254_TM1SEL_EX	0x08

#define TMR8254_TIM0GATE	0x20
#define TMR8254_TIM1GATE	0x40
#define TMR8254_TIM2GATE	0x80
#define TMR8254_TM12MODE16	0x10

#define VL_TIMER_MODE0		0
#define VL_TIMER_MODE1		1
#define VL_TIMER_MODE2		2
#define VL_TIMER_MODE3		3
#define VL_TIMER_MODE4		4
#define VL_TIMER_MODE5		5

#define VL_TIMER0			0
#define VL_TIMER1			1
#define VL_TIMER2			2

#define VL_TIMER_TYPE_EXTERNAL	1
#define VL_TIMER_TYPE_INTERNAL	0

#define TMR8254_BCD			0x01
#define VL_MAX_8254_TIMERS  3
#define VL_MAX_8254_MODES	6

#define FPGA_TMR
#define DTSMASK				0x7f0000
#define TJMAX				100


typedef struct
{
        unsigned char   Address;
        union {
        unsigned char   Data8;
        unsigned short  Data16;
        unsigned long   Data32;
        }Passed;
} VLDriveArg;


// IRQ Signal Numbers.
#define SIG_VL_TIMER 56  // 8254 Timers
#define SIG_VL_DIO   57  // GPIO/DIO


#define IOCTL_VSL_READ_PORT_UCHAR   _IOWR(0xAA, 1, VLDriveArg *)
#define IOCTL_VSL_READ_PORT_USHORT  _IOWR(0xAA, 2, VLDriveArg *)
#define IOCTL_VSL_READ_PORT_ULONG   _IOWR(0xAA, 3, VLDriveArg *)

#define IOCTL_VSL_WRITE_PORT_UCHAR  _IOWR(0xAA, 4, VLDriveArg *)
#define IOCTL_VSL_WRITE_PORT_USHORT _IOWR(0xAA, 5, VLDriveArg *)
#define IOCTL_VSL_WRITE_PORT_ULONG  _IOWR(0xAA, 6, VLDriveArg *)

#define VSL_IOCTL_PASS_TIMER_PID    _IOWR(0xAA, 7, pid_t *)
#define VSL_IOCTL_PASS_DIO_PID      _IOWR(0xAA, 8, pid_t *)
#define VSL_IOCTL_GET_TMR_IRQ       _IOWR(0xAA, 9, long *)

#ifdef __cplusplus
}
#endif

#endif  // _VLAPI_H_
