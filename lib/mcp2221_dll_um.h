//©[2015] Microchip Technology Inc.and its subsidiaries.You may use this software and any derivatives exclusively with Microchip products.
//
//THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
//WARRANTIES OF NON - INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION WITH ANY 
//OTHER PRODUCTS, OR USE IN ANY APPLICATION.
//
//IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER
//RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.TO THE FULLEST EXTENT ALLOWED
//BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID
//DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//
//MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the MCP2221_DLL_UM_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// MCP2221_DLL_UM_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

#ifdef __cplusplus
extern "C"{
#endif


    //for projects importing the .lib, use the MCP2221_LIB preprocessor definition
#ifndef MCP2221_LIB
#ifdef MCP2221_DLL_UM_EXPORTS
#define MCP2221_DLL_UM_API __declspec(dllexport)
#else
#define MCP2221_DLL_UM_API __declspec(dllimport)
#endif
#else 

#define MCP2221_DLL_UM_API
#endif

#define CALLING_CONVENTION __stdcall

    MCP2221_DLL_UM_API void* CALLING_CONVENTION Mcp2221_OpenByIndex(unsigned int VID, unsigned int PID, unsigned int index);
    MCP2221_DLL_UM_API void* CALLING_CONVENTION Mcp2221_OpenBySN(unsigned int VID, unsigned int PID, wchar_t *serialNo);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_Close(void *handle);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_Reset(void *handle);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_CloseAll();


    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetLibraryVersion(wchar_t *version);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetConnectedDevices(unsigned int vid, unsigned int pid, unsigned int *noOfDevs);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetManufacturerDescriptor(void* handle, wchar_t* manufacturerString);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetManufacturerDescriptor(void* handle, wchar_t* manufacturerString);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetProductDescriptor(void* handle, wchar_t* productString);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetProductDescriptor(void* handle, wchar_t* productString);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetSerialNumberDescriptor(void* handle, wchar_t* serialNumber);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetSerialNumberDescriptor(void* handle, wchar_t* serialNumber);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetFactorySerialNumber(void* handle, wchar_t* serialNumber);

    //USB
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetVidPid(void* handle, unsigned int* vid, unsigned int* pid);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetVidPid(void* handle, unsigned int vid, unsigned int pid);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetUsbPowerAttributes(void* handle, unsigned char* powerAttributes, unsigned int* currentReq);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetUsbPowerAttributes(void* handle, unsigned char powerAttributes, unsigned int currentReq);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetSerialNumberEnumerationEnable(void* handle, unsigned char* snEnumEnabled);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetSerialNumberEnumerationEnable(void* handle, unsigned char snEnumEnabled);

    // i2c functions 
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_I2cCancelCurrentTransfer(void* handle);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_I2cRead(void* handle, unsigned int bytesToRead, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char* i2cRxData);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_I2cWrite(void* handle, unsigned int bytesToWrite, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char* i2cTxData);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetAdvancedCommParams(void* handle, unsigned char timeout, unsigned char maxRetries);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetSpeed(void* handle, unsigned int speed);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_I2cWriteNoStop(void* handle, unsigned int bytesToWrite, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char* i2cTxData);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_I2cReadRestart(void* handle, unsigned int bytesToRead, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char* i2cRxData);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_I2cWriteRestart(void* handle, unsigned int bytesToWrite, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char* i2cTxData);


    //smbus
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusWriteByte(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char data);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusReadByte(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char *readByte);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusWriteWord(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char* data);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusReadWord(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char* readData);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusBlockWrite(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char byteCount, unsigned char* data);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusBlockRead(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char byteCount, unsigned char* readData);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusBlockWriteBlockReadProcessCall(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char command, unsigned char writeByteCount, unsigned char* writeData, unsigned char readByteCount, unsigned char* readData);

    //gpio
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetInitialPinValues(void* handle, unsigned char* ledUrxInitVal, unsigned char* ledUtxInitVal, unsigned char* ledI2cInitVal, unsigned char* sspndInitVal, unsigned char* usbCfgInitVal);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetInitialPinValues(void* handle, unsigned char ledUrxInitVal, unsigned char ledUtxInitVal, unsigned char ledI2cInitVal, unsigned char sspndInitVal, unsigned char usbCfgInitVal);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetInterruptEdgeSetting(void* handle, unsigned char whichToGet, unsigned char* interruptPinMode);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetInterruptEdgeSetting(void* handle, unsigned char whichToSet, unsigned char interruptPinMode);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_ClearInterruptPinFlag(void* handle);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetInterruptPinFlag(void* handle, unsigned char* flagValue);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetClockSettings(void* handle, unsigned char whichToGet, unsigned char* dutyCycle, unsigned char* clockDivider);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetClockSettings(void* handle, unsigned char whichToSet, unsigned char dutyCycle, unsigned char clockDivider);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetAdcData(void* handle, unsigned int* adcDataArray);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetAdcVref(void* handle, unsigned char whichToGet, unsigned char* adcVref);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetAdcVref(void* handle, unsigned char whichToSet, unsigned char adcVref);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetDacVref(void* handle, unsigned char whichToGet, unsigned char* dacVref);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetDacVref(void* handle, unsigned char whichToSet, unsigned char dacVref);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetDacValue(void* handle, unsigned char whichToGet, unsigned char* dacValue);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetDacValue(void* handle, unsigned char whichToSet, unsigned char dacValue);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetGpioSettings(void* handle, unsigned char whichToGet, unsigned char* pinFunctions, unsigned char* pinDirections, unsigned char* outputValues);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetGpioSettings(void* handle, unsigned char whichToSet, unsigned char* pinFunctions, unsigned char* pinDirections, unsigned char* outputValues);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetGpioValues(void* handle, unsigned char* gpioValues);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetGpioValues(void* handle, unsigned char* gpioValues);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetGpioDirection(void* handle, unsigned char* gpioDir);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetGpioDirection(void* handle, unsigned char* gpioDir);

    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetSecuritySetting(void* handle, unsigned char* securitySetting);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetSecuritySetting(void* handle, unsigned char securitySetting, char* currentPassword, char* newPassword);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SendPassword(void* handle, char* password);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SetPermanentLock(void* handle);


    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetHwFwRevisions(void* handle, wchar_t* hardwareRevision, wchar_t* firmwareRevision);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_GetLastError();

    MCP2221_DLL_UM_API  int CALLING_CONVENTION Mcp2221_SmbusSendByte(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char data);
    MCP2221_DLL_UM_API int CALLING_CONVENTION Mcp2221_SmbusReceiveByte(void* handle, unsigned char slaveAddress, unsigned char use7bitAddress, unsigned char usePec, unsigned char *readByte);

    /********************************
    Error codes
    *********************************/
#define E_NO_ERR 0
#define E_ERR_UNKOWN_ERROR - 1
#define E_ERR_CMD_FAILED -2
#define E_ERR_INVALID_HANDLE -3
#define E_ERR_INVALID_PARAMETER -4
#define E_ERR_INVALID_PASS -5
#define E_ERR_PASSWORD_LIMIT_REACHED -6
#define E_ERR_FLASH_WRITE_PROTECTED -7
    // null pointer received
#define E_ERR_NULL -10
    // destination string too small
#define E_ERR_DESTINATION_TOO_SMALL -11
#define E_ERR_INPUT_TOO_LARGE -12
#define E_ERR_FLASH_WRITE_FAILED -13
#define E_ERR_MALLOC -14



    //we tried to connect to a device with a non existent index
#define E_ERR_NO_SUCH_INDEX -101
    // no device matching the provided criteria was found
#define E_ERR_DEVICE_NOT_FOUND -103

    // one of the internal buffers of the function was too small
#define E_ERR_INTERNAL_BUFFER_TOO_SMALL -104
    // an error occurred when trying to get the device handle
#define E_ERR_OPEN_DEVICE_ERROR -105
    // connection already opened
#define E_ERR_CONNECTION_ALREADY_OPENED -106 ;

#define E_ERR_CLOSE_FAILED -107


    /******* I2C errors *******/
#define E_ERR_INVALID_SPEED -401
#define E_ERR_SPEED_NOT_SET -402
#define E_ERR_INVALID_BYTE_NUMBER -403
#define E_ERR_INVALID_ADDRESS -404
#define E_ERR_I2C_BUSY -405
    //mcp2221 signaled an error during the i2c read operation
#define E_ERR_I2C_READ_ERROR -406 
#define E_ERR_ADDRESS_NACK -407
#define E_ERR_TIMEOUT -408
#define E_ERR_TOO_MANY_RX_BYTES -409
    //could not copy the data received from the slave into the provided buffer;
#define E_ERR_COPY_RX_DATA_FAILED -410
    // failed to copy the data into the HID buffer
#define E_ERR_COPY_TX_DATA_FAILED -412
    // The i2c engine (inside mcp2221) was already idle. The cancellation command had no effect.
#define E_ERR_NO_EFFECT -411
#define E_ERR_INVALID_PEC -413
    // The slave sent a different value for the block size(byte count) than we expected
#define E_ERR_BLOCK_SIZE_MISMATCH -414


#define E_ERR_RAW_TX_TOO_LARGE -301
#define E_ERR_RAW_TX_COPYFAILED -302
#define E_ERR_RAW_RX_COPYFAILED -303



    /***********************************
    Constants
    ************************************/
#define FLASH_SETTINGS      0
#define RUNTIME_SETTINGS    1

#define NO_CHANGE           0xFF

    //GPIO settings
#define MCP2221_GPFUNC_IO        0

#define MCP2221_GP_SSPND       1
#define MCP2221_GP_CLOCK_OUT   1
#define MCP2221_GP_USBCFG      1
#define MCP2221_GP_LED_I2C     1

#define MCP2221_GP_LED_UART_RX 2
#define MCP2221_GP_ADC         2

#define MCP2221_GP_LED_UART_TX 3
#define MCP2221_GP_DAC         3

#define MCP2221_GP_IOC         4

#define MCP2221_GPDIR_INPUT       1
#define MCP2221_GPDIR_OUTPUT      0

#define MCP2221_GPVAL_HIGH        1
#define MCP2221_GPVAL_LOW         0

#define INTERRUPT_NONE 0
#define INTERRUPT_POSITIVE_EDGE     1
#define INTERRUPT_NEGATIVE_EDGE     2
#define INTERRUPT_BOTH_EDGES        3

#define  VREF_VDD   0
#define  VREF_1024V 1
#define  VREF_2048V 2
#define  VREF_4096V 3

#define MCP2221_USB_BUS     0x80
#define MCP2221_USB_SELF    0x40
#define MCP2221_USB_REMOTE  0x20

#define MCP2221_PASS_ENABLE 1
#define MCP2221_PASS_DISABLE 0
#define MCP2221_PASS_CHANGE 0xff


#ifdef __cplusplus
}
#endif
