/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
@file
Wrapper class for ESP_Modbus API to access Modbus slaves over RS232/485 (via RTU protocol).

@defgroup setup ModbusMaster Object Instantiation/Initialization
@defgroup buffer ModbusMaster Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/

#ifndef ModbusMaster_h
#define ModbusMaster_h

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return ret_val; \
    }
    
// Defines for register areas
#define MODBUS_COIL_START 1
#define MODBUS_COIL_END 10000
#define MODBUS_DISC_INPUT_START MODBUS_COIL_START + MODBUS_COIL_END
#define MODBUS_DISC_INPUT_END 20000
#define MODBUS_INPUT_START 30001
#define MODBUS_INPUT_END 39999
#define MODBUS_HOLD_START 40001
#define MODBUS_HOLD_END 49999

/**
@def __MODBUSMASTER_DEBUG__ (0)
Set to 1 to enable debugging features within class:
  - PIN A cycles for each byte read in the Modbus response
  - PIN B cycles for each millisecond timeout during the Modbus response
*/

/* _____STANDARD INCLUDES____________________________________________________ */
#include <stdint.h>
#include "esp_log.h"

#include "mbcontroller.h"   // include file for common Modbus types


/* _____UTILITY MACROS_______________________________________________________ */


/* _____PROJECT INCLUDES_____________________________________________________ */


/* _____CLASS DEFINITIONS____________________________________________________ */

class ModbusMaster
{
  public:
    ModbusMaster();
    ~ModbusMaster();
    
    esp_err_t begin(uint8_t, mb_communication_info_t*);

    esp_err_t  readCoils(uint16_t, uint16_t, void*);
    esp_err_t  readDiscreteInputs(uint16_t, uint16_t, void*);
    esp_err_t  readHoldingRegisters(uint16_t, uint16_t, void*);
    esp_err_t  readInputRegisters(uint16_t, uint8_t, void*);
    esp_err_t  writeSingleCoil(uint16_t, uint8_t);
    esp_err_t  writeSingleRegister(uint16_t, uint16_t);
    esp_err_t  writeMultipleCoils(uint16_t, uint16_t, void*);
    //esp_err_t  writeMultipleCoils();
    esp_err_t  writeMultipleRegisters(uint16_t, uint16_t, void*);
    //esp_err_t  writeMultipleRegisters();
    //esp_err_t  maskWriteRegister(uint16_t, uint16_t, uint16_t);
    esp_err_t  readWriteMultipleRegisters(uint16_t, uint16_t, uint16_t, uint16_t);
    esp_err_t  readWriteMultipleRegisters(uint16_t, uint16_t);

  private:
    uint8_t _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
    mb_communication_info_t _comm_opts;                         ///< Modbus communication options
    mb_param_request_t _param_request;                          ///< Modbus request structure
    
    // Modbus function codes for bit access
    static const uint8_t ku8MBReadCoils                  = 0x01; ///< Modbus function 0x01 Read Coils
    static const uint8_t ku8MBReadDiscreteInputs         = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
    static const uint8_t ku8MBWriteSingleCoil            = 0x05; ///< Modbus function 0x05 Write Single Coil
    static const uint8_t ku8MBWriteMultipleCoils         = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

    // Modbus function codes for 16 bit access
    static const uint8_t ku8MBReadHoldingRegisters       = 0x03; ///< Modbus function 0x03 Read Holding Registers
    static const uint8_t ku8MBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
    static const uint8_t ku8MBWriteSingleRegister        = 0x06; ///< Modbus function 0x06 Write Single Register
    static const uint8_t ku8MBWriteMultipleRegisters     = 0x10; ///< Modbus function 0x10 Write Multiple Registers
    static const uint8_t ku8MBMaskWriteRegister          = 0x16; ///< Modbus function 0x16 Mask Write Register
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers
    
    // Modbus timeout [milliseconds]
    static const uint16_t ku16MBResponseTimeout          = 2000; ///< Modbus timeout [milliseconds]
    void getRegType(uint16_t, mb_param_type_t*, uint16_t*);
};
#endif

