/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).

@defgroup setup ModbusMaster Object Instantiation/Initialization
@defgroup buffer ModbusMaster Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/
/*

  ModbusMaster.h - Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).

  Library:: ModbusMaster

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

  
#ifndef ModbusMaster_h
#define ModbusMaster_h

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return ret_val; \
    }


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

    esp_err_t ModbusMasterTransaction(uint8_t);
};
#endif

