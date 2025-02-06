/**
@file
The wrapper class for Modbus Slave for communication over RS232/485 (via RTU or ASCII protocol).

@defgroup setup ModbusSlave Object Instantiation/Initialization
@defgroup buffer ModbusSlave register bank management
*/
/*

  ModbusSlave.h - The wrapper class for Modbus Serial Slave to communicate over RS232/485 (via RTU protocol).

  Library:: ModbusSlave

  Copyright:: 2009-2016 Espressif Systems

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

  
#ifndef ModbusSlave_h
#define ModbusSlave_h

// Defines for register areas
#define MODBUS_COIL_START 1
#define MODBUS_COIL_END 10000
#define MODBUS_DISC_INPUT_START MODBUS_COIL_START + MODBUS_COIL_END
#define MODBUS_DISC_INPUT_END 20000
#define MODBUS_INPUT_START 30001
#define MODBUS_INPUT_END 39999
#define MODBUS_HOLD_START 40001
#define MODBUS_HOLD_END 49999

#define MODBUS_PAR_INFO_GET_TOUT pdMS_TO_TICKS(10)

/* _____STANDARD INCLUDES____________________________________________________ */
#include <stdint.h>
#include "esp_log.h"

/* _____UTILITY MACROS_______________________________________________________ */
#define CRITICAL_SECT(inst) for (int st = mbc_slave_lock(inst); (st == 0); mbc_slave_unlock(inst), st = 1)


/* _____PROJECT INCLUDES_____________________________________________________ */
#include "mbcontroller.h"   // include file for common Modbus types

/* _____CLASS DEFINITIONS____________________________________________________ */

class ModbusSlave
{
  public:
    ModbusSlave(mb_communication_info_t *);
    virtual ~ModbusSlave();
    void *getInstance(void);

    esp_err_t begin();
    esp_err_t addRegisterBank(uint16_t regAddress, void* instanceAddress, uint16_t regAreaSize);
    mb_event_group_t run(mb_event_group_t);
    mb_param_info_t getRegisterInfo(void);
    bool isBankAccessed(uint16_t, uint16_t);

    
  private:
    uint8_t _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
    mb_communication_info_t _comm_opts;                         ///< Modbus communication options
    void* _pslave_handle;                                       ///< Modbus slave handle
    mb_param_info_t _reg_info;                                  ///< Modbus slave accessed register information
    mb_event_group_t _reg_event;                                ///< Modbus slave register access event
    
    // The helper functions for Modbus Slave
    void getRegType(uint16_t, mb_param_type_t*, uint16_t*);
    mb_event_group_t getRegEventMask(mb_param_type_t);
};

#endif

