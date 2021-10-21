/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
@file
The wrapper for ESP_Modbus library communicating with Modbus slaves over RS232/485 (via RTU protocol).
*/

/* _____PROJECT INCLUDES_____________________________________________________ */
#include "ModbusSlave.h"

/* _____GLOBAL VARIABLES_____________________________________________________ */


/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object; initialize it using ModbusSlave::begin().

@ingroup setup
*/
ModbusSlave::ModbusSlave(void)
{
    // Do not perform initialization in constructor.
}

/**
Initialize class object.

Assigns the Modbus slave ID and serial communication parameters.
Call once class has been instantiated.

@param pxCommOpts - pointer to communication options structure
@ingroup setup
*/
esp_err_t ModbusSlave::begin(mb_communication_info_t* pxCommOpts)
{
  _u8MBSlave = pxCommOpts->slave_addr;
  _comm_opts = *pxCommOpts;
  esp_err_t err = ESP_FAIL;

  // Initialization of Modbus controller
  err = mbc_slave_init(MB_PORT_SERIAL_SLAVE, &_slave_handler);
  SLAVE_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller init fail, returns(0x%x).",
                            (uint32_t)err);
  // Setup communication parameters and start stack
  err = mbc_slave_setup((void*)&_comm_opts);
  SLAVE_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);
  err = mbc_slave_start();
  SLAVE_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);
  return err;
}

/**
Add Modbus register area to be accessed by slave

@param regAddress the address of Modbus register in the area using Modicon notation ( example: 40005 - Holding register with offset 5 )
@param instanceAddress - pointer to the data which will store the register data
@param regAreaSize - length of area to store register data
@ingroup setup
*/
esp_err_t ModbusSlave::addRegisterBank(uint16_t regAddress, void* instanceAddress, uint16_t regAreaSize)
{    
    mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
    uint16_t reg_off = 0;
    
    getRegType(regAddress, &reg_area.type, &reg_off);
    
    reg_area.start_offset = regAddress - reg_off; // Offset of register area in Modbus protocol
    reg_area.address = (void*)instanceAddress; // Set pointer to storage instance
    // Set the size of register storage instance
    reg_area.size = (size_t)(regAreaSize << 1);

    return mbc_slave_set_descriptor(reg_area);
}

/**
Waits the event defined in the event mask and returns actual event (register read/write type event)

@param EventMask - event mask to wait for
@ingroup setup
*/
mb_event_group_t ModbusSlave::run(mb_event_group_t EventMask)
{    
    mb_event_group_t event = mbc_slave_check_event(EventMask);
    
    ESP_ERROR_CHECK(mbc_slave_get_param_info(&_reg_info, MODBUS_PAR_INFO_GET_TOUT));
    return event;
}

/**
Modbus get information about accessed Modbus register
The register access information is delivered in method run()
*/
mb_param_info_t ModbusSlave::getRegisterInfo()
{    
    return _reg_info;
}

/**
Check the register access information for register area defined as parameters
The register access information is delivered in method run()
*/
bool ModbusSlave::isBankAccessed(uint16_t regAddress, uint16_t regAreaSize)
{    
    mb_param_type_t reg_type;
    uint16_t base_off = 0;
    mb_param_info_t par_info = _reg_info;
    
    getRegType(regAddress, &reg_type, &base_off);
    mb_event_group_t reg_mask = getRegEventMask(reg_type);
    
    uint16_t offset = regAddress - base_off;
    
    //ESP_LOGI("TEST", "test mask %d", (reg_mask & par_info.type));
    bool result = ((reg_mask & par_info.type) && 
                    (offset >= par_info.mb_offset) && 
                    ((offset + regAreaSize) >= (par_info.mb_offset + par_info.size))) ? true : false;
    //ESP_LOGI("TEST", "Bank accessed type %d, offset %d, par_info.mb_off %d, par_info.size %d, reg_mask %d, par_info.type %d, result=%d", 
                        reg_type, offset, par_info.mb_offset, par_info.size, reg_mask, par_info.type, (uint8_t)result);
    return result;
}

ModbusSlave::~ModbusSlave(void)
{
    ESP_ERROR_CHECK(mbc_slave_destroy());
}

/* _____PRIVATE FUNCTIONS____________________________________________________ */

// This helper temporarily not used
void ModbusSlave::getRegType(uint16_t u16RegAddress, mb_param_type_t* regType, uint16_t* u16BaseAddr)
{
    if (u16RegAddress <= MODBUS_COIL_END) { 
        *regType = MB_PARAM_COIL;
        *u16BaseAddr = MODBUS_COIL_START;
    } else if ((u16RegAddress >= MODBUS_DISC_INPUT_START) && (u16RegAddress <= MODBUS_DISC_INPUT_END)) {
        *regType = MB_PARAM_DISCRETE;
        *u16BaseAddr = MODBUS_DISC_INPUT_START;
    } else if ((u16RegAddress >= MODBUS_INPUT_START) && (u16RegAddress <= MODBUS_INPUT_END)) {
        *regType = MB_PARAM_INPUT;
        *u16BaseAddr = MODBUS_INPUT_START;
    } else if ((u16RegAddress >= MODBUS_HOLD_START) && (u16RegAddress <= MODBUS_HOLD_END)) {
        *regType = MB_PARAM_HOLDING;
        *u16BaseAddr = MODBUS_HOLD_START;
    }
    return;
}

// This helper temporarily not used
mb_event_group_t ModbusSlave::getRegEventMask(mb_param_type_t regType)
{
    mb_event_group_t eventMask = MB_EVENT_NO_EVENTS;
    switch(regType) {
        case MB_PARAM_COIL:
            eventMask = (mb_event_group_t)(MB_EVENT_COILS_RD | MB_EVENT_COILS_WR);
            break;
        case MB_PARAM_DISCRETE:
            eventMask = (mb_event_group_t)(MB_EVENT_DISCRETE_RD);
            break;
        case MB_PARAM_INPUT:
            eventMask = (mb_event_group_t)(MB_EVENT_INPUT_REG_RD);
            break;
        case MB_PARAM_HOLDING:
            eventMask = (mb_event_group_t)(MB_EVENT_HOLDING_REG_RD | MB_EVENT_HOLDING_REG_WR);
            break;
        default:
            eventMask = MB_EVENT_NO_EVENTS;
            break;
    }
    return eventMask;
}
