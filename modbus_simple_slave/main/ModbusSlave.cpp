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
static const char *TAG = "ModbusSlave";

/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object; initialize it using ModbusSlave::begin().

@ingroup setup
*/
ModbusSlave::ModbusSlave(mb_communication_info_t* pxCommOpts)
{
    void* slave_handle = NULL;
    MB_RETURN_ON_FALSE((pxCommOpts != NULL), ; , TAG,
                            "mb controller initialization fail.");
    _comm_opts = *pxCommOpts;
    _pslave_handle = NULL;
    esp_err_t err = mbc_slave_create_serial(&_comm_opts, &slave_handle);
    MB_RETURN_ON_FALSE((slave_handle != NULL), ; , TAG,
                        "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG, 
                        "mb controller initialization fail, returns(0x%x).", (int)err);
    ESP_LOGW("TEST", "Inst pointer constructor: %p", slave_handle);
    _pslave_handle = slave_handle;
}

void *ModbusSlave::getInstance(void)
{
    //assert(_pslave_handle);
    return _pslave_handle;
}

/**
Initialize class object.

Assigns the Modbus slave ID and serial communication parameters.
Call once class has been instantiated.

@ingroup setup
*/
esp_err_t ModbusSlave::begin()
{
  _u8MBSlave = _comm_opts.ser_opts.uid;
  esp_err_t err = ESP_FAIL;

  // Starts communication object and stack
  err = mbc_slave_start(_pslave_handle);
  MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returns(0x%x).", (int)err);
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
    reg_area.access = MB_ACCESS_RW;

    return mbc_slave_set_descriptor(_pslave_handle, reg_area);
}

/**
Waits the event defined in the event mask and returns actual event (register read/write type event)

@param EventMask - event mask to wait for
@ingroup setup
*/
mb_event_group_t ModbusSlave::run(mb_event_group_t EventMask)
{    
    MB_RETURN_ON_FALSE((_pslave_handle != NULL), MB_EVENT_NO_EVENTS, TAG,
                            "mb controller initialization fail.");
    (void)mbc_slave_check_event(_pslave_handle, EventMask);
    ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(_pslave_handle, &_reg_info, MODBUS_PAR_INFO_GET_TOUT));
    return _reg_info.type;
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
    ESP_LOGD("TEST", "Bank accessed type %d, offset %d, par_info.mb_off %d, par_info.size %d, reg_mask %x, par_info.type %x, result=%d", 
                       reg_type, offset, par_info.mb_offset, par_info.size, (int)reg_mask, (int)par_info.type, (int)result);
    return result;
}

ModbusSlave::~ModbusSlave(void)
{
    MB_RETURN_ON_FALSE((_pslave_handle != NULL), ; , TAG,
                            "mb controller initialization fail.");
    esp_err_t err = mbc_slave_stop(_pslave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mb controller stop fail or already stopped, returns(0x%x).", (int)err);
    }
    ESP_ERROR_CHECK(mbc_slave_delete(_pslave_handle));
    _pslave_handle = NULL;
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
