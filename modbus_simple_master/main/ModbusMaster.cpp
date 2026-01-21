/*
 * SPDX-FileCopyrightText: 2021 - 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
@file
The wrapper for ESP_Modbus library communicating with Modbus slaves over RS232/485 (via RTU protocol).
*/

/* _____PROJECT INCLUDES_____________________________________________________ */
#include "ModbusMaster.h"

/* _____GLOBAL VARIABLES_____________________________________________________ */
static const char *TAG = "ModbusMaster";

/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object; initialize it using ModbusMaster::begin().
@param pCommOpts a pointer to communication options

@ingroup setup
*/
ModbusMaster::ModbusMaster(mb_communication_info_t *pCommOpts)
{
    void* master_handle = NULL;
    MB_RETURN_ON_FALSE((pCommOpts != NULL), ; , TAG,
                            "mb controller initialization fail.");
    _comm_opts = *pCommOpts;
    _pmaster_handle = NULL;
    esp_err_t err = mbc_master_create_serial(&_comm_opts, &master_handle);
    MB_RETURN_ON_FALSE((master_handle != NULL), ; , TAG,
                        "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG, 
                        "mb controller initialization fail, returns(0x%x).", (int)err);
    _pmaster_handle = master_handle;
}

/**
Initialize Master class object to interract with defined slave UID.

Assigns the Modbus slave ID.
Call once class has been instantiated, typically within start of communication.

@param slave Modbus slave ID (1..247)
@ingroup setup
*/
esp_err_t ModbusMaster::begin(uint8_t slave)
{
    _u8MBSlave = slave;
    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");
    esp_err_t err = mbc_master_start(_pmaster_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), err , TAG,
                        "mb controller start fail, returns(0x%x).", (int)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

/**
Stops Modbus Master object

Assigns the Modbus slave ID and serial port.
Call once class has been instantiated, typically within setup().

@ingroup setup
*/
esp_err_t ModbusMaster::end()
{
    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");
    esp_err_t err = mbc_master_stop(_pmaster_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), err , TAG,
                        "mb controller stop fail, returns(0x%x).", (int)err);
    _u8MBSlave = 0;
    return err;
}

/**
Modbus function 0x01 Read Coils.

This function code is used to read from 1 to 2000 contiguous status of 
coils in a remote device. The request specifies the starting address, 
i.e. the address of the first coil specified, and the number of coils. 
Coils are addressed starting at zero.

The coils in the response buffer are packed as one coil per bit of the 
data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first 
data word contains the output addressed in the query. The other coils 
follow toward the high order end of this word and from low order to high 
order in subsequent words.

If the returned quantity is not a multiple of sixteen, the remaining 
bits in the final data word will be padded with zeros (toward the high 
order end of the word).

@param u16ReadAddress address of first coil (00001..9999)
@param u16BitQty quantity of coils to read (1..2000, enforced by remote device)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup discrete
*/
esp_err_t ModbusMaster::readCoils(uint16_t u16ReadAddress, uint16_t u16BitQty, void* pvBufPtr)
{
    mb_param_type_t reg_type;
    uint16_t reg_off = 0;
    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16ReadAddress, &reg_type, &reg_off);
    MB_RETURN_ON_FALSE((reg_type == MB_PARAM_COIL), ESP_ERR_INVALID_ARG , TAG, 
                            "Incorrect coil address.");

    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBReadCoils;
    _param_request.reg_start = (u16ReadAddress - reg_off);
    _param_request.reg_size = u16BitQty;
    return mbc_master_send_request(_pmaster_handle, &_param_request, pvBufPtr);
}

/**
Modbus function 0x02 Read Discrete Inputs.

This function code is used to read from 1 to 2000 contiguous status of 
discrete inputs in a remote device. The request specifies the starting 
address, i.e. the address of the first input specified, and the number 
of inputs. Discrete inputs are addressed starting at zero.

The discrete inputs in the response buffer are packed as one input per 
bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of 
the first data word contains the input addressed in the query. The other 
inputs follow toward the high order end of this word, and from low order 
to high order in subsequent words.

If the returned quantity is not a multiple of sixteen, the remaining 
bits in the final data word will be padded with zeros (toward the high 
order end of the word).

@param u16ReadAddress address of first discrete input (10001 .. 20000)
@param u16BitQty quantity of discrete inputs to read (1..2000, enforced by remote device)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup discrete
*/
esp_err_t ModbusMaster::readDiscreteInputs(uint16_t u16ReadAddress, uint16_t u16BitQty, void* pvBufPtr)
{
    mb_param_type_t reg_type;
    uint16_t reg_off = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16ReadAddress, &reg_type, &reg_off);
    MB_RETURN_ON_FALSE((reg_type == MB_PARAM_DISCRETE), ESP_ERR_INVALID_ARG , TAG, 
                        "Incorrect discrete address.");
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBReadDiscreteInputs;
    _param_request.reg_start = (u16ReadAddress - reg_off);
    _param_request.reg_size = u16BitQty;
    return mbc_master_send_request(_pmaster_handle, &_param_request, pvBufPtr);
}


/**
Modbus function 0x03 Read Holding Registers.

This function code is used to read the contents of a contiguous block of 
holding registers in a remote device. The request specifies the starting 
register address and the number of registers. Registers are addressed 
starting at zero.

The register data in the response buffer is packed as one word per 
register.

@param u16ReadAddress address of the first holding register (40001..49999)
@param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup register
*/
esp_err_t ModbusMaster::readHoldingRegisters(uint16_t u16ReadAddress, uint16_t u16ReadQty, void* pvBufPtr)
{
    mb_param_type_t regType;
    uint16_t u16Regbase = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16ReadAddress, &regType, &u16Regbase);
    MB_RETURN_ON_FALSE((regType == MB_PARAM_HOLDING), ESP_ERR_INVALID_ARG , TAG,
                        "Incorrect register address %d.", u16ReadAddress);
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBReadHoldingRegisters;
    _param_request.reg_start = (u16ReadAddress - u16Regbase);
    _param_request.reg_size = u16ReadQty;
    return mbc_master_send_request(_pmaster_handle, &_param_request, pvBufPtr);
}


/**
Modbus function 0x04 Read Input Registers.

This function code is used to read from 1 to 125 contiguous input 
registers in a remote device. The request specifies the starting 
register address and the number of registers. Registers are addressed 
starting at zero.

The register data in the response buffer is packed as one word per 
register.

@param u16ReadAddress address of the first input register (30001..39999)
@param u16ReadQty quantity of input registers to read (1..125, enforced by remote device)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup register
*/
esp_err_t ModbusMaster::readInputRegisters(uint16_t u16ReadAddress, uint8_t u16ReadQty, void* pvBufPtr)
{
    mb_param_type_t regType;
    uint16_t u16Regbase = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16ReadAddress, &regType, &u16Regbase);
    MB_RETURN_ON_FALSE((regType == MB_PARAM_INPUT), ESP_ERR_INVALID_ARG , TAG,
                        "Incorrect register address %d.", u16ReadAddress);
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBReadInputRegisters;
    _param_request.reg_start = (u16ReadAddress - u16Regbase);
    _param_request.reg_size = u16ReadQty;
    return mbc_master_send_request(_pmaster_handle, &_param_request, pvBufPtr);
}


/**
Modbus function 0x05 Write Single Coil.

This function code is used to write a single output to either ON or OFF 
in a remote device. The requested ON/OFF state is specified by a 
constant in the state field. A non-zero value requests the output to be 
ON and a value of 0 requests it to be OFF. The request specifies the 
address of the coil to be forced. Coils are addressed starting at zero.

@param u16WriteAddress address of the coil (00001..09999)
@param u8State 0=OFF, non-zero=ON (0x00..0xFF)
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup discrete
*/
esp_err_t ModbusMaster::writeSingleCoil(uint16_t u16WriteAddress, uint8_t u8State)
{
    mb_param_type_t regType;
    uint16_t u16Regbase = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16WriteAddress, &regType, &u16Regbase);
    MB_RETURN_ON_FALSE((regType == MB_PARAM_COIL), ESP_ERR_INVALID_ARG , TAG,
                    "Incorrect coil address %d.", u16WriteAddress);
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBWriteSingleCoil;
    _param_request.reg_start = (u16WriteAddress - u16Regbase);
    _param_request.reg_size = 1;
    uint16_t u16StateTemp = (u8State ? 0xFF00 : 0x0000);
    return mbc_master_send_request(_pmaster_handle, &_param_request, &u16StateTemp);
}


/**
Modbus function 0x06 Write Single Register.

This function code is used to write a single holding register in a 
remote device. The request specifies the address of the register to be 
written. Registers are addressed starting at zero.

@param u16WriteAddress address of the holding register (40001..49999)
@param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup register
*/
esp_err_t ModbusMaster::writeSingleRegister(uint16_t u16WriteAddress, uint16_t u16WriteValue)
{
    mb_param_type_t regType;
    uint16_t u16Regbase = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16WriteAddress, &regType, &u16Regbase);
    MB_RETURN_ON_FALSE((regType == MB_PARAM_HOLDING), ESP_ERR_INVALID_ARG , TAG,
                    "Incorrect holding address %d.", u16WriteAddress);
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBWriteMultipleRegisters;
    _param_request.reg_start = (u16WriteAddress - u16Regbase);
    _param_request.reg_size = 1;
    uint16_t u16WriteValue_temp = u16WriteValue;
    return mbc_master_send_request(_pmaster_handle, &_param_request, &u16WriteValue_temp);
}


/**
Modbus function 0x0F Write Multiple Coils.

This function code is used to force each coil in a sequence of coils to 
either ON or OFF in a remote device. The request specifies the coil 
references to be forced. Coils are addressed starting at zero.

The requested ON/OFF states are specified by contents of the transmit 
buffer. A logical '1' in a bit position of the buffer requests the 
corresponding output to be ON. A logical '0' requests it to be OFF.

@param u16WriteAddress address of the first coil (00001..09999)
@param u16BitQty quantity of coils to write (1..2000, enforced by remote device)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup discrete
*/
esp_err_t ModbusMaster::writeMultipleCoils(uint16_t u16WriteAddress, uint16_t u16BitQty, void* pvBufPtr)
{
    mb_param_type_t regType;
    uint16_t u16Regbase = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16WriteAddress, &regType, &u16Regbase);
    MB_RETURN_ON_FALSE((regType == MB_PARAM_COIL), ESP_ERR_INVALID_ARG , TAG,
                        "Incorrect coild register address %d.", u16WriteAddress);
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBWriteMultipleCoils;
    _param_request.reg_start = (u16WriteAddress - u16Regbase);
    _param_request.reg_size = u16BitQty;
    return mbc_master_send_request(_pmaster_handle, &_param_request, pvBufPtr);
}

/**
Modbus function 0x10 Write Multiple Registers.

This function code is used to write a block of contiguous registers (1 
to 123 registers) in a remote device.

The requested written values are specified in the transmit buffer. Data 
is packed as one word per register.

@param u16WriteAddress address of the holding register (40001..49999)
@param u16WriteQty quantity of holding registers to write (1..123, enforced by remote device)
@param pvBufPtr - pointer to data buffer
@return ESP_OK on success; See ESP_Modbus errors for more information
@ingroup register
*/
esp_err_t ModbusMaster::writeMultipleRegisters(uint16_t u16WriteAddress, uint16_t u16WriteQty, void* pvBufPtr)
{
    mb_param_type_t regType;
    uint16_t u16Regbase = 0;

    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ESP_ERR_INVALID_ARG , TAG,
                            "mb controller initialization fail.");

    getRegType(u16WriteAddress, &regType, &u16Regbase);
    MB_RETURN_ON_FALSE((regType == MB_PARAM_HOLDING), ESP_ERR_INVALID_ARG , TAG,
                    "Incorrect holding register address %d.", u16WriteAddress);
    
    _param_request.slave_addr = _u8MBSlave;
    _param_request.command = ku8MBWriteMultipleRegisters;
    _param_request.reg_start = (u16WriteAddress - u16Regbase);
    _param_request.reg_size = u16WriteQty;
    return mbc_master_send_request(_pmaster_handle, &_param_request, pvBufPtr);
}

/**
Modbus function 0x17 Read Write Multiple Registers.

This function code performs a combination of one read operation and one 
write operation in a single MODBUS transaction. The write operation is 
performed before the read. Holding registers are addressed starting at 
zero.

The request specifies the starting address and number of holding 
registers to be read as well as the starting address, and the number of 
holding registers. The data to be written is specified in the transmit 
buffer.

@param u16ReadAddress address of the first holding register (30001..39999)
@param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
@param u16WriteAddress address of the first holding register (0x0000..0xFFFF)
@param u16WriteQty quantity of holding registers to write (1..121, enforced by remote device)
@return 0 on success; exception number on failure
@ingroup register
*/
esp_err_t ModbusMaster::readWriteMultipleRegisters(uint16_t u16ReadAddress,
  uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
    ESP_LOGE(TAG, "Temporary unsupported command %d.", ku8MBReadWriteMultipleRegisters);
    return ESP_FAIL;
}

esp_err_t ModbusMaster::readWriteMultipleRegisters(uint16_t u16ReadAddress,
  uint16_t u16ReadQty)
{
    ESP_LOGE(TAG, "Temporarily unsupported command %d.", ku8MBReadWriteMultipleRegisters);
    return ESP_FAIL;
}

ModbusMaster::~ModbusMaster(void)
{
    MB_RETURN_ON_FALSE((_pmaster_handle != NULL), ; , TAG,
                            "mb controller initialization fail.");
    // The delete will check and stop object if needed
    ESP_ERROR_CHECK(mbc_master_delete(_pmaster_handle));
    _pmaster_handle = NULL;
}

/* _____PRIVATE FUNCTIONS____________________________________________________ */

void ModbusMaster::getRegType(uint16_t u16RegAddress, mb_param_type_t* regType, uint16_t* u16BaseAddr)
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
