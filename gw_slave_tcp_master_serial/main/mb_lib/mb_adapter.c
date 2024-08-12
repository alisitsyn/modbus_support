/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_timer.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

// #include "mb_common.h"
// #include "esp_modbus_common.h"
// #include "mbc_slave.h"

// #include "mb_common.h"
// #include "port_common.h"
// #include "mb_config.h"
// #include "port_serial_common.h"
// #include "port_tcp_common.h"
// #include "port_adapter.h"

#include "mb_adapter.h"
#include "mb.h"
#include "mb_m.h"
#include "esp_modbus_master.h"

#ifdef __cplusplus
extern "C" {
#endif

static const char *TAG = "mb_adapter";

// Below are function wrappers to substitute actual port object with the adapter object for test purpose

#define MB_MBAP_LEN         (7)
#define MB_PDU_SZ           (5)

#define MB_TCP_MBAP_GET_FIELD(buffer, field) ((uint16_t)((buffer[field] << 8U) | buffer[field + 1]))
#define MB_TCP_MBAP_SET_FIELD(buffer, field, val) { \
    buffer[(field)] = (uint8_t)((val) >> 8U);       \
    buffer[(field) + 1] = (uint8_t)((val) & 0xFF);  \
}

#define MB_GET_USH_FIELD(pbuf, offset) (__extension__(                                  \
{                                                                                       \
    ((uint16_t)((MB_GET_ELEM(pbuf, offset) << 8U) | MB_GET_ELEM(pbuf, offset + 1)));    \
}                                                                                       \
))

#define MB_SET_USH_FIELD(pbuf, offset, val) (__extension__(     \
{                                                               \
    MB_GET_ELEM(pbuf, offset) = (uint8_t)((val) >> 8U);         \
    MB_GET_ELEM(pbuf, offset + 1) = (uint8_t)((val) & 0xFF);    \
    (*(uint16_t *)(&MB_GET_ELEM(pbuf, offset)));                \
}                                                               \
))

#define EACH_ITEM(array, length) \
(typeof(*(array)) *pitem = (array); (pitem < &((array)[length])); pitem++)

#define MB_SWAP_BUFFER(pinst, size) (__extension__(         \
{                                                           \
    for EACH_ITEM(pinst, size) {                            \
        (*pitem) = MB_GET_USH_FIELD(((uint8_t*)pitem), 0);  \
    }                                                       \
    (pinst);                                                \
}                                                           \
))

#define MB_GET_ELEM(buf, offset) (__extension__(    \
{                                                   \
   (*((uint8_t *)(buf + offset)));                  \
}                                                   \
))

#define MB_DEVICE_MAPPING 0

uint8_t *mb_restore_mbap_ptr(uint8_t *pdata, uint16_t reg_offs, uint16_t reg_cnt, eMBRegisterMode mode, mb_param_request_t *preq, uint16_t *plen)
{
    uint8_t *pbuf = (uint8_t *)pdata;
    mb_param_request_t request = { 0 };
    uint8_t *ret_ptr = NULL;
    uint16_t length = 0;
    
    // Todo: other commands handling can be added later
    switch (mode) {
            case MB_REG_READ:
                    // Read input registers
                    // The command handling to be added here
                    
                    // Read holding registers
                    // TID   PID   LEN   UID FC OFF    RLEN  B1 B2
                    // 00 01 00 00 00 05 01  03 00 01  00 01 00 00
                    // Note: callback func updates buffer for answer
                    // TID   PID   LEN   UID FC CNT B1 B2
                    // 00 01 00 00 00 05 01  03 02  01  00
                    if ((MB_GET_ELEM(pbuf,-2) == 0x03) && (MB_GET_USH_FIELD(pbuf, -7) == 0x0000)) {
                        ret_ptr = &MB_GET_ELEM(pbuf, -9);
#if MB_DEVICE_MAPPING
                        // Note: Need to restore the offset set by function handler for correct mapping
                        MB_SET_USH_FIELD(pbuf, -1, reg_offs);
#endif
                        request.slave_addr = MB_GET_ELEM(pbuf, -3);
                        request.command = MB_GET_ELEM(pbuf, -2);
                        request.reg_start = reg_offs;
                        request.reg_size = reg_cnt;
                        length = MB_MBAP_LEN + MB_PDU_SZ;
                    } 
                break;
            case MB_REG_WRITE:
                    // Write Holding register
                    // TID   PID   LEN   UID FC OFFS  RLEN  B1 B2
                    // 00 01 00 00 00 01 01  06 00 01 00 01 00 01
                    if ((MB_GET_ELEM(pbuf, -5) == 0x06) && (MB_GET_USH_FIELD(pbuf, -10) == 0x0000)) {
                        ret_ptr = &MB_GET_ELEM(pbuf, -12);
                        request.slave_addr = MB_GET_ELEM(pbuf, -6);
                        request.command = MB_GET_ELEM(pbuf, -5);
                        request.reg_start = MB_GET_USH_FIELD(pbuf, -4);
                        request.reg_size = MB_GET_USH_FIELD(pbuf, -2);
                        length =  MB_MBAP_LEN + MB_PDU_SZ + (reg_cnt << 1);
                    } 
                    // Write Multiple Holding registers
                    // TID   PID   LEN   UID FC OFFS  RLEN  BCNT B1 B2
                    // 00 01 00 00 00 01 01  10 00 01 00 01 02   00 00
                    // 41 29 00 00 00 0b 01  10 00 01 00 02 04   00 00 00 00
                    else if((MB_GET_ELEM(pbuf, -6) == 0x10)
                            && (MB_GET_USH_FIELD(pbuf, -11) == 0x0000)
                            && ((MB_GET_USH_FIELD(pbuf, -3) << 1) == MB_GET_ELEM(pbuf, -1))
                            ) {
                        ret_ptr = &MB_GET_ELEM(pbuf, -13);
                        request.slave_addr = MB_GET_ELEM(pbuf, -7);
                        request.command = MB_GET_ELEM(pbuf, -6);
                        request.reg_start = MB_GET_USH_FIELD(pbuf, -5);
                        request.reg_size = MB_GET_USH_FIELD(pbuf, -3);
                        length =  MB_MBAP_LEN + MB_PDU_SZ + 1 + MB_GET_ELEM(pbuf, -1);
                    }
                break;
            default:
                break;
    }
    if (ret_ptr) {
        if (preq) *preq = request;
        if (plen) *plen = length;
    }
    return ret_ptr;
}

eMBErrorCode mb_master_transfer_request(uint8_t *preg_data, uint16_t reg_offs, uint16_t reg_cnt, eMBRegisterMode mode)
{
    uint16_t len_bytes = 0;
    mb_param_request_t request = { 0 };
    eMBErrorCode ret = MB_EILLSTATE;
    
    // Note: Other safer approaches are possible for this funtion
    uint8_t *mbap_ptr = mb_restore_mbap_ptr(preg_data, reg_offs, reg_cnt, mode, &request, &len_bytes);
    if (mbap_ptr) {
        ESP_LOG_BUFFER_HEX_LEVEL("GW_TCP_MBAP", (void*)mbap_ptr, len_bytes, ESP_LOG_WARN);
        // Change the own address of master to let it handle response from slave for the UID
        vMBMasterSetDestAddress(request.slave_addr);
        // Todo: Need to control timeout correctly to prevent desynchronization issues
        // between TCP slave and SERIAL master
        esp_err_t err = mbc_master_send_request(&request, (void *)preg_data);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "Received response from serial slave UID: %d.", request.slave_addr);
            // need to swap the buffer bytes (MB network format)
            //ESP_LOG_BUFFER_HEX_LEVEL("GW_SER_BUF1", (void*)preg_data, (reg_cnt << 1), ESP_LOG_WARN);
            MB_SWAP_BUFFER((uint16_t*)preg_data, reg_cnt);
            ESP_LOG_BUFFER_HEX_LEVEL("GW_SER_DATA", (void*)preg_data, (reg_cnt << 1), ESP_LOG_WARN);
            ret = MB_ENOERR;
        }
    }
    return ret;
}

eMBErrorCode __wrap_eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNRegs);
   
    eMBErrorCode xRet = mb_master_transfer_request(pucRegBuffer, usAddress, usNRegs, eMode);
    // In any case try to map the data to the device registers (can be disabled)
#if MB_DEVICE_MAPPING
    xRet = __real_eMBRegHoldingCB(pucRegBuffer, usAddress, usNRegs, eMode);
#endif
    return xRet;
}

eMBErrorCode __wrap_eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    //
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNRegs);
    eMBErrorCode xRet = mb_master_transfer_request(pucRegBuffer, usAddress, usNRegs, MB_REG_READ);

#if MB_DEVICE_MAPPING
    xRet = __real_eMBRegInputCB(pucRegBuffer, usAddress, usNRegs);
#endif

    return xRet;
}

eMBErrorCode __wrap_eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNDiscrete);
    return __real_eMBRegDiscreteCB(pucRegBuffer, usAddress, usNDiscrete);
}

eMBErrorCode __wrap_eMBRegCoilsCB(UCHAR* pucRegBuffer, USHORT usAddress,
                            USHORT usNCoils, eMBRegisterMode eMode)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNCoils);
    return __real_eMBRegCoilsCB(pucRegBuffer, usAddress, usNCoils, eMode);
}

eMBErrorCode __wrap_eMBRegInputCBSerialMaster(UCHAR * pucRegBuffer, USHORT usAddress,
                                USHORT usNRegs)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNRegs);
    return __real_eMBRegInputCBSerialMaster(pucRegBuffer, usAddress, usNRegs);;
}

eMBErrorCode __wrap_eMBRegHoldingCBSerialMaster(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNRegs, eMBRegisterMode eMode)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNRegs);
    return __real_eMBRegHoldingCBSerialMaster(pucRegBuffer, usAddress, usNRegs, eMode);
}

eMBErrorCode __wrap_eMBRegCoilsCBSerialMaster(UCHAR* pucRegBuffer, USHORT usAddress,
        USHORT usNCoils, eMBRegisterMode eMode)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNCoils);
    return  __real_eMBRegCoilsCBSerialMaster(pucRegBuffer, usAddress, usNCoils, eMode);
}

eMBErrorCode __wrap_eMBRegDiscreteCBSerialMaster(UCHAR * pucRegBuffer, USHORT usAddress,
                            USHORT usNDiscrete)
{
    ESP_LOGW(TAG, "callback %s, %p, %u, %u", __func__, pucRegBuffer, usAddress, usNDiscrete);
    return __real_eMBRegDiscreteCBSerialMaster(pucRegBuffer, usAddress, usNDiscrete);
}

#ifdef __cplusplus
}
#endif