/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#pragma once

#include <sdkconfig.h>
#include "esp_log.h"

#include "mb.h"

extern eMBErrorCode __real_eMBRegInputCBSerialMaster(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);

extern eMBErrorCode __real_eMBRegHoldingCBSerialMaster(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode);

extern eMBErrorCode __real_eMBRegCoilsCBSerialMaster(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode);

extern eMBErrorCode __real_eMBRegDiscreteCBSerialMaster(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);

extern eMBErrorCode __real_mbc_reg_input_slave_cb(UCHAR * reg_buffer, USHORT address, USHORT n_regs);

extern eMBErrorCode __real_mbc_reg_holding_slave_cb(UCHAR * reg_buffer, USHORT address, USHORT n_regs, eMBRegisterMode mode);

extern eMBErrorCode __real_mbc_reg_coils_slave_cb(UCHAR* reg_buffer, USHORT address, USHORT n_coils, eMBRegisterMode mode);

extern eMBErrorCode __real_mbc_reg_discrete_slave_cb(UCHAR* reg_buffer, USHORT address, USHORT n_discrete);

extern eMBErrorCode __real_eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);

extern eMBErrorCode __real_eMBRegCoilsCB(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode);

extern eMBErrorCode __real_eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode);

extern eMBErrorCode __real_eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);

eMBErrorCode __wrap_eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);

eMBErrorCode __wrap_eMBRegCoilsCB(UCHAR* pucRegBuffer, USHORT usAddress,
                            USHORT usNCoils, eMBRegisterMode eMode);

eMBErrorCode __wrap_eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
                                USHORT usNRegs, eMBRegisterMode eMode);

eMBErrorCode __wrap_eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);