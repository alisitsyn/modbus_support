/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbascii.c,v 1.17 2010/06/06 13:47:07 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb_m.h"
#include "mbconfig.h"
#include "mbascii.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

#if MB_MASTER_ASCII_ENABLED > 0

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_M_RX_INIT,     /*!< Receiver is in initial state. */
    STATE_M_RX_IDLE,     /*!< Receiver is in idle state. */
    STATE_M_RX_RCV       /*!< Frame is beeing received. */
} eMBMasterAsciiRcvState;

typedef enum
{
    STATE_M_TX_IDLE,   /*!< Transmitter is in idle state. */
    STATE_M_TX_START,  /*!< Starting transmission (':' sent). */
    STATE_M_TX_END     /*!< End of transmission. */
} eMBMasterAsciiSndState;

#define MB_ASCII_BUF_SIZE (MB_SERIAL_BUF_SIZE * 2 + MB_SER_PDU_SIZE_CRC + 2)
#define MB_SER_PDU_SIZE_MIN 4 /*!< Minimum size of a Modbus RTU frame. */

/* ----------------------- Shared values  -----------------------------------*/
/* These Modbus values are shared in ASCII mode*/
extern volatile UCHAR ucMasterRcvBuf[];
extern volatile eMBMasterTimerMode eMasterCurTimerMode;

/* ----------------------- Static functions ---------------------------------*/
static UCHAR prvucMBCHAR2BIN(UCHAR ucCharacter);

static UCHAR prvucMBBIN2CHAR(UCHAR ucByte);

static UCHAR prvucMBLRC(UCHAR *pucFrame, USHORT usLen);

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBMasterAsciiSndState eSndState;
static volatile eMBMasterAsciiRcvState eRcvState;

static volatile UCHAR *ucMasterASCIIRcvBuf = ucMasterRcvBuf;

static volatile USHORT usMasterRcvBufferPos;

static volatile UCHAR *pucMasterSndBufferCur;
static volatile USHORT usMasterSndBufferCount;

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBMasterASCIIInit(UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity)
{
    eMBErrorCode eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION();

    if (xMBMasterPortSerialInit(ucPort, ulBaudRate, MB_ASCII_BITS_PER_SYMB, eParity) != TRUE)
    {
        eStatus = MB_EPORTERR;
    }
    else if (xMBMasterPortTimersInit(MB_ASCII_TIMEOUT_MS * MB_TIMER_TICS_PER_MS) != TRUE)
    {
        eStatus = MB_EPORTERR;
    }

    EXIT_CRITICAL_SECTION();

    return eStatus;
}

void eMBMasterASCIIStart(void)
{
    ENTER_CRITICAL_SECTION();
    eRcvState = STATE_M_RX_INIT;
    vMBMasterPortSerialEnable(TRUE, FALSE);
    vMBMasterPortTimersT35Enable();
    EXIT_CRITICAL_SECTION();
}

void eMBMasterASCIIStop(void)
{
    ENTER_CRITICAL_SECTION();
    vMBMasterPortSerialEnable(FALSE, FALSE);
    vMBMasterPortTimersDisable();
    EXIT_CRITICAL_SECTION();
}

static int xMBMasterASCIIGetBinaryBuffer(UCHAR *pcBuffer, int xLength)
{
    int xBinIdx = 0;
    UCHAR ucLRC = 0;

    assert(pcBuffer && (xLength < MB_SERIAL_BUF_SIZE));

    if ((pcBuffer[0] == ':') && (pcBuffer[xLength - 1] == '\n') && (pcBuffer[xLength - 2] == '\r'))
    {
        for (int xStrIdx = 1; (xStrIdx < xLength) && (pcBuffer[xStrIdx] > ' '); xStrIdx += 2)
        {
            pcBuffer[xBinIdx] = (prvucMBCHAR2BIN(pcBuffer[xStrIdx]) << 4); // High nibble
            pcBuffer[xBinIdx] |= prvucMBCHAR2BIN(pcBuffer[xStrIdx + 1]);   // Low nibble
            ucLRC += pcBuffer[xBinIdx++];
        }
    }
    
    ucLRC = (UCHAR)(-((CHAR)ucLRC));
    xBinIdx = ((ucLRC == 0) && (xBinIdx == ((xLength - 3) >> 1))) ? xBinIdx : -1;
    return xBinIdx;
}

eMBErrorCode
eMBMasterASCIIReceive(UCHAR *pucRcvAddress, UCHAR **pucFrame, USHORT *pusLength)
{
    eMBErrorCode eStatus = MB_ENOERR;
    UCHAR *pucMBASCIIFrame = (UCHAR *)ucMasterASCIIRcvBuf;
    USHORT usFrameLength = usMasterRcvBufferPos;

    if (xMBMasterPortSerialGetResponse(&pucMBASCIIFrame, &usFrameLength) == FALSE)
    {
        return MB_EIO;
    }

    assert(usFrameLength <= MB_SER_PDU_SIZE_MAX);

    int xCount = xMBMasterASCIIGetBinaryBuffer(pucMBASCIIFrame, usFrameLength);

    ENTER_CRITICAL_SECTION();

    assert(pucMBASCIIFrame);
    /* Length and CRC check */
    if (xCount >= MB_ASCII_SER_PDU_SIZE_MIN)
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = pucMBASCIIFrame[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = (USHORT)(usFrameLength - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_LRC);

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = (UCHAR *)&pucMBASCIIFrame[MB_SER_PDU_PDU_OFF];

        eRcvState = STATE_M_RX_IDLE;
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION();
    return eStatus;
}

// This is helper function to fill and send ASCII frame buffer
static int xMBMasterASCIISendBuffer(UCHAR *pcBuffer, int xBinLength)
{
    int xBinIdx = 0;
    int xStrIdx = 0;
    UCHAR pucAsciiBuffer[MB_ASCII_BUF_SIZE] = {0};
    UCHAR ucLRC = 0;

    assert(((xBinLength << 1) + 5) < MB_ASCII_BUF_SIZE);

    ENTER_CRITICAL_SECTION();

    pucAsciiBuffer[0] = ':';
    for (xStrIdx = 1; (xBinIdx < xBinLength); xBinIdx++)
    {
        pucAsciiBuffer[xStrIdx++] = prvucMBBIN2CHAR((UCHAR)(pcBuffer[xBinIdx] >> 4));   // High nibble
        pucAsciiBuffer[xStrIdx++] = prvucMBBIN2CHAR((UCHAR)(pcBuffer[xBinIdx] & 0X0F)); // Low nibble
        ucLRC += pcBuffer[xBinIdx];
    }
    ucLRC = (UCHAR)(-((CHAR)ucLRC));
    pucAsciiBuffer[xStrIdx++] = prvucMBBIN2CHAR((UCHAR)(ucLRC >> 4));
    pucAsciiBuffer[xStrIdx++] = prvucMBBIN2CHAR((UCHAR)(ucLRC & 0X0F));
    pucAsciiBuffer[xStrIdx++] = '\r';
    pucAsciiBuffer[xStrIdx++] = '\n';
    
    EXIT_CRITICAL_SECTION();

    if (xMBMasterPortSerialSendRequest((UCHAR *)pucAsciiBuffer, xStrIdx) == FALSE)
    {
        xStrIdx = -1;
    }

    return xStrIdx;
}

eMBErrorCode
eMBMasterASCIISend(UCHAR ucSlaveAddress, const UCHAR *pucFrame, USHORT usLength)
{
    eMBErrorCode eStatus = MB_ENOERR;

    if (ucSlaveAddress > MB_MASTER_TOTAL_SLAVE_NUM)
        return MB_EINVAL;

    if (pucFrame && usLength)
    {
        ENTER_CRITICAL_SECTION();
        
        /* First byte before the Modbus-PDU is the slave address. */
        pucMasterSndBufferCur = (UCHAR *)pucFrame - 1;
        usMasterSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucMasterSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usMasterSndBufferCount += usLength;

        /* Activate the transmitter. */
        eSndState = STATE_M_TX_START;

        EXIT_CRITICAL_SECTION();

        int xRet = xMBMasterASCIISendBuffer((UCHAR *)pucMasterSndBufferCur, usLength + 1);

        BOOL xFrameIsBroadcast = (ucSlaveAddress == MB_ADDRESS_BROADCAST) ? TRUE : FALSE;
        vMBMasterRequestSetType(xFrameIsBroadcast);

        // If the frame is broadcast, master will enable timer of convert delay,
        // else master will enable timer of respond timeout. */
        if (xFrameIsBroadcast)
        {
            vMBMasterPortTimersConvertDelayEnable();
        }
        else
        {
            vMBMasterPortTimersRespondTimeoutEnable();
        }

        eStatus = xRet ? MB_ENOERR : MB_EIO;
        eSndState = STATE_M_TX_IDLE;
    }
    else
    {
        eStatus = MB_EIO;
    }
    return eStatus;
}

// These functions are placeholders (not used for processing)
BOOL xMBMasterASCIIReceiveFSM(void)
{
    return TRUE;
}

BOOL xMBMasterASCIITransmitFSM(void)
{
    return TRUE;
}

BOOL MB_PORT_ISR_ATTR
xMBMasterASCIITimerT1SExpired(void)
{
    BOOL xNeedPoll = FALSE;
    eMBMasterTimerMode eMasterTimerMode;

    eMasterTimerMode = xMBMasterGetCurTimerMode();
    vMBMasterPortTimersDisable();

    switch (eMasterTimerMode) 
    {
        case MB_TMODE_T35:
            eRcvState = STATE_M_RX_IDLE;
            xNeedPoll = xMBMasterPortEventPost(EV_MASTER_READY);
            ESP_EARLY_LOGD("TIMER_INT", "EV_MASTER_READY");
            break;

        case MB_TMODE_RESPOND_TIMEOUT:
            vMBMasterSetErrorType(EV_ERROR_RESPOND_TIMEOUT);
            xNeedPoll = xMBMasterPortEventPost(EV_MASTER_ERROR_PROCESS);
            ESP_EARLY_LOGD("TIMER_INT", "EV_ERROR_RESPOND_TIMEOUT");
            break;

        case MB_TMODE_CONVERT_DELAY:
            /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
            xNeedPoll = xMBMasterPortEventPost(EV_MASTER_EXECUTE);
            ESP_EARLY_LOGD("TIMER_INT", "MB_TMODE_CONVERT_DELAY");
            break;
    }

    return xNeedPoll;
}

static UCHAR
prvucMBCHAR2BIN(UCHAR ucCharacter)
{
    if ((ucCharacter >= '0') && (ucCharacter <= '9'))
    {
        return (UCHAR)(ucCharacter - '0');
    }
    else if ((ucCharacter >= 'A') && (ucCharacter <= 'F'))
    {
        return (UCHAR)(ucCharacter - 'A' + 0x0A);
    }
    else
    {
        return 0xFF;
    }
}

static UCHAR
prvucMBBIN2CHAR(UCHAR ucByte)
{
    if (ucByte <= 0x09)
    {
        return (UCHAR)('0' + ucByte);
    }
    else if ((ucByte >= 0x0A) && (ucByte <= 0x0F))
    {
        return (UCHAR)(ucByte - 0x0A + 'A');
    }
    else
    {
        /* Programming error. */
        assert(0);
    }
    return '0';
}

UCHAR __attribute__ ((unused))
prvucMBLRC(UCHAR *pucFrame, USHORT usLen)
{
    UCHAR ucLRC = 0; /* LRC char initialized */

    while (usLen--)
    {
        ucLRC += *pucFrame++; /* Add buffer byte without carry */
    }

    /* Return twos complement */
    ucLRC = (UCHAR)(-((CHAR)ucLRC));
    return ucLRC;
}

#endif
