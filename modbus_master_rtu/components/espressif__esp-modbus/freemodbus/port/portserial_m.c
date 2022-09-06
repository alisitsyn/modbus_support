/*
 * SPDX-FileCopyrightText: 2013 Armink
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SPDX-FileContributor: 2016-2021 Espressif Systems (Shanghai) CO LTD
 */
/*
 * FreeModbus Libary: ESP32 Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include <string.h>
#include "driver/uart.h"
#include "soc/dport_access.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mbport.h"
#include "mb_m.h"
#include "mbrtu.h"
#include "mbconfig.h"
#include "port_serial_master.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SERIAL_RX_FLUSH_RETRY    (2)

typedef struct {
    portMUX_TYPE xSpinLock;
    UCHAR ucUartNumber;
    BOOL bRxStateEnabled;
    BOOL bTxStateEnabled;
    uint64_t xTransactionCount;
    USHORT usRecvLength;
    uint64_t xSendTimeStamp;
    uint64_t xRecvTimeStamp;
    uint32_t usFlags;
    QueueHandle_t xMbUartQueue; // A queue to handle UART event.
    TaskHandle_t  xMbTaskHandle; // UART task to handle UART event.
    SemaphoreHandle_t xMasterSemaRxHandle; // Rx blocking semaphore handle
} PortContext_t;

/* ----------------------- Static variables ---------------------------------*/
static const CHAR *TAG = "MB_MASTER_SERIAL";

// The UART hardware context
static PortContext_t xPortContext = {
        .xSpinLock = portMUX_INITIALIZER_UNLOCKED, \
        .ucUartNumber= (UART_NUM_MAX - 1), \
        .bRxStateEnabled = FALSE, \
        .bTxStateEnabled = FALSE, \
        .xRecvTimeStamp = 0, \
        .xSendTimeStamp = 0, \
        .xTransactionCount = 0, \
        .usRecvLength = 0, \
        .usFlags = 0, \
        .xMbUartQueue = NULL, \
        .xMbTaskHandle = NULL, \
        .xMasterSemaRxHandle = NULL \
};

// Function returns time left for response processing according to response timeout
static IRAM_ATTR int64_t xMBSerialPortMasterGetRespTimeLeft(int64_t xTime)
{
    int64_t xStartTime = xTime;
    if (xStartTime <= 0) {
        xStartTime = esp_timer_get_time();
    }
    int64_t xTimeStamp = xStartTime - xPortContext.xSendTimeStamp;
    int64_t xTimeLeft = (xTimeStamp > (1000 * MB_MASTER_TIMEOUT_MS_RESPOND)) ? 0 :
            (MB_MASTER_TIMEOUT_MS_RESPOND - (xTimeStamp / 1000) - 1);
    return xTimeLeft;
}

static BOOL xMBMasterPortRxSemaInit( void )
{
    xPortContext.xMasterSemaRxHandle = xSemaphoreCreateBinary();
    MB_PORT_CHECK((xPortContext.xMasterSemaRxHandle != NULL), FALSE , "%s: RX semaphore create failure.", __func__);
    return TRUE;
}

static BOOL xMBMasterPortRxSemaTake( LONG lTimeOut )
{
    BaseType_t xStatus = pdTRUE;
    xStatus = xSemaphoreTake(xPortContext.xMasterSemaRxHandle, lTimeOut );
    MB_PORT_CHECK((xStatus == pdTRUE), FALSE , "%s: RX semaphore take failure.", __func__);
    ESP_LOGV(MB_PORT_TAG,"%s:Take RX semaphore (%lu ticks).", __func__, lTimeOut);
    return TRUE;
}

static void vMBMasterRxSemaRelease( void )
{
    BaseType_t xStatus = pdFALSE;
    xStatus = xSemaphoreGive(xPortContext.xMasterSemaRxHandle);
    if (xStatus != pdTRUE) {
        ESP_LOGD(MB_PORT_TAG,"%s:RX semaphore is free.", __func__);
    }
}

static BOOL vMBMasterRxSemaIsBusy( void )
{
    BaseType_t xStatus = pdFALSE;
    xStatus = (uxSemaphoreGetCount(xPortContext.xMasterSemaRxHandle) == 0) ? TRUE : FALSE;
    return xStatus;
}

static void vMBMasterRxFlush( void )
{
    size_t xSize = 1;
    esp_err_t xErr = ESP_OK;
    for (int xCount = 0; (xCount < MB_SERIAL_RX_FLUSH_RETRY) && xSize; xCount++) {
        xErr = uart_get_buffered_data_len(xPortContext.ucUartNumber, &xSize);
        MB_PORT_CHECK((xErr == ESP_OK), ; , "mb flush serial fail, error = 0x%x.", xErr);
        BaseType_t xStatus = xQueueReset(xPortContext.xMbUartQueue);
        if (xStatus) {
            xErr = uart_flush_input(xPortContext.ucUartNumber);
            MB_PORT_CHECK((xErr == ESP_OK), ; , "mb flush serial fail, error = 0x%x.", xErr);
        }
    }
}

void vMBMasterRxFlushResp( void )
{
    size_t xSize = 1;
    esp_err_t xErr = ESP_OK;
    int64_t xStartTime = esp_timer_get_time();
    int64_t xTimeLeft = 0;
    while( (xTimeLeft < (1000 * MB_MASTER_TIMEOUT_MS_RESPOND)) ){
        xTimeLeft = esp_timer_get_time() - xStartTime;
        xErr = uart_get_buffered_data_len(xPortContext.ucUartNumber, &xSize);
        MB_PORT_CHECK((xErr == ESP_OK), ; , "mb flush serial fail, error = 0x%x.", xErr);
        BaseType_t xStatus = xQueueReset(xPortContext.xMbUartQueue);
        if (xStatus) {
            xErr = uart_flush_input(xPortContext.ucUartNumber);
            MB_PORT_CHECK((xErr == ESP_OK), ; , "mb flush serial fail, error = 0x%x.", xErr);
        } 
    }
}

void vMBMasterPortSerialEnable(BOOL bRxEnable, BOOL bTxEnable)
{
    // This function can be called from xMBRTUTransmitFSM() of different task
    if (bTxEnable) {
        xPortContext.bTxStateEnabled = TRUE;
    } else {
        xPortContext.bTxStateEnabled = FALSE;
    }
    if (bRxEnable) {
        xPortContext.bRxStateEnabled = TRUE;
        vMBMasterRxSemaRelease();
        vTaskResume(xPortContext.xMbTaskHandle); // Resume receiver task
    } else {
        vTaskSuspend(xPortContext.xMbTaskHandle); // Block receiver task
        xPortContext.bRxStateEnabled = FALSE;
    }
}

BOOL xMBMasterPortSerialGetResponse( UCHAR **ppucMBSerialFrame, USHORT * usSerialLength ) 
{
    USHORT usCount = xPortContext.usRecvLength;
    BOOL xReadStatus = FALSE;

    MB_PORT_CHECK((ppucMBSerialFrame && usSerialLength), FALSE, "mb serial get buffer failure.");

    xReadStatus = xMBMasterPortRxSemaTake(pdMS_TO_TICKS(xMBSerialPortMasterGetRespTimeLeft(0)));
    if (xReadStatus && usCount && ppucMBSerialFrame) {
        // Read frame data from the ringbuffer of receiver
        usCount = uart_read_bytes(xPortContext.ucUartNumber, (uint8_t*)*ppucMBSerialFrame,
                                    xPortContext.usRecvLength, pdMS_TO_TICKS(xMBSerialPortMasterGetRespTimeLeft(0)));
        // The buffer is transferred into Modbus stack and is not needed here any more
        uart_flush_input(xPortContext.ucUartNumber);
        ESP_LOGD(TAG, "Received data: %d(bytes in buffer)\n", (uint32_t)usCount);
        xReadStatus = TRUE;
        // Store the timestamp of received frame
        xPortContext.xRecvTimeStamp = esp_timer_get_time();
        ESP_LOG_BUFFER_HEX_LEVEL("RECEIVE", (void*)*ppucMBSerialFrame, usCount, ESP_LOG_DEBUG);
        int64_t xTime = xPortContext.xRecvTimeStamp - xPortContext.xSendTimeStamp;
        ESP_LOGD(TAG, "Request processing time[us] = %ju.", xTime);
    } else {
        ESP_LOGE(TAG, "%s: Junk data (%d bytes) received. ", __func__, *usSerialLength);
    }
    *usSerialLength = usCount;
    return xReadStatus;
}

BOOL xMBMasterPortSerialSendRequest( UCHAR *pucMBSerialFrame, USHORT usSerialLength )
{
    BOOL xResult = FALSE;
    int xCount = 0;

    // The place to enable RS485 driver
    vMBMasterPortSerialEnable( FALSE, TRUE );

    if (pucMBSerialFrame && usSerialLength && xPortContext.bTxStateEnabled) {
        // Flush buffer received from previous transaction
        vMBMasterRxFlush();
        xCount = uart_write_bytes(xPortContext.ucUartNumber, pucMBSerialFrame, usSerialLength);
        ESP_LOGD(TAG, "MB_TX_buffer sent: (%d) bytes.", (uint16_t)(xCount - 1));
        // Waits while UART sending the packet
        esp_err_t xTxStatus = uart_wait_tx_done(xPortContext.ucUartNumber, MB_SERIAL_TX_TOUT_TICKS);
        vMBMasterPortSerialEnable(TRUE, FALSE);
        MB_PORT_CHECK((xTxStatus == ESP_OK), FALSE, "mb serial sent buffer failure.");
        // Print sent packet, the tag used is more clear to see
        ESP_LOG_BUFFER_HEX_LEVEL("SENT", (void*)pucMBSerialFrame, usSerialLength, ESP_LOG_DEBUG);
        xMBMasterPortEventPost(EV_MASTER_FRAME_SENT);
        xPortContext.xSendTimeStamp = esp_timer_get_time();
        xResult = TRUE;
    } else {
        ESP_LOGE(TAG, "Send callback %d, %p, %d. ", xPortContext.bTxStateEnabled, pucMBSerialFrame, usSerialLength);
    }
    return xResult;
}

// UART receive event task
static void vUartTask(void* pvParameters)
{
    uart_event_t xEvent;
    for(;;) {
        if (xMBPortSerialWaitEvent(xPortContext.xMbUartQueue, (void*)&xEvent, portMAX_DELAY)) {
            ESP_LOGD(TAG, "MB_uart[%d] event:", xPortContext.ucUartNumber);
            switch(xEvent.type) {
                //Event of UART receiving data
                case UART_DATA:
                    ESP_LOGD(TAG,"Data event, len: %d.", xEvent.size);
                    // This flag set in the event means that no more
                    // data received during configured timeout and UART TOUT feature is triggered
                    if (xEvent.timeout_flag) {
                        // Response is received but previous packet processing is pending
                        // Do not wait completion of processing and just discard received data as incorrect
                        if (vMBMasterRxSemaIsBusy()) {
                            vMBMasterRxFlush();
                            break;
                        }
                        uart_get_buffered_data_len(xPortContext.ucUartNumber, (unsigned int*)&xEvent.size);
                        xPortContext.usRecvLength = (xEvent.size < MB_SERIAL_BUF_SIZE) ? xEvent.size : MB_SERIAL_BUF_SIZE;
                        if (xEvent.size <= MB_SER_PDU_SIZE_MIN) {
                            ESP_LOGW(TAG,"Drop short packet %d byte(s)", xEvent.size);
                            vMBMasterRxFlush();
                            break;
                        }
                        // New frame is received, send event to main FSM
                        xMBMasterPortEventPost(EV_MASTER_FRAME_RECEIVED);
                        ESP_LOGD(TAG,"Received %d bytes in buffer.", xPortContext.usRecvLength);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGD(TAG, "hw fifo overflow.");
                    xQueueReset(xPortContext.xMbUartQueue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGD(TAG, "ring buffer full.");
                    xQueueReset(xPortContext.xMbUartQueue);
                    uart_flush_input(xPortContext.ucUartNumber);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGD(TAG, "uart rx break.");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGD(TAG, "uart parity error.");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGD(TAG, "uart frame error.");
                    break;
                default:
                    ESP_LOGD(TAG, "uart event type: %d.", xEvent.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    esp_err_t xErr = ESP_OK;
    // Set communication port number
    xPortContext.ucUartNumber = ucPORT;
    // Configure serial communication parameters
    UCHAR ucParity = UART_PARITY_DISABLE;
    UCHAR ucData = UART_DATA_8_BITS;
    switch(eParity){
        case MB_PAR_NONE:
            ucParity = UART_PARITY_DISABLE;
            break;
        case MB_PAR_ODD:
            ucParity = UART_PARITY_ODD;
            break;
        case MB_PAR_EVEN:
            ucParity = UART_PARITY_EVEN;
            break;
        default:
            ESP_LOGE(TAG, "Incorrect parity option: %d", eParity);
            return FALSE;
    }
    switch(ucDataBits){
        case 5:
            ucData = UART_DATA_5_BITS;
            break;
        case 6:
            ucData = UART_DATA_6_BITS;
            break;
        case 7:
            ucData = UART_DATA_7_BITS;
            break;
        case 8:
            ucData = UART_DATA_8_BITS;
            break;
        default:
            ucData = UART_DATA_8_BITS;
            break;
    }
    uart_config_t xUartConfig = {
        .baud_rate = ulBaudRate,
        .data_bits = ucData,
        .parity = ucParity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 2,
        .source_clk = UART_SCLK_APB,
    };
    // Set UART config
    xErr = uart_param_config(xPortContext.ucUartNumber, &xUartConfig);
    MB_PORT_CHECK((xErr == ESP_OK),
            FALSE, "mb config failure, uart_param_config() returned (0x%x).", xErr);
    // Install UART driver, and get the queue.
    xErr = uart_driver_install(xPortContext.ucUartNumber, MB_SERIAL_BUF_SIZE, MB_SERIAL_BUF_SIZE,
                                    MB_QUEUE_LENGTH, &xPortContext.xMbUartQueue, MB_PORT_SERIAL_ISR_FLAG);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "mb serial driver failure, uart_driver_install() returned (0x%x).", xErr);
    // Set timeout for TOUT interrupt (T3.5 modbus time)
    xErr = uart_set_rx_timeout(xPortContext.ucUartNumber, MB_SERIAL_TOUT);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "mb serial set rx timeout failure, uart_set_rx_timeout() returned (0x%x).", xErr);

    // Set always timeout flag to trigger timeout interrupt even after rx fifo full
    uart_set_always_rx_timeout(xPortContext.ucUartNumber, true);
    MB_PORT_CHECK((xMBMasterPortRxSemaInit()), FALSE,
                        "mb serial RX semaphore create fail.");
    // Create a task to handle UART events
    BaseType_t xStatus = xTaskCreatePinnedToCore(vUartTask, "uart_queue_task",
                                                    MB_SERIAL_TASK_STACK_SIZE,
                                                    NULL, MB_SERIAL_TASK_PRIO,
                                                    &xPortContext.xMbTaskHandle, MB_PORT_TASK_AFFINITY);
    if (xStatus != pdPASS) {
        vTaskDelete(xPortContext.xMbTaskHandle);
        // Force exit from function with failure
        MB_PORT_CHECK(FALSE, FALSE,
                "mb stack serial task creation error. xTaskCreate() returned (0x%x).",
                xStatus);
    } else {
        vTaskSuspend(xPortContext.xMbTaskHandle); // Suspend serial task while stack is not started
    }
    ESP_LOGD(MB_PORT_TAG,"%s Init serial.", __func__);
    return TRUE;
}

void vMBMasterPortSerialClose(void)
{
    (void)vTaskDelete(xPortContext.xMbTaskHandle);
    ESP_ERROR_CHECK(uart_driver_delete(xPortContext.ucUartNumber));
}

// The functions belos are just placeholders now (will not be used)
BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
    return TRUE;
}

// Get one byte from intermediate RX buffer
BOOL xMBMasterPortSerialGetByte(CHAR* pucByte)
{
    return TRUE;
}
