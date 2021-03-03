/* Copyright 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
/*
  * FreeModbus Libary: ESP32 TCP Port
  * Copyright (C) 2006 Christian Walter <wolti@sil.at>
  * Parts of crt0.S Copyright (c) 1995, 1996, 1998 Cygnus Support
  *
  * This library is free software; you can redistribute it and/or
  * modify it under the terms of the GNU Lesser General Public
  * License as published by the Free Software Foundation; either
  * version 2.1 of the License, or (at your option) any later version.
  *
  * This library is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * Lesser General Public License for more details.
  *
  * You should have received a copy of the GNU Lesser General Public
  * License along with this library; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  *
  * File: $Id: port.h,v 1.2 2006/09/04 14:39:20 wolti Exp $
  */

/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "sys/time.h"

/* ----------------------- lwIP includes ------------------------------------*/
#include "lwip/err.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port_tcp_slave.h"

/* ----------------------- MBAP Header --------------------------------------*/
#define MB_TCP_UID          6
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         7

/* ----------------------- Defines  -----------------------------------------*/
#define MB_TCP_DEFAULT_PORT         ( 502 ) // TCP listening port. */
#define MB_TCP_BUF_SIZE             ( 256 + 7 ) // Must hold a complete Modbus TCP frame.
#define MB_TCP_CONNECTION_TIMEOUT   ( CONFIG_MB_TCP_CONNECTION_TOUT_SEC ) // disconnect timeout in Seconds

#define MB_TCP_IPV4                 ( CONFIG_MB_TCP_IPV4 )
#define MB_TCP_PORT_TAG             "MB_TCP_PORT"
#define MB_TCP_TASK_PRIO            ( CONFIG_MB_TCP_TASK_PRIO )
#define MB_TCP_TASK_STACK_SIZE      ( CONFIG_MB_TCP_TASK_STACK_SIZE )

/* ----------------------- Prototypes ---------------------------------------*/
void vMBPortEventClose( void );
#ifdef MB_TCP_DEBUG
void vMBPortLog( eMBPortLogLevel eLevel, const CHAR * szModule, const CHAR * szFmt, ... );
#endif

/* ----------------------- Static variables ---------------------------------*/
static int xActiveSock = -1;
static int xListenSock = -1;
static UCHAR aucTCPBuf[MB_TCP_BUF_SIZE];
static USHORT usTCPBufPos = 0;
static TaskHandle_t  xMbTcpTaskHandle;
static USHORT usPort = MB_TCP_DEFAULT_PORT;

/* ----------------------- Static functions ---------------------------------*/
static int xMBTCPPortAcceptConnection(int xListenSockId)
{
    // Address structure large enough for both IPv4 or IPv6
    struct sockaddr_in6 xSrcAddr;
    socklen_t xAddrLen = sizeof(xSrcAddr);
    CHAR cAddrStr[128];
    int xSockId = xActiveSock;
    // Accept new socket connection if not active
    while(xSockId < 0) {
        xSockId = accept(xListenSockId, (struct sockaddr *)&xSrcAddr, &xAddrLen);
        if (xSockId < 0) {
            ESP_LOGE(MB_TCP_PORT_TAG, "Unable to accept connection: errno=%d", errno);
            close(xSockId);
        } else {
            ESP_LOGI(MB_TCP_PORT_TAG, "Socket connection accepted: (#%d)", xSockId);
            // Get the sender's ip address as string
            if (xSrcAddr.sin6_family == PF_INET) {
                inet_ntoa_r(((struct sockaddr_in *)&xSrcAddr)->sin_addr.s_addr, cAddrStr, sizeof(cAddrStr) - 1);
            } else if (xSrcAddr.sin6_family == PF_INET6) {
                inet6_ntoa_r(xSrcAddr.sin6_addr, cAddrStr, sizeof(cAddrStr) - 1);
            }
            ESP_LOGI(MB_TCP_PORT_TAG, "Accept client connection from address: %s", cAddrStr);
            vTaskDelay(1);
        }
    } // while(xSockId < 0)
    return xSockId;
}

static err_t xMBTCPPortRxPoll()
{
    err_t xErr = ERR_CLSD;
    UCHAR ucRxBuffer[MB_TCP_BUF_SIZE]; // Temp rx buffer

    // Receive data from connected client
    if (xActiveSock > -1) {
        // Blocking read of buffer with PUSH flag from active socket
        int xLength = recv(xActiveSock, ucRxBuffer, sizeof(ucRxBuffer) - 1, 0);
        if (xLength < 0) {
            if (errno == EAGAIN) {
                ESP_LOGE(MB_TCP_PORT_TAG, "recv timeout");
                xErr = ERR_TIMEOUT;
            } else {
                // If an error occured during receiving
                ESP_LOGE(MB_TCP_PORT_TAG, "recv failed: length=%d, errno=%d", xLength, errno);
                xErr = ERR_BUF;
            }
        }
        else if (xLength == 0) {
            // Socket connection closed
            ESP_LOGI(MB_TCP_PORT_TAG, "Socket (#%d) connection closed.", xActiveSock);
            xErr = ERR_CLSD;
            //break;
        } else {
            // Data buffer received
            // Check for internal buffer overflow. In case of an error drop the client.
            if ( ( usTCPBufPos + xLength ) >= MB_TCP_BUF_SIZE ) {
                xErr = ERR_BUF;
                //break;
            } else {
                // Copy data into Modbus buffer
                memcpy( &aucTCPBuf[usTCPBufPos], &ucRxBuffer[0], xLength );
                usTCPBufPos += xLength;
                // If we have received the MBAP header we can analyze it and calculate
                // the number of bytes left to complete the current request. If complete
                // notify the protocol stack.
                if ( usTCPBufPos >= MB_TCP_FUNC ) {
                    // Length is a byte count of Modbus PDU (function code + data) and the
                    // unit identifier.
                    xLength = aucTCPBuf[MB_TCP_LEN] << 8U;
                    xLength |= aucTCPBuf[MB_TCP_LEN + 1];
                    // Is the frame already complete.
                    if ( usTCPBufPos < ( MB_TCP_UID + xLength ) ) {
                        // The incompleted frame is received
                    } else if ( usTCPBufPos == ( MB_TCP_UID + xLength ) ) {
#ifdef MB_TCP_DEBUG
                        prvvMBTCPLogFrame(MB_TCP_PORT_TAG, (UCHAR*)&aucTCPBuf[0], usTCPBufPos);
#endif
                        // Complete frame received
                        ( void )xMBPortEventPost(EV_FRAME_RECEIVED);
                        xErr = ESP_OK;
                    }
                } else { // if( usTCPBufPos >= MB_TCP_FUNC )
                    ESP_LOGI(MB_TCP_PORT_TAG, "Incorrect buffer received (%u) bytes.", xLength);
                    // This should not happen. We can't deal with such a client and
                    // drop the connection for security reasons.
                    xErr = ERR_BUF;
                    //break;
                }
            } // else if( ( usTCPBufPos + xLength ) >= MB_TCP_BUF_SIZE )
        } // if data received
    }
    return xErr;
}

static void
vMBTCPPortServerTask(void *pvParameters)
{
    int xAddrFamily;
    int xIpProtocol;
    struct timeval tv;
    int64_t xLastResponseTimeStamp = 0;

    // Main connection cycle
    while (1) {
#ifdef MB_TCP_IPV4
        struct sockaddr_in xDestAddr;
        xDestAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        xDestAddr.sin_family = AF_INET;
        xDestAddr.sin_port = htons(usPort);
        xAddrFamily = AF_INET;
        xIpProtocol = IPPROTO_IP;
#else // IPV6 option
        struct sockaddr_in6 xDestAddr;
        bzero(&xDestAddr.sin6_addr.un, sizeof(xDestAddr.sin6_addr.un));
        xDestAddr.sin6_family = AF_INET6;
        xDestAddr.sin6_port = htons(usPort);
        xAddrFamily = AF_INET6;
        xIpProtocol = IPPROTO_IPV6;
#endif
        if (xListenSock < 0) {
            xListenSock = socket(xAddrFamily, SOCK_STREAM, xIpProtocol);
            if (xListenSock < 0) {
                ESP_LOGE(MB_TCP_PORT_TAG, "Unable to create socket: errno %d", errno);
                close(xListenSock);
                continue;
            }
        }
        ESP_LOGI(MB_TCP_PORT_TAG, "Socket created (#%d)", xListenSock);

        // Bind connection
        int xErr = bind(xListenSock, (struct sockaddr *)&xDestAddr, sizeof(xDestAddr));
        if (xErr != 0) {
            // Bind fail then try to bind again
            ESP_LOGE(MB_TCP_PORT_TAG, "Socket unable to bind socket (#%d): error=%d, errno=%d",
                                                    xListenSock, xErr, errno);
            close(xListenSock);
            continue;
        }
        ESP_LOGI(MB_TCP_PORT_TAG, "Socket (#%d) binded.", xListenSock);

        // Listen connection
        xErr = listen(xListenSock, 3);
        if (xErr != 0) {
            // Listen connection fail => restart connection
            ESP_LOGE(MB_TCP_PORT_TAG, "Error occured during listen: error=%d, errno=%d", xErr, errno);
            close(xListenSock);
            continue;
        }
        ESP_LOGI(MB_TCP_PORT_TAG, "Listening socket (#%d).", xListenSock);

        // Connection handling cycle
        while(1) {
            // Accept new client connection
            xActiveSock = xMBTCPPortAcceptConnection(xListenSock);
            if (xActiveSock > 0) {
                xLastResponseTimeStamp = xMBTCPGetTimeStamp();
            }
            // Set socket connection timeout in seconds
            tv.tv_sec = MB_TCP_CONNECTION_TIMEOUT;
            tv.tv_usec = 0;
            setsockopt(xActiveSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            xErr = (int)xMBTCPPortRxPoll();
            // If an invalid data received from socket or connection fail
            // or if timeout then drop connection and restart
            if (xErr != ERR_OK) {
                uint64_t xTimeStamp = xMBTCPGetTimeStamp();
                // If data update is timed out
                if (xErr == ERR_TIMEOUT) {
                    ESP_LOGE(MB_TCP_PORT_TAG, "Data receive timeout, time[us]: %ju, close active connection.",
                                                            (xTimeStamp - xLastResponseTimeStamp));
                } else {
                    ESP_LOGE(MB_TCP_PORT_TAG, "Data processing error. Restartarting...");
                }
                close(xActiveSock);
                xActiveSock = -1;
            } else {
                // Get timestamp of last data update
                xLastResponseTimeStamp = xMBTCPGetTimeStamp();
            }
        } // while(1) // Handle connection cycle
    } // Main connection cycle
    vTaskDelete(NULL);
}

/* ----------------------- Begin implementation -----------------------------*/
BOOL
xMBTCPPortInit( USHORT usTCPPort )
{
    BOOL bOkay = FALSE;
    if( usTCPPort == 0 )
    {
        usPort = MB_TCP_DEFAULT_PORT;
    }
    else
    {
        usPort = (USHORT)usTCPPort;
    }
    // Create task for packet processing
    BaseType_t xErr = xTaskCreate(vMBTCPPortServerTask,
                                    "tcp_server_task",
                                    MB_TCP_TASK_STACK_SIZE,
                                    NULL,
                                    MB_TCP_TASK_PRIO,
                                    &xMbTcpTaskHandle);
    if (xErr != pdTRUE)
    {
        ESP_LOGE(MB_TCP_PORT_TAG, "Server task creation failure.");
        (void)vTaskDelete(xMbTcpTaskHandle);
    } else {
        ESP_LOGI(MB_TCP_PORT_TAG, "Protocol stack initialized.");
        bOkay = TRUE;
    }
    return bOkay;
}

void
vMBTCPPortClose( )
{
    // Release resources for the event queue.
    vMBPortEventClose( );
    (void)vTaskDelete(xMbTcpTaskHandle);
}

void
vMBTCPPortDisable( void )
{
    if (shutdown(xActiveSock, SHUT_RDWR) == -1) {
        ESP_LOGE(MB_TCP_PORT_TAG, "Shutdown failed: errno %d", errno);
    }
    (void)vTaskSuspend(xMbTcpTaskHandle);
    close(xActiveSock);
    close(xListenSock);
    xActiveSock = -1;
    xListenSock = -1;
}

BOOL
xMBTCPPortGetRequest( UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength )
{
    *ppucMBTCPFrame = &aucTCPBuf[0];
    *usTCPLength = usTCPBufPos;

    // Reset the buffer.
    usTCPBufPos = 0;
    return TRUE;
}

BOOL
xMBTCPPortSendResponse( const UCHAR * pucMBTCPFrame, USHORT usTCPLength )
{
    BOOL bFrameSent = TRUE;
    int xErr = send(xActiveSock, pucMBTCPFrame, usTCPLength, 0);
    if (xErr < 0) {
        ESP_LOGE(MB_TCP_PORT_TAG, "Error occured during sending: errno %d", errno);
        bFrameSent = FALSE;
        close(xActiveSock);
        xActiveSock = -1;
    }
    return bFrameSent;
}
