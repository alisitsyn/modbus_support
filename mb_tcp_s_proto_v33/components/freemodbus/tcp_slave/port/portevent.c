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
  * File: $Id: portevent.c,v 1.1 2006/09/04 01:41:49 wolti Exp $
  */

/* ----------------------- System includes ----------------------------------*/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* ----------------------- lwIP ---------------------------------------------*/
#include "lwip/api.h"
#include "lwip/sys.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "port_tcp_slave.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_POLL_CYCLETIME       (portMAX_DELAY)
/* ----------------------- Static variables ---------------------------------*/
static xQueueHandle xQueueHdl;

#define MB_EVENT_QUEUE_SIZE     (1)
#define MB_EVENT_QUEUE_TIMEOUT  (pdMS_TO_TICKS(CONFIG_MB_EVENT_QUEUE_TIMEOUT))

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    BOOL bStatus = FALSE;
    if((xQueueHdl = xQueueCreate(MB_EVENT_QUEUE_SIZE, sizeof(eMBEventType))) != NULL)
    {
        vQueueAddToRegistry(xQueueHdl, "MbPortEventQueue");
        bStatus = TRUE;
    }
    return bStatus;
}

void
vMBPortEventClose( void )
{
    if(xQueueHdl != NULL)
    {
        vQueueDelete(xQueueHdl);
        xQueueHdl = NULL;
    }
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    BOOL bStatus = TRUE;
    assert(xQueueHdl != NULL);

    if( (BOOL)xPortInIsrContext() == TRUE ) {
        xQueueSendFromISR(xQueueHdl, (const void*)&eEvent, pdFALSE);
    }
    else {
        xQueueSend(xQueueHdl, (const void*)&eEvent, MB_EVENT_QUEUE_TIMEOUT);
    }
    return bStatus;
}

BOOL
xMBPortEventGet( eMBEventType * peEvent )
{
    assert(xQueueHdl != NULL);
    BOOL xEventHappened = FALSE;

    if (xQueueReceive(xQueueHdl, peEvent, MB_POLL_CYCLETIME) == pdTRUE) {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

xQueueHandle
xMBPortEventGetHandle(void)
{
    if(xQueueHdl != NULL) //
    {
        return xQueueHdl;
    }
    return NULL;
}
