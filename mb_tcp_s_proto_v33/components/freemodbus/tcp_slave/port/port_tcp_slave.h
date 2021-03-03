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

#ifndef _PORT_TCP_SLAVE_H
#define _PORT_TCP_SLAVE_H

/* ----------------------- Platform includes --------------------------------*/
#include "esp_log.h"

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "port.h"

/* ----------------------- Defines ------------------------------------------*/

#ifndef TRUE
#define TRUE                    1
#endif

#ifndef FALSE
#define FALSE                   0
#endif

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#define _ip4_addr_byte(ipaddr, idx) (((const u8_t*)(&(ipaddr)->u_addr.ip4))[idx])

#define MB_TCP_DEBUG 1       // Debug output in TCP module.

#define MB_TCP_PORT_TAG "MB_TCP_PORT"

#define MB_TCP_PORT_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MB_TCP_PORT_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

/* ----------------------- Type definitions ---------------------------------*/

#ifdef MB_TCP_DEBUG
typedef enum
{
    MB_LOG_DEBUG,
    MB_LOG_INFO,
    MB_LOG_WARN,
    MB_LOG_ERROR
} eMBPortLogLevel;
#endif

/* ----------------------- Function prototypes ------------------------------*/
#ifdef MB_TCP_DEBUG
void vMBPortLog( eMBPortLogLevel eLevel, const CHAR * szModule,
                                                const CHAR * szFmt, ... );
void prvvMBTCPLogFrame( const CHAR * pucMsg, UCHAR * pucFrame, USHORT usFrameLen );
#endif

int64_t xMBTCPGetTimeStamp( );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
