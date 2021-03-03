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
#include <stdarg.h>
#include <string.h>

#include "port.h"
#include "mbport.h"
#include "esp_timer.h"
#include "port_tcp_slave.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_FRAME_LOG_BUFSIZE    512

/* ----------------------- Start implementation -----------------------------*/

/* ----------------------- Variables ----------------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

// The helper function to get time stamp in microseconds
int64_t xMBTCPGetTimeStamp()
{
    int64_t xTimeStamp = esp_timer_get_time();
    return xTimeStamp;
}

#ifdef MB_TCP_DEBUG
void
prvvMBTCPLogFrame( const CHAR * pucMsg, UCHAR * pucFrame, USHORT usFrameLen )
{
    int             i;
    int             res = 0;
    int             iBufPos = 0;
    size_t          iBufLeft = MB_FRAME_LOG_BUFSIZE;
    static CHAR     arcBuffer[MB_FRAME_LOG_BUFSIZE];

    assert( pucFrame != NULL );

    for ( i = 0; i < usFrameLen; i++ ) {
        // Print some additional frame information.
        switch ( i )
        {
        case 0:
            // TID = Transaction Identifier.
            res = snprintf( &arcBuffer[iBufPos], iBufLeft, "| TID = " );
            break;
        case 2:
            // PID = Protocol Identifier.
            res = snprintf( &arcBuffer[iBufPos], iBufLeft, " | PID = " );
            break;
        case 4:
            // Length
            res = snprintf( &arcBuffer[iBufPos], iBufLeft, " | LEN = " );
            break;
        case 6:
            // UID = Unit Identifier.
            res = snprintf( &arcBuffer[iBufPos], iBufLeft, " | UID = " );
            break;
        case 7:
            // MB Function Code.
            res = snprintf( &arcBuffer[iBufPos], iBufLeft, " | FUNC = " );
            break;
        case 8:
            // MB PDU rest.
            res = snprintf( &arcBuffer[iBufPos], iBufLeft, " | DATA = " );
            break;
        default:
            res = 0;
            break;
        }
        if( res == -1 ) {
            break;
        }
        else {
            iBufPos += res;
            iBufLeft -= res;
        }

        // Print the data.
        res = snprintf( &arcBuffer[iBufPos], iBufLeft, "%02X", pucFrame[i] );
        if( res == -1 ) {
            break;
        } else {
            iBufPos += res;
            iBufLeft -= res;
        }
    }

    if( res != -1 ) {
        // Append an end of frame string.
        res = snprintf( &arcBuffer[iBufPos], iBufLeft, " |\r\n" );
        if( res != -1 ) {
            vMBPortLog( MB_LOG_DEBUG, (const CHAR*)pucMsg, "%s", arcBuffer );
        }
    }
}

void
vMBPortLog( eMBPortLogLevel eLevel, const CHAR * szModule, const CHAR * szFmt, ... )
{
    va_list args;
    static const char *arszLevel2Str[] = { "DEBUG", "INFO", "WARN", "ERROR" };

    ( void )printf( "%s: %s: ", arszLevel2Str[eLevel], szModule );
    va_start( args, szFmt );
    vprintf( szFmt, args );
    va_end( args );
}
#endif
