/*
 * SPDX-FileCopyrightText: 2021 - 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* CPP Wrapper Slave Example ESP32

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "mbcontroller.h"       // for mbcontroller defines and api
#include "esp_log.h"            // for log_write
#include "esp_random.h"
#include "sdkconfig.h"

#include "ModbusSlave.h"

#define MB_RETRIES      (100)
#define MB_PORT_NUM     ((uart_port_t)CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR   (CONFIG_MB_SLAVE_ADDR)      // The address of device in Modbus network
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  ((mb_event_group_t)(MB_READ_MASK | MB_WRITE_MASK))

static const char *TAG = "SLAVE_TEST";

// An example application of Modbus CPP Slave.
extern "C" void app_main(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm;

    // Setup communication parameters and start stack
    #if CONFIG_MB_COMM_MODE_ASCII
        comm.ser_opts.mode = MB_ASCII;
    #elif CONFIG_MB_COMM_MODE_RTU
        comm.ser_opts.mode = MB_RTU;
    #endif
    comm.ser_opts.uid = MB_SLAVE_ADDR;
    comm.ser_opts.port = MB_PORT_NUM;
    comm.ser_opts.baudrate = MB_DEV_SPEED;
    comm.ser_opts.parity = MB_PARITY_NONE;

    // Create the class instance
    ModbusSlave *pModbusSlave = new ModbusSlave(&comm);

    // Set UART pin numbers
    uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                                    CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    // Set driver mode to Half Duplex
    esp_err_t err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG,
                    "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);
    
    // The register areas defined to store registers data
    uint16_t hold_array[6] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666 };
    uint16_t input_array[6] = { 0xAAAA, 0x9999, 0x8888, 0x7777, 0x6666, 0x5555 };
    uint16_t coil_array[6] = { 0xAAAA, 0xFFFF, 0xAAAA, 0xFFFF, 0xAAAA, 0xFFFF };
    size_t reg_size = (sizeof(hold_array) / sizeof(hold_array[0]));
    
    // Add registers below into assiciated register bank (use PLC based address)
    // These registers are accessed inside this defined area only (exception returned outside of this range).
    // Make sure the Master reads/writes to these areas!!!
    pModbusSlave->addRegisterBank(40001, &hold_array[0], reg_size);
    pModbusSlave->addRegisterBank(30001, &input_array[0], (sizeof(input_array) << 1));
    pModbusSlave->addRegisterBank(1, &coil_array[0], (sizeof(coil_array) * 2 * 8));

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(pModbusSlave->begin());

    for (int i = 0; i < MB_RETRIES; i++) {
        ESP_LOG_BUFFER_HEX_LEVEL("Holding Register bank", hold_array, sizeof(hold_array), ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEX_LEVEL("Input Register bank", input_array, sizeof(input_array), ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEX_LEVEL("Coil Register bank", input_array, sizeof(coil_array), ESP_LOG_INFO);
        
        // Wait for update events when access from Modbus Master is completed (register areas accessed in wr/rd commands)
        mb_event_group_t event = pModbusSlave->run(MB_READ_WRITE_MASK);
        if (!(event & MB_READ_WRITE_MASK)) {
            ESP_LOGE(TAG, "Incorrect modbus access type: %d.", event);
        }
        // Get register access information
        mb_param_info_t info = pModbusSlave->getRegisterInfo();
        ESP_LOGW(TAG, "Got Event: %d.", event);
        const char* rw_str = (info.type & MB_READ_MASK) ? "READ" : "WRITE";
        
        // Check if the selected holding register area is accessed by Master
        if (pModbusSlave->isBankAccessed(40001, reg_size)) {
            CRITICAL_SECT(pModbusSlave->getInstance()) {
                ESP_LOGW(TAG, "Holding bank address offset: %d, size %d, %s accessed.", info.mb_offset, info.size, rw_str);
                esp_fill_random(info.address, (info.size << 1));
            }
        }
        
        // // Check if the selected input register area is accessed by Master
        if (pModbusSlave->isBankAccessed(30001, reg_size)) {
            ESP_LOGW(TAG, "Input bank address offset: %d, size %d, %s accessed.", info.mb_offset, info.size, rw_str);
            CRITICAL_SECT(pModbusSlave->getInstance()) {
                // Reinitialize the accessed register values with random numbers
                esp_fill_random(info.address, (info.size << 1));
            }
        }
        
        if (pModbusSlave->isBankAccessed(1, 10)) {
            ESP_LOGW(TAG, "Coils address offset: %d, size %d, %s accessed.", info.mb_offset, info.size, rw_str);
            CRITICAL_SECT(pModbusSlave->getInstance()) {
                // Reinitialize the accessed register values with random numbers
                esp_fill_random(info.address, (info.size >> 3));
            }
        }
        vTaskDelay(1);
    }
    ESP_LOGI("MODBUS", "Modbus Test completed.");
    delete pModbusSlave;
}

