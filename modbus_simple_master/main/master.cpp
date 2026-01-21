/*
 * SPDX-FileCopyrightText: 2021 - 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "string.h"
#include "esp_log.h"

#include "ModbusMaster.h"
#include "sdkconfig.h"

#define MASTER_TAG "MODBUS_MASTER"

#define MASTER_MAX_RETRY                (10)
#define MASTER_PORT_NUM                 (uart_port_t)(CONFIG_MB_UART_PORT_NUM)
#define MASTER_SPEED                    (CONFIG_MB_UART_BAUD_RATE)
#define MB_SLAVE_SHORT_ADDRESS          (1)
#define MB_UART_RXD_PIN                 (CONFIG_MB_UART_RXD)
#define MB_UART_TXD_PIN                 (CONFIG_MB_UART_TXD)
#define MB_UART_RTS_PIN                 (CONFIG_MB_UART_RTS)

#define TAG "MB_MASTER_MAIN"

// Example code to read and write Modbus registers using CPP wrapper class
extern "C" void app_main()
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm;
    comm.ser_opts.port = MASTER_PORT_NUM;
#if CONFIG_MB_COMM_MODE_ASCII
    comm.ser_opts.mode = MB_ASCII;
#elif CONFIG_MB_COMM_MODE_RTU
    comm.ser_opts.mode = MB_RTU;
#endif
    comm.ser_opts.baudrate = MASTER_SPEED;
    comm.ser_opts.parity = MB_PARITY_NONE;
    comm.ser_opts.uid = 0;
    comm.ser_opts.response_tout_ms = 1000;
    comm.ser_opts.data_bits = UART_DATA_8_BITS;
    comm.ser_opts.stop_bits = UART_STOP_BITS_1;
    ModbusMaster* pModbusMaster = new ModbusMaster(&comm);

    // Set UART pin numbers
    esp_err_t err = uart_set_pin(MASTER_PORT_NUM, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                            MB_UART_RTS_PIN, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG,
                        "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MASTER_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    uint16_t reg_holding_array[6] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666 };
    uint16_t reg_input_array[6] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666 };
    uint16_t temp_coils = 0xFFFF;

    // Initialization of device peripheral and objects
    err = pModbusMaster->begin(MB_SLAVE_SHORT_ADDRESS);
    MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG,
                            "mb controller initialization fail, returns(0x%x).", (int)err);

    for (int i = 0; i < MASTER_MAX_RETRY; i++) {
        err = pModbusMaster->writeMultipleRegisters(40001, 5, &reg_holding_array[0]);
        if ((err != ESP_OK)) {
            ESP_LOGE(TAG, "Modbus Write Holding registers error: (0x%x).", (int)err);
        } else {
            ESP_LOGI(TAG, "Modbus Write Holding registers successful.");
        }

        memset(reg_holding_array, 0x00, 10);
        err = pModbusMaster->readHoldingRegisters(40001, 5, &reg_holding_array[0]);
        if ((err != ESP_OK)) {
            ESP_LOGE(TAG, "Modbus Read Holding registers error: (0x%x).", (int)err);
        } else {
            ESP_LOGI(TAG, "Modbus Read Holding registers successful.");
        }
        ESP_LOG_BUFFER_HEX_LEVEL("HOLDING_REGS", reg_holding_array, 10, ESP_LOG_INFO);

        err = pModbusMaster->readInputRegisters(30001, 5, &reg_input_array[0]);
        if ((err != ESP_OK)) {
            ESP_LOGE(TAG, "Modbus Read Inputs error: (0x%x).", (int)err);
        } else {
            ESP_LOGI(TAG, "Modbus Read Inputs successful.");
        }

        temp_coils ^= 0xAAAA;

        err = pModbusMaster->writeMultipleCoils(00001, 10, &temp_coils);
        if ((err != ESP_OK)) {
            ESP_LOGE(TAG, "Modbus Write Multiple Coils error: (0x%x).", (int)err);
        } else {
            ESP_LOGI(TAG, "Modbus Write Multiple Coils successful.");
        }

        err = pModbusMaster->readCoils(00001, 10, &temp_coils);
        if ((err != ESP_OK)) {
            ESP_LOGE(TAG, "Modbus Read Coils error: (0x%x).", (int)err);
        } else {
            ESP_LOGI(TAG, "Modbus Read Coils successful.");
        }
        ESP_LOGI("COILS", "Actual coil value: 0x%X", temp_coils);

    }
    err = pModbusMaster->end();
    if ((err != ESP_OK)) {
        ESP_LOGW(TAG, "Modbus stop fail, or already stopped, error: (0x%x).", (int)err);
    }
    ESP_LOGI(TAG, "Modbus Test completed, destroy Moddbus controller.");

    delete pModbusMaster;
}

