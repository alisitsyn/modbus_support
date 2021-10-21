/* FreeModbus Slave Example ESP32

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "mbcontroller.h"       // for mbcontroller defines and api
#include "esp_log.h"            // for log_write
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ModbusSlave.h"

#define MB_RETRIES      (100)
#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
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
#define MB_READ_WRITE_MASK                  (mb_event_group_t)(MB_READ_MASK | MB_WRITE_MASK)

static const char *SLAVE_TAG = "SLAVE_TEST";

static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

// An example application of Modbus slave. It is based on freemodbus stack.
// See deviceparams.h file for more information about assigned Modbus parameters.
// These parameters can be accessed from main application and also can be changed
// by external Modbus master host.
void app_main(void)
{
    // Create the class instance
    ModbusSlave* pModbusSlave = new ModbusSlave();
    
    // Initialize and start Modbus controller
    mb_communication_info_t comm;

    // Setup communication parameters and start stack
    #if CONFIG_MB_COMM_MODE_ASCII
        comm.mode = MB_MODE_ASCII,
    #elif CONFIG_MB_COMM_MODE_RTU
        comm.mode = MB_MODE_RTU,
    #endif

    comm.slave_addr = MB_SLAVE_ADDR;
    comm.port = MB_PORT_NUM;
    comm.baudrate = MB_DEV_SPEED;
    comm.parity = MB_PARITY_NONE;

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(pModbusSlave->begin(&comm));

    // Set UART pin numbers
    uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                                    CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    // Set driver mode to Half Duplex
    esp_err_t err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    SLAVE_CHECK((err == ESP_OK), ; ,
                    "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);
    
    // The register areas defined to store registers data
    uint16_t hold_array[6] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666 };
    uint16_t input_array[6] = { 0xAAAA, 0x9999, 0x8888, 0x7777, 0x6666, 0x5555 };
    uint16_t coil_array[6] = { 0xAAAA, 0xFFFF, 0xAAAA, 0xFFFF, 0xAAAA, 0xFFFF };
    size_t reg_size = (sizeof(hold_array) / sizeof(hold_array[0]));
    
    // Add registers below into assiciated register bank (use PLC based address)
    // These registers accessed inside this defined area only (exception returned outside of this range).
    // Make sure the Master reads/writes to these areas!!!
    pModbusSlave->addRegisterBank(40001, &hold_array[0], reg_size);
    pModbusSlave->addRegisterBank(30001, &input_array[0], reg_size);
    pModbusSlave->addRegisterBank(1, &coil_array[0], (reg_size * 2 * 8));

    for (int i = 0; i < MB_RETRIES; i++) {
        ESP_LOG_BUFFER_HEX_LEVEL("Holding Register bank", hold_array, sizeof(hold_array), ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEX_LEVEL("Input Register bank", input_array, sizeof(input_array), ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEX_LEVEL("Coil Register bank", input_array, sizeof(coil_array), ESP_LOG_INFO);
        
        // Wait for update events when access from Modbus Master is completed (register areas accessed in wr/rd commands)
        mb_event_group_t event = pModbusSlave->run(MB_READ_WRITE_MASK);
        if (!(event & MB_READ_WRITE_MASK)) {
            ESP_LOGI(SLAVE_TAG, "Incorrect modbus access type: %d.", event);
        }
        
        // Get register access information
        mb_param_info_t info = pModbusSlave->getRegisterInfo();
        
        // Check if the selected holding register area is accessed by Master
        if (pModbusSlave->isBankAccessed(40001, reg_size)) {    
            ESP_LOGI(SLAVE_TAG, "Holding bank address offset: %d, size %d is accessed.", info.mb_offset, info.size);
            portENTER_CRITICAL(&param_lock);
            // Reinitialize the accessed register values with random numbers
            esp_fill_random(info.address, (info.size << 1));
            portEXIT_CRITICAL(&param_lock);
        }
        
        // Check if the selected input register area is accessed by Master
        if (pModbusSlave->isBankAccessed(30001, reg_size)) {
            ESP_LOGI(SLAVE_TAG, "Input bank address offset: %d, size %d is accessed.", info.mb_offset, info.size);
            portENTER_CRITICAL(&param_lock);
            // Reinitialize the accessed register values with random numbers
            esp_fill_random(info.address, (info.size << 1));
            portEXIT_CRITICAL(&param_lock);
        }
        
        if (pModbusSlave->isBankAccessed(1, 8)) {
            ESP_LOGI(SLAVE_TAG, "Cols address offset: %d, size %d is accessed.", info.mb_offset, info.size);
            portENTER_CRITICAL(&param_lock);
            // Reinitialize the accessed register values with random numbers
            esp_fill_random(info.address, (info.size >> 3));
            portEXIT_CRITICAL(&param_lock);
        }
        vTaskDelay(1);
    }
    ESP_LOGI("MODBUS", "Modbus Test completed.");
    delete pModbusSlave;
}

#ifdef __cplusplus
}
#endif
