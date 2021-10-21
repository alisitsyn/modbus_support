//#include "mbcontroller.h"   // for common Modbus defines
#include "string.h"
#include "esp_log.h"

#include "ModbusMaster.h"

#define MASTER_MAX_CIDS 2
#define MASTER_MAX_RETRY 100
#define MASTER_PORT_NUM 2
#define MASTER_SPEED 115200
#define MASTER_TAG "MODBUS_MASTER"
#define MB_UART_RXD_PIN 22
#define MB_UART_TXD_PIN 23
#define MB_UART_RTS_PIN 18

#ifdef __cplusplus
extern "C" {
#endif

#define TAG "MB_MASTER_MAIN"
#define MB_SLAVE_SHORT_ADDRESS 1
#define MB_RETRIES 50


// Example code to read and write Modbus registers using CPP wrapper class
void app_main()
{
    ModbusMaster* pModbusMaster = new ModbusMaster();
    
    // Initialize and start Modbus controller
    mb_communication_info_t comm;

    comm.port = MASTER_PORT_NUM;
    comm.mode = MB_MODE_RTU;
    comm.baudrate = MASTER_SPEED;
    comm.parity = MB_PARITY_NONE;


    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(pModbusMaster->begin(MB_SLAVE_SHORT_ADDRESS, &comm));

    // Set UART pin numbers
    uart_set_pin(MASTER_PORT_NUM, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                            MB_UART_RTS_PIN, UART_PIN_NO_CHANGE);

    // Set driver mode to Half Duplex
    esp_err_t err = uart_set_mode(MASTER_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ; ,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    uint16_t reg_array[6] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666 };
    uint16_t temp_coils = 0xFFFF;

    for (int i = 0; i < MB_RETRIES; i++) {
        err = pModbusMaster->writeMultipleRegisters(40001, 5, &reg_array[0]);
        MASTER_CHECK((err == ESP_OK), ; ,
                    "Modbus Write error: (0x%x).", (uint32_t)err);

        memset(reg_array, 0x00, 10);
        err = pModbusMaster->readHoldingRegisters(40001, 5, &reg_array[0]);
        MASTER_CHECK((err == ESP_OK), ; ,
                    "Modbus Read error: (0x%x).", (uint32_t)err);

        temp_coils ^= 0xAAAA;

        err = pModbusMaster->writeMultipleCoils(00001, 10, &temp_coils);
        MASTER_CHECK((err == ESP_OK), ; ,
                            "Modbus Write Coils error: (0x%x).", (uint32_t)err);

        err = pModbusMaster->readCoils(00001, 10, &temp_coils);
        MASTER_CHECK((err == ESP_OK), ; ,
                                    "Modbus Read Coils error: (0x%x).", (uint32_t)err);

        ESP_LOGI("MB", "COILS: 0x%X", temp_coils);

        ESP_LOG_BUFFER_HEX_LEVEL("MB", reg_array, 10, ESP_LOG_INFO);

    }
    ESP_LOGI("MODBUS", "Modbus Test completed.");
    delete pModbusMaster;
}


#ifdef __cplusplus
}
#endif
