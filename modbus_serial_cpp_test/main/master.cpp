#include "string.h"
#include "esp_log.h"

#define MASTER_MAX_CIDS 2
#define MASTER_MAX_RETRY 100
#define MASTER_PORT_NUM (uart_port_t)2
#define MASTER_SPEED 115200
#define MASTER_TAG "MODBUS_MASTER"
#define MB_UART_RXD_PIN 22
#define MB_UART_TXD_PIN 23
#define MB_UART_RTS_PIN 18

#define TAG "MB_MASTER_MAIN"
#define MB_SLAVE_SHORT_ADDRESS 1
#define MB_RETRIES 50

#include "sdkconfig.h"

#include "mbcontroller.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    MB_DEVICE_ADDR1 = 1
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_DEV_REG0 = 0,
    CID_DEV_REG1
};


#define STR(fieldname) ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

static void *master_handle = NULL;

// Example Data (Object) Dictionary for Modbus parameters
const mb_parameter_descriptor_t device_parameters[2] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-0"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2, 1,
                    0, PARAM_TYPE_U16, PARAM_SIZE_U16, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-1"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 1,
                    0, PARAM_TYPE_U16, PARAM_SIZE_U16, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize Modbus controller
    mb_communication_info_t comm;
    comm.ser_opts.port = (uart_port_t)MASTER_PORT_NUM;
    comm.ser_opts.mode = (mb_comm_mode_t)MB_RTU;
    comm.ser_opts.baudrate = MASTER_SPEED;
    comm.ser_opts.parity = MB_PARITY_NONE;
    comm.ser_opts.uid = 0;
    comm.ser_opts.response_tout_ms = 1000;
    comm.ser_opts.data_bits = UART_DATA_8_BITS;
    comm.ser_opts.stop_bits = UART_STOP_BITS_1;

    esp_err_t err = mbc_master_create_serial(&comm, &master_handle);
    MB_RETURN_ON_FALSE((master_handle != NULL), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller initialization fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
    err = uart_set_pin(MASTER_PORT_NUM, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                              MB_UART_RTS_PIN, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                        "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    err = mbc_master_start(master_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MASTER_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    err = mbc_master_set_descriptor(master_handle, &device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller set descriptor fail, returns(0x%x).", (int)err);

    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
}

#ifdef __cplusplus
}
#endif
