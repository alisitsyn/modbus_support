#include "mbcontroller.h"   // for common Modbus defines
#include "string.h"
#include "esp_log.h"

#define MASTER_MAX_CIDS 2
#define MASTER_MAX_RETRY 100
#define MASTER_PORT_NUM 2
#define MASTER_SPEED 115200
#define MASTER_TAG "MODBUS_MASTER"
#define MB_UART_RXD_PIN 22
#define MB_UART_TXD_PIN 23
#define MB_UART_RTS_PIN 18

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

#define STR(fieldname) ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_DEV_REG0 = 0,
    CID_DEV_REG1
};

// Example Data (Object) Dictionary for Modbus parameters
const mb_parameter_descriptor_t device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-0"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-1"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

static esp_err_t read_modbus_parameter(uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0;
        err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%04x) parameter read successful.",
                                     param_descriptor->cid,
                                     (char*)param_descriptor->param_key,
                                     (char*)param_descriptor->param_units,
                                     *(uint16_t*)par_data);
        }
        else 
        {
            ESP_LOGE(MASTER_TAG, "Characteristic #%d %s (%s), parameter read fail.", 
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (char*)param_descriptor->param_units);
        }
    }
    return err;  
}

static esp_err_t write_modbus_parameter(uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0; // type of parameter from dictionary
        err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%04x), write successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        *(uint16_t*)par_data);
        } else {
            ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (int)err,
                                    (char*)esp_err_to_name(err));
        }
    }
    return err;  
}

// This is user function to read and write modbus holding registers
static void master_read_write_func(void *arg)
{
    esp_err_t err = ESP_OK;
    uint16_t register_data = 0;
    
    ESP_LOGI(MASTER_TAG, "Start modbus test...");
    
    for(uint16_t retry = 0; retry <= MASTER_MAX_RETRY; retry++) {
        // Simply read your register here CID_DEV_REG0 - one register from address 0 (see device_parameters)
        err = read_modbus_parameter(CID_DEV_REG0, &register_data);   
        if (err == ESP_OK) {
            // if read successful then increase value of the parameter 
            // Insert your modbus read_write register functionality here
            register_data += 1;
            err = write_modbus_parameter(CID_DEV_REG0, &register_data);   
        }
        err = read_modbus_parameter(CID_DEV_REG1, &register_data);   
        if (err == ESP_OK) {
            register_data += 1;
            err = write_modbus_parameter(CID_DEV_REG1, &register_data);   
        }
    }
    ESP_LOGI(MASTER_TAG, "Modbus test is completed.");
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MASTER_PORT_NUM,
            .mode = MB_MODE_RTU,
            .baudrate = MASTER_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                                "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MASTER_PORT_NUM, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                            MB_UART_RTS_PIN, UART_PIN_NO_CHANGE);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);
    // Set driver mode to Half Duplex
    err = uart_set_mode(MASTER_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);
    
    // Write registers to predefined state
    uint16_t register_data = 0x1234;
    write_modbus_parameter(CID_DEV_REG0, &register_data);
    register_data = 0x5678;
    write_modbus_parameter(CID_DEV_REG1, &register_data);

    master_read_write_func(NULL);
}
