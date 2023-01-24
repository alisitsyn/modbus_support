#include "mbcontroller.h"   // for common Modbus defines
#include "string.h"
#include "esp_log.h"

#define MASTER_MAX_CIDS 2
#define MASTER_MAX_UPDATES 3000
#define MASTER_MAX_RETRY 5
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

#pragma pack(push, 1)
typedef struct
{
    // runtime data
    uint64_t dt;
    uint16_t status;
    uint16_t dummy_reg1;
    uint16_t dummy_reg2;
} rt_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    // application info
    uint16_t boot_ver1;
    uint16_t boot_ver2;
    uint16_t app_fname1;
    uint16_t app_fname2;
    uint16_t app_fname3;
    uint16_t app_fname4;
    uint16_t app_fver1;
    uint16_t app_fver2;
    uint16_t app_fver3;
    uint16_t app_fver4;
    uint16_t dummy_reg;
} dev_info_params_t;
#pragma pack(pop) 


#define RT_DATA_REG_OFFSET 0
#define RT_DATA_SZ (sizeof(rt_params_t) >> 1)

#define APP_DATA_REG_OFFSET 100 // Set to known number //(RT_DATA_REG_OFFSET + sizeof(rt_params_t) + 4)
#define APP_DATA_REG_SZ (sizeof(dev_info_params_t) >> 1)


//static dev_info_params_t dev_info;

// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_DEV_RT_DATA = 0,
    CID_DEV_APP_INFO
};

// Example Data (Object) Dictionary for Modbus parameters
const mb_parameter_descriptor_t device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_RT_DATA, STR("Device_rt_data"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, RT_DATA_REG_OFFSET, RT_DATA_SZ,
                    0, PARAM_TYPE_ASCII, (RT_DATA_SZ << 1), OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_APP_INFO, STR("Dev_app_info"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, APP_DATA_REG_OFFSET, APP_DATA_REG_SZ,
                    0, PARAM_TYPE_ASCII, (APP_DATA_REG_SZ << 1), OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

static dev_info_params_t dev_data = {
    .boot_ver1 = 48,
    .boot_ver2 = 50,
    .app_fname1 = 77,
    .app_fname2 = 66,
    .app_fname3 = 49,
    .app_fname4 = 52,
    .app_fver1 = 52,
    .app_fver2 = 83,
    .app_fver3 = 48,
    .app_fver4 = 0,
};
static rt_params_t rt_data = {
    .status = 0, 
    .dt = 10
};

static esp_err_t read_modbus_parameter(uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0;
        err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s), REG(%d, %d), parameter read successful.",
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (char*)param_descriptor->param_units,
                                    param_descriptor->mb_reg_start,
                                    param_descriptor->mb_size
                                    );
            ESP_LOG_BUFFER_HEX_LEVEL("GET", par_data, param_descriptor->param_size, ESP_LOG_WARN);
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
            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s), REG(%d, %d), write successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        param_descriptor->mb_reg_start,
                                        param_descriptor->mb_size
                                        );
            ESP_LOG_BUFFER_HEX_LEVEL("SET", par_data, param_descriptor->param_size, ESP_LOG_WARN);
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

extern void vMBMasterRxFlush( void );

// This is user function to read and write modbus holding registers
static void master_read_write_func(void *arg)
{
    esp_err_t err = ESP_OK;
    int i = 0;
    
    ESP_LOGI(MASTER_TAG, "Start modbus test...");

    // Just try to set initial parameters, do not use the result yet
    write_modbus_parameter(CID_DEV_RT_DATA, (uint16_t*)&rt_data);
    write_modbus_parameter(CID_DEV_APP_INFO, (uint16_t*)&dev_data);
    
    for(uint16_t num_updates = 0; num_updates <= MASTER_MAX_UPDATES; num_updates++) {
        // read CID_DEV_RT_DATA - realtime data from device
        for(i = 0; i < MASTER_MAX_RETRY; i++){
            err = read_modbus_parameter(CID_DEV_RT_DATA, (uint16_t*)&rt_data);   
            if (err == ESP_OK) {
                break;
            } else {
                vMBMasterRxFlush();
                ESP_LOGW("DBG", "RT data retry: %d", i);
            }
        }

        // Read CID_DEV_APP_INFO - read device data
        for(i = 0; i < MASTER_MAX_RETRY; i++){
            err = read_modbus_parameter(CID_DEV_APP_INFO, (uint16_t*)&dev_data);   
            if (err == ESP_OK) {
                break;
            } else {
                vMBMasterRxFlush();
                ESP_LOGW("DBG", "APP info retry: %d", i);
            }
        }
    }
    ESP_LOGI(MASTER_TAG, "Modbus test is completed.");
    ESP_ERROR_CHECK(mbc_master_destroy());
    vTaskDelete(NULL);
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

    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}

static TaskHandle_t  task_handle = NULL;

void app_main(void)
{
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());

    BaseType_t xStatus = xTaskCreatePinnedToCore(master_read_write_func, "mb_task",
                                                    CONFIG_MB_SERIAL_TASK_STACK_SIZE,
                                                    NULL, (CONFIG_MB_SERIAL_TASK_PRIO - 2),
                                                    &task_handle, CONFIG_FMB_PORT_TASK_AFFINITY);
}
