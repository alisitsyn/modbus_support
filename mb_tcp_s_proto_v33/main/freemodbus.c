/* FreeModbus Slave Example ESP32

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_log.h"            // for log_write

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "mbcontroller.h"       // for mbcontroller defines and api
#include "deviceparams.h"       // for device parameters structures

// The example use simple WiFi configuration that you can set via
//   'make menuconfig'.
//   If you'd rather not, just change the below entries to strings with
//   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"

#define EXAMPLE_WIFI_SSID   CONFIG_ESP_WIFI_SSID
#define EXAMPLE_WIFI_PASS   CONFIG_ESP_WIFI_PASSWORD

#define MB_TCP_PORT_NUMBER  CONFIG_MB_EXAMPLE_PORT

// FreeRTOS event group to signal when we are connected & ready to make a request //
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

// Defines below are used to define register start address for each type of Modbus registers
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_INPUT_START                  (0x0000)
#define MB_REG_HOLDING_START                (0x0000)
#define MB_REG_COILS_START                  (0x0000)

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL                (10)
#define MB_CHAN_DATA_OFFSET                 (0.01f)

static const char *TAG = "MODBUS_SLAVE_APP";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                         ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");
        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    EventBits_t event = xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    if (event & IPV4_GOTIP_BIT) {
        ESP_LOGI(TAG, "Connected to AP with IPV4 address.");
    } else if (event & IPV6_GOTIP_BIT) {
        ESP_LOGI(TAG, "Connected to AP with IPV6 address.");
    }
}

// Set register values into known state
static void setup_reg_data()
{
    // Define initial state of parameters
    discrete_reg_params.discrete_input1 = 1;
    discrete_reg_params.discrete_input3 = 1;
    discrete_reg_params.discrete_input5 = 1;
    discrete_reg_params.discrete_input7 = 1;

    holding_reg_params.data_chan0 = 1.34;
    holding_reg_params.data_chan1 = 2.56;
    holding_reg_params.data_chan2 = 3.78;
    holding_reg_params.data_chan3 = 4.90;

    coil_reg_params.coil0 = 1;
    coil_reg_params.coil2 = 1;
    coil_reg_params.coil4 = 1;
    coil_reg_params.coil6 = 1;
    coil_reg_params.coil7 = 1;

    input_reg_params.data_chan0 = 1.34;
    input_reg_params.data_chan1 = 2.56;
    input_reg_params.data_chan2 = 3.78;
    input_reg_params.data_chan3 = 4.90;
}

// An example application of Modbus slave. It is based on freemodbus stack.
// See deviceparams.h file for more information about assigned Modbus parameters.
// These parameters can be accessed from main application and also can be changed
// by external Modbus master host.
void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    initialise_wifi();

    mb_param_info_t reg_info; // keeps the Modbus registers access information
    mb_communication_info_t comm_info; // Modbus communication parameters
    mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

    wait_for_ip();

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    void* mbc_slave_handler = NULL;

    ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_TCP_SLAVE, &mbc_slave_handler)); // Initialization of Modbus controller

    // Setup communication parameters and start stack
    comm_info.mode = MB_MODE_RTU;
    comm_info.tcp_port = MB_TCP_PORT_NUMBER;
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbcontroller_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.
    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START; // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_params; // Set pointer to storage instance
    reg_area.size = sizeof(holding_reg_params); // Set the size of register storage instance
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Initialization of Input Registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START;
    reg_area.address = (void*)&input_reg_params;
    reg_area.size = sizeof(input_reg_params);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Initialization of Coils register area
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address = (void*)&coil_reg_params;
    reg_area.size = sizeof(coil_reg_params);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Initialization of Discrete Inputs register area
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_INPUT_START;
    reg_area.address = (void*)&discrete_reg_params;
    reg_area.size = sizeof(discrete_reg_params);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    setup_reg_data(); // Set values into known state

    // Starts of modbus controller and stack
    ESP_ERROR_CHECK(mbc_slave_start());

    // The cycle below will be terminated when parameter holdingRegParams.dataChan0
    // incremented each access cycle reaches the CHAN_DATA_MAX_VAL value.
    for(;holding_reg_params.data_chan0 < MB_CHAN_DATA_MAX_VAL;){
        // Check for read/write events of Modbus master for certain events
        mb_event_group_t event = mbc_slave_check_event((MB_EVENT_HOLDING_REG_WR
                                                        | MB_EVENT_INPUT_REG_RD
                                                        | MB_EVENT_HOLDING_REG_RD
                                                        | MB_EVENT_DISCRETE_RD
                                                        | MB_EVENT_COILS_RD));
        // Filter events and process them accordingly
        if((event & MB_EVENT_HOLDING_REG_WR) || (event & MB_EVENT_HOLDING_REG_RD)) {
            // Get parameter information from parameter queue
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("HOLDING READ/WRITE: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                    (uint32_t)reg_info.time_stamp,
                    (uint32_t)reg_info.mb_offset,
                    (uint32_t)reg_info.type,
                    (uint32_t)reg_info.address,
                    (uint32_t)reg_info.size);
            if (reg_info.address == (uint8_t*)&holding_reg_params.data_chan0) {
                holding_reg_params.data_chan0 += MB_CHAN_DATA_OFFSET;
            }
        } else if (event & MB_EVENT_INPUT_REG_RD) {
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("INPUT READ: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                    (uint32_t)reg_info.time_stamp,
                    (uint32_t)reg_info.mb_offset,
                    (uint32_t)reg_info.type,
                    (uint32_t)reg_info.address,
                    (uint32_t)reg_info.size);
        } else if (event & MB_EVENT_DISCRETE_RD) {
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("DISCRETE READ: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                                (uint32_t)reg_info.time_stamp,
                                (uint32_t)reg_info.mb_offset,
                                (uint32_t)reg_info.type,
                                (uint32_t)reg_info.address,
                                (uint32_t)reg_info.size);
        } else if (event & MB_EVENT_COILS_RD) {
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("COILS READ: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                                (uint32_t)reg_info.time_stamp,
                                (uint32_t)reg_info.mb_offset,
                                (uint32_t)reg_info.type,
                                (uint32_t)reg_info.address,
                                (uint32_t)reg_info.size);
        }
    }
    // Destroy of Modbus controller once get maximum value of data_chan0
    printf("Modbus controller destroyed.\r\n");
    ESP_ERROR_CHECK(mbc_slave_destroy());
}
