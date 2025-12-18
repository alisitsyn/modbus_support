/*
 * SPDX-FileCopyrightText: 2016-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// FreeModbus Slave Example ESP32

#include <stdio.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mdns.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "mbcontroller.h"       // for mbcontroller defines and api
#include "modbus_params.h"      // for modbus parameters structures

#define MB_TCP_PORT_NUMBER      (CONFIG_FMB_TCP_PORT_DEFAULT)
#define MB_MDNS_PORT            (502)

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_INPUT_START_AREA0            (INPUT_OFFSET(input_data0)) // register offset input area 0
#define MB_REG_INPUT_START_AREA1            (INPUT_OFFSET(input_data4)) // register offset input area 1
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(holding_data0))
#define MB_REG_HOLDING_START_AREA1          (HOLD_OFFSET(holding_data4))

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL                (10)
#define MB_CHAN_DATA_OFFSET                 (1.1f)

#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)

static const char *TAG = "SLAVE_TEST";

static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

#if CONFIG_MB_MDNS_IP_RESOLVER

#define MB_ID_BYTE0(id) ((uint8_t)(id))
#define MB_ID_BYTE1(id) ((uint8_t)(((uint16_t)(id) >> 8) & 0xFF))
#define MB_ID_BYTE2(id) ((uint8_t)(((uint32_t)(id) >> 16) & 0xFF))
#define MB_ID_BYTE3(id) ((uint8_t)(((uint32_t)(id) >> 24) & 0xFF))

#define MB_ID2STR(id) MB_ID_BYTE0(id), MB_ID_BYTE1(id), MB_ID_BYTE2(id), MB_ID_BYTE3(id)

#if CONFIG_FMB_CONTROLLER_SLAVE_ID_SUPPORT
#define MB_DEVICE_ID (uint32_t)CONFIG_FMB_CONTROLLER_SLAVE_ID
#endif

#define MB_SLAVE_ADDR (CONFIG_MB_SLAVE_ADDR)

#define MB_MDNS_INSTANCE(pref) pref"mb_slave_tcp"

// convert mac from binary format to string
static inline char* gen_mac_str(const uint8_t* mac, char* pref, char* mac_str)
{
    sprintf(mac_str, "%s%02X%02X%02X%02X%02X%02X", pref, MAC2STR(mac));
    return mac_str;
}

static inline char* gen_id_str(char* service_name, char* slave_id_str)
{
    sprintf(slave_id_str, "%s%02X%02X%02X%02X", service_name, MB_ID2STR(MB_DEVICE_ID));
    return slave_id_str;
}

static inline char* gen_host_name_str(char* service_name, char* name)
{
    sprintf(name, "%s_%02X", service_name, MB_SLAVE_ADDR);
    return name;
}

static void start_mdns_service(void)
{
    char temp_str[32] = {0};
    uint8_t sta_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(sta_mac, ESP_MAC_WIFI_STA));
    char* hostname = gen_host_name_str(MB_MDNS_INSTANCE(""), temp_str);
    //initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    //set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));
    ESP_LOGI(TAG, "mdns hostname set to: [%s]", hostname);

    //set default mDNS instance name
    ESP_ERROR_CHECK(mdns_instance_name_set(MB_MDNS_INSTANCE("esp32_")));

    //structure with TXT records
    mdns_txt_item_t serviceTxtData[] = {
        {"board","esp32"}
    };

    //initialize service
    ESP_ERROR_CHECK(mdns_service_add(hostname, "_modbus", "_tcp", MB_MDNS_PORT, serviceTxtData, 1));
    //add mac key string text item
    ESP_ERROR_CHECK(mdns_service_txt_item_set("_modbus", "_tcp", "mac", gen_mac_str(sta_mac, "\0", temp_str)));
    //add slave id key txt item
    ESP_ERROR_CHECK( mdns_service_txt_item_set("_modbus", "_tcp", "mb_id", gen_id_str("\0", temp_str)));
}

static void stop_mdns_service(void)
{
    mdns_free();
}

#endif

// Set register values into known state
static void setup_reg_data(void)
{
    // Define initial state of parameters
    discrete_reg_params.discrete_input0 = 1;
    discrete_reg_params.discrete_input1 = 0;
    discrete_reg_params.discrete_input2 = 1;
    discrete_reg_params.discrete_input3 = 0;
    discrete_reg_params.discrete_input4 = 1;
    discrete_reg_params.discrete_input5 = 0;
    discrete_reg_params.discrete_input6 = 1;
    discrete_reg_params.discrete_input7 = 0;

    holding_reg_params.holding_data0 = 1.34;
    holding_reg_params.holding_data1 = 2.56;
    holding_reg_params.holding_data2 = 3.78;
    holding_reg_params.holding_data3 = 4.90;

    holding_reg_params.holding_data4 = 5.67;
    holding_reg_params.holding_data5 = 6.78;
    holding_reg_params.holding_data6 = 7.79;
    holding_reg_params.holding_data7 = 8.80;
    coil_reg_params.coils_port0 = 0x55;
    coil_reg_params.coils_port1 = 0xAA;

    input_reg_params.input_data0 = 1.12;
    input_reg_params.input_data1 = 2.34;
    input_reg_params.input_data2 = 3.56;
    input_reg_params.input_data3 = 4.78;
    input_reg_params.input_data4 = 1.12;
    input_reg_params.input_data5 = 2.34;
    input_reg_params.input_data6 = 3.56;
    input_reg_params.input_data7 = 4.78;
}

static void slave_operation_func(void *arg)
{
    mb_param_info_t reg_info; // keeps the Modbus registers access information

    ESP_LOGI(TAG, "Modbus slave stack initialized.");
    ESP_LOGI(TAG, "Start modbus test...");
    // The cycle below will be terminated when parameter holding_data0
    // incremented each access cycle reaches the CHAN_DATA_MAX_VAL value.
    for(;holding_reg_params.holding_data0 < MB_CHAN_DATA_MAX_VAL;) {
        // Check for read/write events of Modbus master for certain events
        mb_event_group_t event = mbc_slave_check_event(MB_READ_WRITE_MASK);
        const char* rw_str = (event & MB_READ_MASK) ? "READ" : "WRITE";
        // Filter events and process them accordingly
        if(event & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD)) {
            // Get parameter information from parameter queue
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            ESP_LOGI(TAG, "HOLDING %s (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%.4x, SIZE:%u",
                    rw_str,
                    (uint32_t)reg_info.time_stamp,
                    (uint32_t)reg_info.mb_offset,
                    (uint32_t)reg_info.type,
                    (uint32_t)reg_info.address,
                    (uint32_t)reg_info.size);
            if (reg_info.address == (uint8_t*)&holding_reg_params.holding_data0)
            {
                portENTER_CRITICAL(&param_lock);
                holding_reg_params.holding_data0 += MB_CHAN_DATA_OFFSET;
                if (holding_reg_params.holding_data0 >= (MB_CHAN_DATA_MAX_VAL - MB_CHAN_DATA_OFFSET)) {
                    coil_reg_params.coils_port1 = 0xFF;
                }
                portEXIT_CRITICAL(&param_lock);
            }
        } else if (event & MB_EVENT_INPUT_REG_RD) {
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            ESP_LOGI(TAG, "INPUT READ (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%.4x, SIZE:%u",
                    (uint32_t)reg_info.time_stamp,
                    (uint32_t)reg_info.mb_offset,
                    (uint32_t)reg_info.type,
                    (uint32_t)reg_info.address,
                    (uint32_t)reg_info.size);
        } else if (event & MB_EVENT_DISCRETE_RD) {
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            ESP_LOGI(TAG, "DISCRETE READ (%u us): ADDR:%u, TYPE:%u, INST_ADDR:0x%.4x, SIZE:%u",
                                (uint32_t)reg_info.time_stamp,
                                (uint32_t)reg_info.mb_offset,
                                (uint32_t)reg_info.type,
                                (uint32_t)reg_info.address,
                                (uint32_t)reg_info.size);
        } else if (event & (MB_EVENT_COILS_RD | MB_EVENT_COILS_WR)) {
            ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            ESP_LOGI(TAG, "COILS %s (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%.4x, SIZE:%u",
                                rw_str,
                                (uint32_t)reg_info.time_stamp,
                                (uint32_t)reg_info.mb_offset,
                                (uint32_t)reg_info.type,
                                (uint32_t)reg_info.address,
                                (uint32_t)reg_info.size);
            if (coil_reg_params.coils_port1 == 0xFF) break;
        }
    }
    // Destroy of Modbus controller on alarm
    ESP_LOGI(TAG,"Modbus controller destroyed.");
    vTaskDelay(100);
}

#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "lwip/netif.h"
#include "esp_netif_types.h"

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}
#endif // CONFIG_ETH_USE_SPI_ETHERNET

#if CONFIG_EXAMPLE_USE_SPI_ETHERNET
#define INIT_SPI_ETH_MODULE_CONFIG(eth_module_config, num)                                      \
    do {                                                                                        \
        eth_module_config[num].spi_cs_gpio = CONFIG_EXAMPLE_ETH_SPI_CS ##num## _GPIO;           \
        eth_module_config[num].int_gpio = CONFIG_EXAMPLE_ETH_SPI_INT ##num## _GPIO;             \
        eth_module_config[num].phy_reset_gpio = CONFIG_EXAMPLE_ETH_SPI_PHY_RST ##num## _GPIO;   \
        eth_module_config[num].phy_addr = CONFIG_EXAMPLE_ETH_SPI_PHY_ADDR ##num;                \
    } while(0)

typedef struct {
    uint8_t spi_cs_gpio;
    uint8_t int_gpio;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
}spi_eth_module_config_t;
#endif


esp_netif_t *eth_netif_spi = { NULL };

static void init_ethernet()
{
    esp_netif_ip_info_t info_ip; // called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
 
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
 
    // Create instance(s) of esp-netif for SPI Ethernet(s)
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config.if_desc = "eth0";
    esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    //esp_netif_t *eth_netif_spi = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    itoa(0, num_str, 10);
    strcat(strcpy(if_key_str, "ETH_SPI_"), num_str);
    strcat(strcpy(if_desc_str, "eth"), num_str);
    esp_netif_config.if_key = if_key_str;
    esp_netif_config.if_desc = if_desc_str;
    esp_netif_config.route_prio = 30;
    eth_netif_spi = esp_netif_new(&cfg_spi);
 
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif_spi));

    char* ip= "192.168.32.106";
    char* gateway = "192.168.32.3";
    char* netmask = "255.255.254.0";
    esp_netif_ip_info_t info_t;
    memset(&info_t, 0, sizeof(esp_netif_ip_info_t));
    info_t.ip.addr = esp_ip4addr_aton((const char *)ip);
    info_t.gw.addr = esp_ip4addr_aton((const char *)gateway);
    info_t.netmask.addr = esp_ip4addr_aton((const char *)netmask);
    esp_netif_set_ip_info(eth_netif_spi, &info_t);  
 
    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config_spi = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config_spi = ETH_PHY_DEFAULT_CONFIG();
 
    // Init SPI bus
    spi_device_handle_t spi_handle = { NULL };
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_12,
        .mosi_io_num = GPIO_NUM_13,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
 
    // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
    gpio_install_isr_service(0);
 
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, 1)); //
 
    // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO, Interrupt GPIO, etc.)
    spi_eth_module_config_t spi_eth_module_config = {
        .spi_cs_gpio = GPIO_NUM_15,
        .phy_reset_gpio = GPIO_NUM_5,
        .int_gpio = GPIO_NUM_4,
        .phy_addr = 1
    };
 
    // Configure SPI interface and Ethernet driver for specific SPI module
    esp_eth_mac_t *mac_spi;
    esp_eth_phy_t *phy_spi;
    esp_eth_handle_t eth_handle_spi = { NULL };
 
    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = 12 * 1000 * 1000,
        .queue_size = 20
    };
 
    // Set SPI module Chip Select GPIO
    devcfg.spics_io_num = spi_eth_module_config.spi_cs_gpio;
 
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));
 
    // w5500 ethernet driver is based on spi driver
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
 
    // Set remaining GPIO numbers and configuration used by the SPI module
    w5500_config.int_gpio_num = spi_eth_module_config.int_gpio;
    phy_config_spi.phy_addr = spi_eth_module_config.phy_addr;
    phy_config_spi.reset_gpio_num = spi_eth_module_config.phy_reset_gpio;
 
    mac_spi = esp_eth_mac_new_w5500(&w5500_config, &mac_config_spi);
    phy_spi = esp_eth_phy_new_w5500(&phy_config_spi);
 
    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi, phy_spi);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi));
 
    /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
    02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.*/
 
 
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle_spi, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
        0x02, 0x34, 0xDF, 0x56, 0x58, 0xE2
    }));
 
    // attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_spi, esp_eth_new_netif_glue(eth_handle_spi)));
 
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
 
 
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi));

#if CONFIG_MB_MDNS_IP_RESOLVER
    // Start mdns service and register device
    start_mdns_service();
#endif
 
    // mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
 
    // void* slave_handler = NULL;
 
    //  // Initialization of Modbus controller
    // esp_err_t err = mbc_slave_init_tcp(&slave_handler);
    // MB_RETURN_ON_FALSE((err == ESP_OK && slave_handler != NULL), ESP_ERR_INVALID_STATE,
    //                             SLAVE_TAG,
    //                             "mb controller initialization fail.");
 
    // mb_communication_info_t comm_info = {
    //     .ip_port = MB_TCP_SLAVE_PORT,                    // Modbus TCP port number (default = 502)
    //     .ip_addr_type = MB_IPV4,                   // version of IP protocol
    //     .ip_mode = MB_MODE_TCP,                    // Port communication mode
    //     .ip_addr = NULL,                           // This field keeps the client IP address to bind, NULL - bind to any client
    //     .ip_netif_ptr = (void*)eth_netif_spi     // esp_netif_ptr - pointer to the corresponding network interface
    // };
 
    // // Setup communication parameters and start stack
    // err = mbc_slave_setup((void*)&comm_info);
    // MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
    //                                     SLAVE_TAG,
    //                                     "mbc_slave_setup fail, returns(0x%x).",
    //                                     (uint32_t)err);
 
    // reg_area.type = MB_PARAM_HOLDING;                               // Set type of register area
    // reg_area.start_offset = MB_REG_HOLDING_START_AREA0;             // Offset of register area in Modbus protocol
    // reg_area.address = (void*)&holding_reg_area[0];                 // Set pointer to storage instance
    // reg_area.size = sizeof(holding_reg_area) << 1;                  // Set the size of register storage area in bytes
    // ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
    // MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
    //                                 SLAVE_TAG,
    //                                 "mbc_slave_set_descriptor fail, returns(0x%x).",
    //                                 (uint32_t)err);
 
    // // Starts of modbus controller and stack
    // err = mbc_slave_start();
    // MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
    //                                     SLAVE_TAG,
    //                                     "mbc_slave_start fail, returns(0x%x).",
    //                                     (uint32_t)err);
    // vTaskDelay(5);
 
    // printf("modbusTCP initialized\n");
    // // esp_netif_get_ip_info(eth_handle_spi, &ip_info);
    // // ESP_LOGI(SLAVE_TAG, "ip: " IPSTR ", mask: " IPSTR ", gw: " IPSTR,
    // //         IP2STR(&ip_info.ip),
    // //         IP2STR(&ip_info.netmask),
    // //         IP2STR(&ip_info.gw));
    // esp_netif_get_ip_info(eth_netif_spi, &ip_info);
    // ESP_LOGI(SLAVE_TAG, "ip: " IPSTR ", mask: " IPSTR ", gw: " IPSTR,
    //         IP2STR(&ip_info.ip),
    //         IP2STR(&ip_info.netmask),
    //         IP2STR(&ip_info.gw));
   
    // // holding_reg_area[0] = 0x01;
    // // holding_reg_area[1] = 0x02;
   
    //return err;
}

static esp_err_t init_services(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      result = nvs_flash_init();
    }
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                            TAG,
                            "nvs_flash_init fail, returns(0x%x).",
                            (uint32_t)result);
    result = esp_netif_init();
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                            TAG,
                            "esp_netif_init fail, returns(0x%x).",
                            (uint32_t)result);
    result = esp_event_loop_create_default();
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                            TAG,
                            "esp_event_loop_create_default fail, returns(0x%x).",
                            (uint32_t)result);
#if CONFIG_MB_MDNS_IP_RESOLVER
    // Start mdns service and register device
    start_mdns_service();
#endif
    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    // Read "Establishing Wi-Fi or Ethernet Connection" section in
    // examples/protocols/README.md for more information about this function.
    result = example_connect();
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "example_connect fail, returns(0x%x).",
                                (uint32_t)result);
#if CONFIG_EXAMPLE_CONNECT_WIFI
    result = esp_wifi_set_ps(WIFI_PS_NONE);
    MB_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE,
                                   TAG,
                                   "esp_wifi_set_ps fail, returns(0x%x).",
                                   (uint32_t)result);
#endif
    return ESP_OK;
}

static esp_err_t destroy_services(void)
{
    esp_err_t err = ESP_OK;

    err = example_disconnect();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                   TAG,
                                   "example_disconnect fail, returns(0x%x).",
                                   (uint32_t)err);
    err = esp_event_loop_delete_default();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                       TAG,
                                       "esp_event_loop_delete_default fail, returns(0x%x).",
                                       (uint32_t)err);
    err = esp_netif_deinit();
    MB_RETURN_ON_FALSE((err == ESP_OK || err == ESP_ERR_NOT_SUPPORTED), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "esp_netif_deinit fail, returns(0x%x).",
                                        (uint32_t)err);
    err = nvs_flash_deinit();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "nvs_flash_deinit fail, returns(0x%x).",
                                (uint32_t)err);
#if CONFIG_MB_MDNS_IP_RESOLVER
    stop_mdns_service();
#endif
    return err;
}

// Modbus slave initialization
static esp_err_t slave_init(mb_communication_info_t* comm_info)
{
    mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

    void* slave_handler = NULL;

    // Initialization of Modbus controller
    esp_err_t err = mbc_slave_init_tcp(&slave_handler);
    MB_RETURN_ON_FALSE((err == ESP_OK && slave_handler != NULL), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mb controller initialization fail.");

    comm_info->ip_addr = NULL; // Bind to any address
    comm_info->ip_netif_ptr = eth_netif_spi;//(void*)get_example_netif();

    // Setup communication parameters and start stack
    err = mbc_slave_setup((void*)comm_info);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_setup fail, returns(0x%x).",
                                        (uint32_t)err);

    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbc_slave_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.
    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0; // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_params.holding_data0; // Set pointer to storage instance
    reg_area.size = (MB_REG_HOLDING_START_AREA1 - MB_REG_HOLDING_START_AREA0) << 1; // Set the size of register storage instance
    err = mbc_slave_set_descriptor(reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (uint32_t)err);

    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA1; // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_params.holding_data4; // Set pointer to storage instance
    reg_area.size = sizeof(float) << 2; // Set the size of register storage instance
    err = mbc_slave_set_descriptor(reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (uint32_t)err);

    // Initialization of Input Registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;
    reg_area.address = (void*)&input_reg_params.input_data0;
    reg_area.size = sizeof(float) << 2;
    err = mbc_slave_set_descriptor(reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_set_descriptor fail, returns(0x%x).",
                                        (uint32_t)err);
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA1;
    reg_area.address = (void*)&input_reg_params.input_data4;
    reg_area.size = sizeof(float) << 2;
    err = mbc_slave_set_descriptor(reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_set_descriptor fail, returns(0x%x).",
                                        (uint32_t)err);

    // Initialization of Coils register area
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address = (void*)&coil_reg_params;
    reg_area.size = sizeof(coil_reg_params);
    err = mbc_slave_set_descriptor(reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (uint32_t)err);

    // Initialization of Discrete Inputs register area
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_INPUT_START;
    reg_area.address = (void*)&discrete_reg_params;
    reg_area.size = sizeof(discrete_reg_params);
    err = mbc_slave_set_descriptor(reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (uint32_t)err);

    // Set values into known state
    setup_reg_data();

    // Starts of modbus controller and stack
    err = mbc_slave_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_start fail, returns(0x%x).",
                                        (uint32_t)err);
    vTaskDelay(5);
    return err;
}

static esp_err_t slave_destroy(void)
{
    esp_err_t err = mbc_slave_destroy();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mbc_slave_destroy fail, returns(0x%x).",
                                (uint32_t)err);
    return err;
}

#include "sys/time.h"

// An example application of Modbus slave. It is based on freemodbus stack.
// See deviceparams.h file for more information about assigned Modbus parameters.
// These parameters can be accessed from main application and also can be changed
// by external Modbus master host.
void app_main(void)
{
    //ESP_ERROR_CHECK(init_services());

    init_ethernet();

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    mb_communication_info_t comm_info = { 0 };

#if !CONFIG_EXAMPLE_CONNECT_IPV6
    comm_info.ip_addr_type = MB_IPV4;
#else
    comm_info.ip_addr_type = MB_IPV6;
#endif
    comm_info.ip_mode = MB_MODE_TCP;

    comm_info.ip_port = MB_TCP_PORT_NUMBER;
    ESP_ERROR_CHECK(slave_init(&comm_info));

    struct timeval xTime;
    uint16_t xTimeMs = 1; // 65535

    xTime.tv_sec = xTimeMs / 1000;
    xTime.tv_usec = (xTimeMs % 1000) * 1000;

    printf("Time: tv_sec: %ld, tv_usec: %ld\r\n", xTime.tv_sec, xTime.tv_usec);

    // The Modbus slave logic is located in this function (user handling of Modbus)
    slave_operation_func(NULL);

    ESP_ERROR_CHECK(slave_destroy());
    ESP_ERROR_CHECK(destroy_services());
}
