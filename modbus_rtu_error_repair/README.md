# Simplified Modbus Master example to read critical data

Initializes Modbus master interface then reads and writes two device structures in the connected Modbus device.
This project demonstrates how to read and write critical data over modbus and repair from failures.

# Setup example

All required configuration data is in one source file master.c.

Configure the parameters below or leave them as is:
```
#define MASTER_PORT_NUM 2   // UART port number
#define MASTER_SPEED 115200 // Modbus communication speed
#define MB_UART_RXD_PIN 22  // Modbus RS485 interface pins accordingly
#define MB_UART_TXD_PIN 23
#define MB_UART_RTS_PIN 18
```

Simplified Modbus connection schematic for example test:
 ```
    MB_DEVICE_ADDR1
    -------------                -------------
    |           | RS485 network  |           |
    |  Slave  1 |---<>--+---<>---|  Master   |
    |           |                |           |
    -------------                -------------
```

# Setup connections

The MAX485 line driver is used as an example below but other similar chips can be used as well.
RS485 example circuit schematic for connection of master and slave devices into segment:
```
         VCC ---------------+                               +-------------- VCC
                            |                               |
                    +-------x-------+               +-------x-------+
         RXD <------| RO            | DIFFERENTIAL  |             RO|-----> RXD
                    |              B|---------------|B              |
         TXD ------>| DI   MAX485   |    \  /       |    MAX485   DI|<----- TXD
ESP32 WROVER KIT 1  |               |   RS-485 side |               |      External PC (emulator) with USB to serial or
         RTS --+--->| DE            |    /  \       |             DE|---+  ESP32 WROVER KIT 2 (slave)     
               |    |              A|---------------|A              |   |
               +----| /RE           |    PAIR       |            /RE|---+-- RTS
                    +-------x-------+               +-------x-------+
                            |                               |
                           ---                             --- 
                    Modbus Master device             Modbus Slave device
                           
```


```
idf.py build 
idf.py -p PORT -b 921600 flash monitor
```

In order to read or write other registers of device, edit the device_parameters table.

By default it is supposed that your device contains two holding registers that correspond to CID:
```
+-----------------------------------------------------+
| CID              | Start address of modbus register |
|-----------------------------------------------------|
| CID_DEV_RT_DATA  | 0                                |
|-----------------------------------------------------|
| CID_DEV_APP_INFO | 1                                |
+-----------------------------------------------------+
```
Note:
This increases delays of MB processing but allows to decrease possible issues.
`CONFIG_FMB_SERIAL_TASK_PRIO` - should be set to your (highest task priority + 1) executed on the same CPU.

## Example Output:

I (236) cpu_start: Project name:     simple_mb_master
I (241) cpu_start: App version:      9fc77d5
I (246) cpu_start: Compile time:     Jan 24 2023 18:00:48
I (253) cpu_start: ELF file SHA256:  68c5e7e532d8ff70...
I (258) cpu_start: ESP-IDF:          v5.1-dev-3002-gd0cf174019
I (265) cpu_start: Min chip rev:     v0.0
I (270) cpu_start: Max chip rev:     v3.99
I (275) cpu_start: Chip rev:         v1.0
I (279) heap_init: Initializing. RAM available for dynamic allocation:
I (287) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (293) heap_init: At 3FFB2B48 len 0002D4B8 (181 KiB): DRAM
I (299) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (305) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (312) heap_init: At 4008D93C len 000126C4 (73 KiB): IRAM
I (319) spi_flash: detected chip: generic
I (322) spi_flash: flash io: dio
W (326) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (340) app_start: Starting scheduler on CPU0
I (345) app_start: Starting scheduler on CPU1
I (345) main_task: Started on CPU0
I (355) main_task: Calling app_main()
I (355) uart: queue free spaces: 20
I (355) MODBUS_MASTER: Modbus master stack initialized...
I (365) MODBUS_MASTER: Start modbus test...
I (375) main_task: Returned from app_main()
I (405) MODBUS_MASTER: Characteristic #0 Device_rt_data (Data), REG(0, 7), write successful.
W (405) SET: 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
I (435) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), REG(100, 11), write successful.
W (435) SET: 30 00 32 00 4d 00 42 00 31 00 34 00 34 00 53 00
W (435) SET: 30 00 00 00 00 00
I (495) MODBUS_MASTER: Characteristic #0 Device_rt_data (Data), REG(0, 7), parameter read successful.
W (495) GET: 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
I (545) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), REG(100, 11), parameter read successful.
W (545) GET: 30 00 32 00 4d 00 42 00 31 00 34 00 34 00 53 00
W (555) GET: 30 00 00 00 00 00
I (595) MODBUS_MASTER: Characteristic #0 Device_rt_data (Data), REG(0, 7), parameter read successful.
W (595) GET: 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
I (665) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), REG(100, 11), parameter read successful.
W (665) GET: 30 00 32 00 4d 00 42 00 31 00 34 00 34 00 53 00
W (665) GET: 30 00 00 00 00 00
I (725) MODBUS_MASTER: Characteristic #0 Device_rt_data (Data), REG(0, 7), parameter read successful.
W (725) GET: 0a 00 00 00 00 00 00 00 00 00 00 00 00 00

Possible issues:

E (12615) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (12615) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), parameter read fail.
W (12625) DBG: APP info retry: 0
E (13125) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (13125) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), parameter read fail.
W (13135) DBG: APP info retry: 1
E (13645) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (13645) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), parameter read fail.
W (13655) DBG: APP info retry: 2
E (14155) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (14155) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), parameter read fail.
W (14165) DBG: APP info retry: 3
E (14675) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (14675) MODBUS_MASTER: Characteristic #1 Dev_app_info (Data), parameter read fail.
W (14685) DBG: APP info retry: 4
E (15185) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (15185) MODBUS_MASTER: Characteristic #0 Device_rt_data (Data), parameter read fail.
W (15195) DBG: RT data retry: 0
E (15695) MB_CONTROLLER_MASTER: mbc_master_get_parameter(73): Master get parameter failure, error=(0x107) (ESP_ERR_TIMEOUT).
E (15705) MODBUS_MASTER: Characteristic #0 Device_rt_data (Data), parameter read fail.
W (15705) DBG: RT data retry: 1

These errors mean that slave device is not able to communicate and doing retries. Check board connections.





