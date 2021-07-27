# Simplified Modbus Master example
Initializes Modbus master interface and reads and writes two holding registers in the connected Modbus device.

# Setup example

All required configuration data is in one source file master.c.

Configure the parameters below or leave them as is:
```
#define MASTER_PORT_NUM 2 // UART port number 
#define MASTER_SPEED 115200 // Modbus communication speed
#define MB_UART_RXD_PIN 22 // Modbus RS485 interface pins accordingly
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
---------------------------------------------------
| CID          | Start address of modbus register | 
---------------------------------------------------
| CID_DEV_REG0 | 0                                |
---------------------------------------------------
| CID_DEV_REG1 | 1                                |
---------------------------------------------------
```
## Fixes

This project contains the latest fixes implemented in ESP_IDF v4.2 and ported for v4.0.
These fixes are included in components/freemodbus, components/driver folders and override existing functionality in ESP_IDF.

Note: Enable `CONFIG_FMB_TIMER_ISR_IN_IRAM` option to decrease handling delays if you use NVS flash functions in your application.
`CONFIG_FMB_TIMER_PORT_ENABLED` should be disabled to avoid failures when you have other tasks started on the same CPU. 
This increases delays of MB processing but allows to decrease possible issues.
`CONFIG_FMB_SERIAL_TASK_PRIO` - should be set to your (highest task priority + 1) executed on the same CPU.
As an alternative replace the sources in ESP_IDF folders with the `*.c/h files` from components folder of this project.

## Example Output:

I (333) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (344) uart: queue free spaces: 20
I (404) MODBUS_MASTER: Modbus master stack initialized...
I (504) MODBUS_MASTER: Start modbus test...
I (544) MODBUS_MASTER: Characteristic #0 MB_hold_reg-0 (Data) value = (0x1234) parameter read successful.
I (594) MODBUS_MASTER: Characteristic #0 MB_hold_reg-0 (Data) value = (0x1234), write successful.
I (634) MODBUS_MASTER: Characteristic #1 MB_hold_reg-1 (Data) value = (0x1234) parameter read successful.
I (674) MODBUS_MASTER: Characteristic #1 MB_hold_reg-1 (Data) value = (0x5678), write successful.
...
I (2224) MODBUS_MASTER: Characteristic #0 MB_hold_reg-0 (Data) value = (0x1234), write successful.
I (2254) MODBUS_MASTER: Characteristic #1 MB_hold_reg-1 (Data) value = (0x5678) parameter read successful.
I (2284) MODBUS_MASTER: Characteristic #1 MB_hold_reg-1 (Data) value = (0x5678), write successful.
I (2314) MODBUS_MASTER: Characteristic #0 MB_hold_reg-0 (Data) value = (0x1234) parameter read successful.
I (9884) MODBUS_MASTER: Modbus test is completed.

Possible issues:

E (9714) MODBUS_MASTER: Characteristic #0 MB_hold_reg-0 (Data), parameter read fail.
E (9874) MB_CONTROLLER_MASTER: mbc_master_get_parameter(111): SERIAL master get parameter failure error=(0x107) (ESP_ERR_TIMEOUT).
These errors mean that slave device is not able to communicate. Check board connections.





