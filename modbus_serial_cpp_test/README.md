# Simplified Modbus Master example
Initializes Modbus master interface and reads and writes holding registers and coild in the connected Modbus device.
This example uses preliminary CPP ModbusMaster class `main/ModbusMaster.cpp/h` to read/write Modbus registers.
The class is updated from Arduino project and allow to use ESP_Modbus library in CPP projects.

Note: This project is not final and is prepared just for demonstration for CPP users.

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
    MB_SLAVE_SHORT_ADDRESS
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

By default it is supposed that your device contains five holding registers at Modbus address = 0.
Every retry cycle Master reads the holding registers of slave defined as `MB_SLAVE_SHORT_ADDRESS`. Also reads the coil registers and invert some bits defined in masked value.

Note: Enable `CONFIG_FMB_TIMER_ISR_IN_IRAM` option to decrease handling delays if you use NVS flash functions in your application.
`CONFIG_FMB_TIMER_PORT_ENABLED` should be disabled to avoid failures when you have other tasks started on the same CPU. 
This increases delays of MB processing but allows to decrease possible issues.
`CONFIG_FMB_SERIAL_TASK_PRIO` - should be set to your (highest task priority + 1) executed on the same CPU.
As an alternative replace the sources in ESP_IDF folders with the `*.c/h files` from components folder of this project.

## Example Output:
```
I (360) MB_MASTER: Modbus master stack initialized...
I (550) MB: COILS: 0x5555
I (550) MB: 11 11 22 22 33 33 44 44 55 55
I (790) MB: COILS: 0xFFFF
I (790) MB: 11 11 22 22 33 33 44 44 55 55
I (1010) MB: COILS: 0x5555
I (1010) MB: 11 11 22 22 33 33 44 44 55 55
I (1190) MB: COILS: 0xFFFF
I (1190) MB: 11 11 22 22 33 33 44 44 55 55
I (1380) MB: COILS: 0x5555
I (1380) MB: 11 11 22 22 33 33 44 44 55 55
I (1590) MB: COILS: 0xFFFF
I (1590) MB: 11 11 22 22 33 33 44 44 55 55
I (1840) MB: COILS: 0x5555
I (1840) MB: 11 11 22 22 33 33 44 44 55 55
I (2040) MB: COILS: 0xFFFF
I (2040) MB: 11 11 22 22 33 33 44 44 55 55
I (2230) MB: COILS: 0x5555
I (2230) MB: 11 11 22 22 33 33 44 44 55 55
I (2450) MB: COILS: 0xFFFF
I (2450) MB: 11 11 22 22 33 33 44 44 55 55
I (2450) MODBUS: Modbus Test completed.
```

Possible issues:

E (9714) MODBUS_MASTER: Characteristic #0 MB_hold_reg-0 (Data), parameter read fail.
E (9874) MB_CONTROLLER_MASTER: mbc_master_get_parameter(111): SERIAL master get parameter failure error=(0x107) (ESP_ERR_TIMEOUT).
These errors mean that slave device is not able to communicate. Check board connections.





