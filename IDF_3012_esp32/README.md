| Supported Targets | ESP32 |
| ----------------- | ----- |

# Modbus Master Slave Example

This example demonstrates using of FreeModbus stack port implementation for ESP32 as a master device and slave device simultaneously. 
This implementation is able to read/write values of slave devices connected into Modbus master port. All parameters to be accessed are defined in data dictionary of the modbus master example source file.
The values represented as characteristics with its name and characteristic CID which are linked into registers of slave devices connected into Modbus segment. 
The example implements simple control algorithm and checks parameters from slave device and gets alarm (relay in the slave device) when value of holding_data0 parameter exceeded limit.
The instances for the modbus parameters are common for master and slave examples and located in `examples/protocols/modbus/mb_example_common` folder.

Example parameters definition:
--------------------------------------------------------------------------------------------------
| Slave Address       | Characteristic ID    | Characteristic name  | Description                |
|---------------------|----------------------|----------------------|----------------------------|
| MB_DEVICE_ADDR1     | CID_INP_DATA_0,      | Input_1              | Data channel 1             |
| MB_DEVICE_ADDR1     | CID_HOLD_DATA_0,     | Humidity_1           | Humidity 1                 |
--------------------------------------------------------------------------------------------------
Note: The Slave Address is the same for all parameters for example test but it can be changed in the ```Example Data (Object) Dictionary``` table of master example to address parameters from other slaves.
The Kconfig ```Modbus slave address``` - CONFIG_MB_SLAVE_ADDR parameter in slave example can be configured to create Modbus multi slave segment.

Simplified Modbus connection schematic for example test:
 ```
    MB_DEVICE_ADDR1
    ---------------                -----------------
    |             | RS485 network  |               |
    |  Slave port |---<>--+---<>---|  Master port  |
    |             |                |               |
    ---------------                -----------------
```
Modbus multi slave segment connection schematic:
```
    MB_DEVICE_ADDR1
    -----------------
    |               |   
    |  Ext Slave 1  |---<>--+
    |               |       |
    -----------------       |
    MB_DEVICE_ADDR2         |
    -----------------       |        ------------------
    |               |       |        |                |
    |  Ext Slave  2 |---<>--+---<>---|  Master port   |
    |               |       |        |                |
    -----------------       |        ------------------
    MB_DEVICE_ADDR3         |
    -----------------  RS485 network
    |               |       |
    |  Ext Slave 3  |---<>--+
    |               |
    -----------------
    
    --------------------                -----------------
    |                  | RS485 network  |               |
    |  External Master |---<>--+---<>---|  Slave port   |
    |                  |                |               |
    --------------------                -----------------    
    
```

## Hardware required :
Option 1:
PC (Modbus Slave app) + USB Serial adapter connected to USB port + RS485 line drivers + ESP32 WROVER-KIT board. 

Option 2:
Several ESP32 WROVER-KIT board flashed with modbus_slave example software to represent slave device with specific slave address (See CONFIG_MB_SLAVE_ADDR). The slave addresses for each board have to be configured as defined in "connection schematic" above.
One ESP32 WROVER-KIT board flashed with modbus_master example. All the boards require connection of RS485 line drivers (see below).

The MAX485 line driver is used as an example below but other similar chips can be used as well.
RS485 example circuit schematic for connection of master and slave devices into segment:
```
Note:
S_RXD - SLAVE RXD PIN configured in kconfig below (CONFIG_MB_SLAVE_UART_RXD)
M_RXD - MASTER RXD PIN configured in kconfig below (CONFIG_MB_MASTER_UART_RXD)
         VCC ---------------+                               +--------------- VCC
                            |                               |
                    +-------x-------+               +-------x-------+
       M_RXD <------| RO            | DIFFERENTIAL  |             RO|-----> S_RXD
                    |              B|---------------|B              |
       M_TXD ------>| DI   MAX485   |    \  /       |    MAX485   DI|<----- S_TXD
ESP32 WROVER KIT    |               |   RS-485 side |               |      External PC (emulator) with USB to serial or
       M_RTS --+--->| DE            |    /  \       |             DE|---+  ESP32 WROVER KIT (slave port)     
master port         |    |         A|---------------|A              |   |
               +----| /RE           |    PAIR       |            /RE|---+-- S_RTS
                    +-------x-------+               +-------x-------+
                            |                               |
                           ---                             --- 
                    Modbus Master device             Modbus Slave device
                           
```
The Master and slave A, B 


## How to setup and use an example:

### Configure the application
Start the command below to setup configuration:
```
idf.py menuconfig
```
Configure the UART port numbers and pins (master and slave ports accordingly) used for modbus communication using the table below.
Define the communication mode parameter for master and slave in Kconfig - CONFIG_MB_COMM_MODE (must be the same for master and slave devices in one segment).
Configure the slave address for each slave in the Modbus segment (the CONFIG_MB_SLAVE_ADDR in Kconfig).

```
  -------------------------------------------------------------------------------------------------------------------------
  | ESP32 Interface | #define                  | Default ESP32 Pin     | Default ESP32-S2 Pins| External RS485 Driver Pin |
  | ----------------|--------------------------|-----------------------|----------------------|---------------------------|
  | Slave (RxD)     | CONFIG_MB_SLAVE_UART_RXD | GPIO25                | GPIO7                | DI                        |
  | Slave (TxD)     | CONFIG_MB_SLAVE_UART_TXD | GPIO26                | GPIO8                | RO                        |
  | Slave (RTS)     | CONFIG_MB_SLAVE_UART_RTS | GPIO27                | GPIO9                | ~RE/DE                    |
  | Master (RxD)    | CONFIG_MB_MASTER_UART_RXD| GPIO22                | GPIO19               | DI                        |
  | Master (TxD)    | CONFIG_MB_MASTER_UART_TXD| GPIO23                | GPIO20               | RO                        |
  | Master (RTS)    | CONFIG_MB_MASTER_UART_RTS| GPIO18                | GPIO18               | ~RE/DE                    |
  | Ground          | n/a                      | GND                   | GND                  | GND                       |
  -------------------------------------------------------------------------------------------------------------------------
```
Note: The GPIO22 - GPIO25 can not be used with ESP32-S2 chip because they are used for flash chip connection. Please refer to UART documentation for selected target.

Connect USB to RS485 adapter to computer and connect its A, B output lines with the A, B lines of RS485 line driver connected to ESP32 (See picture above).

The communication parameters of Modbus stack allow to configure it appropriately but usually it is enough to use default settings.
See the help string of parameters for more information.

### Setup external Modbus slave devices or emulator
Option 1:
Configure the external Modbus master software according to port configuration parameters used in the example. The Modbus Slave application can be used with this example to emulate slave devices with its parameters. Use official documentation for software to setup emulation of slave devices.

Option 2:
Other option is to have the modbus_slave example application flashed into ESP32 WROVER KIT board and connect boards together as showed on the Modbus connection schematic above. See the Modbus slave API documentation to configure communication parameters and slave addresses as defined in "Example parameters definition" table above.

### Build and flash software of master device
Build the project and flash it to the board, then run monitor tool to view serial output:
```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output
Example output of the application:
```
I (490) uart: queue free spaces: 20
I (500) uart: queue free spaces: 20
I (500) MASTER_TEST: Modbus master stack initialized...
in serial master task
I (500) MASTER_TEST: Start modbus test...
I (500) SLAVE_TEST: Modbus slave stack initialized.
I (510) SLAVE_TEST: Start modbus test...
I (510) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 1.340000 (0x3fab851f) read successful.
In serial slave task
I (530) SLAVE_TEST: HOLDING READ (69159 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (530) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
I (540) SLAVE_TEST: INPUT READ (88451 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (1050) SLAVE_TEST: HOLDING READ (608668 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (1050) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 1.540000 (0x3fdeb853) read successful.
I (1060) SLAVE_TEST: INPUT READ (621329 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (1060) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
I (1580) SLAVE_TEST: HOLDING READ (1138726 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (1580) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 1.740000 (0x3ff851ed) read successful.
I (1590) SLAVE_TEST: INPUT READ (1151475 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (1600) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
I (2110) SLAVE_TEST: HOLDING READ (1668664 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (2110) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 1.940000 (0x4008f5c3) read successful.
I (2120) SLAVE_TEST: INPUT READ (1681414 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (2120) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
I (2640) SLAVE_TEST: HOLDING READ (2198672 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (2640) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 2.140000 (0x4015c290) read successful.
I (2650) SLAVE_TEST: INPUT READ (2211422 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (2650) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
I (3170) SLAVE_TEST: HOLDING READ (2728680 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (3170) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 2.340000 (0x40228f5d) read successful.
I (3180) SLAVE_TEST: INPUT READ (2741429 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (3190) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
I (3700) SLAVE_TEST: HOLDING READ (3258688 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4074, SIZE:2
I (3700) MASTER_TEST: Characteristic #0 Humidity_1 (%rH) value = 2.540000 (0x402f5c2a) read successful.
I (3710) SLAVE_TEST: INPUT READ (3271437 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3f28, SIZE:2
I (3720) MASTER_TEST: Characteristic #1 Input_1 (C) value = 1.120000 (0x3f8f5c29) read successful.
...
I (10175) MASTER_TEST: Destroy master...
...
```
The example reads the characteristics from slave device(s), while alarm is not triggered in the slave device (See the "Example parameters definition"). The output line describes Timestamp, Cid of characteristic, Characteristic name (Units), Characteristic value (Hex).

