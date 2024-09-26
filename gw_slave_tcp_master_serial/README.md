| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# Modbus TCP Slave to Modbus Serial gateway example

This example demonstrates using of FreeModbus stack port implementation for ESP32 targets as a TCP gateway device.
This implementation is able to read/write requests from external TCP Master and translate them to the slave devices connected into Modbus segment. The modbus data dictionary is not used but the mapping areas are defined in the Modbus TCP to reflect the values in its memory (this can be disabled).

The Gateway gets requests from Slave TCP and translates the request to installed Modbus Master instance previously configured.
The translation and mapping teqnique uses the approach with wrapped callback read/write functions to make the adapter to translate the data between instances. This example is prepared from scratch to just demonstrate possible approach for the gateway. Other approaches can be used to override the component sources and realize the gateway object for data translation. The adapter functionality is located in mb_lib library which exposes some internals of installed modbus library.

STATUS: WIP, only holding registers are supported (functionality will be extended later).

The instances for the modbus parameters are common for master and slave examples and located in `examples/protocols/modbus/mb_example_common` folder.

The Kconfig ```Modbus slave address``` - CONFIG_MB_SLAVE_ADDR parameter in slave example can be configured to create Modbus multi slave segment.

Simplified Modbus connection schematic for example test:
 ```
    MB_DEVICE_ADDR1
    -------------                -------------
    |           |     Network    |           |
    |  Slave  1 |---<>--+---<>---|  Master   |
    |           |                |           |
    -------------                -------------
```
Modbus multi slave segment connection schematic:
```
    MB_DEVICE_ADDR1
    -------------
    |           |
    |  Slave 1  |---<>--+
    |           |       |
    -------------       |
    MB_DEVICE_ADDR2     |
    -------------       |        -------------
    |           |       |        |           |
    |  Slave  2 |---<>--+---<>---|  Master   |
    |           |       |        |           |
    -------------       |        -------------
    MB_DEVICE_ADDR3     |
    -------------     Network (Ethernet or WiFi connection)
    |           |       |
    |  Slave 3  |---<>--+
    |           |
    -------------
```

## Hardware required :
Option 1:
PC (Modbus TCP Slave application) + ESP32 based development board with modbus_tcp_slave example.

Option 2:
Several ESP32 based boards flashed with modbus_tcp_slave example software to represent slave devices. The IP slave addresses for each board have to be configured in `Modbus Example Configuration` menu according to the communication table of example.
One ESP32 based development board should be flashed with modbus_master example and connected to the same network. All the boards require configuration of network settings as described in `examples/common_components/protocol_examples_common`.

## How to setup and use an example:

### Configure the application
Start the command below to setup configuration:
```
idf.py menuconfig
```

The communication parameters of Modbus stack allow to configure it appropriately but usually it is enough to use default settings.
See the help string of parameters for more information.
There are three ways to configure how the master example will obtain slave IP addresses in the network:
* Enable CONFIG_MB_MDNS_IP_RESOLVER option allows to query for modbus services provided by Modbus slaves in the network and automatically configure IP table. This requires to activate the same option for each slave with unique modbus slave address configured in `Modbus Example Configuration` menu.

### Setup external Modbus slave devices or emulator
Option 1:
Configure the external Modbus master software according to port configuration parameters used in the example. The Modbus Slave application can be used with this example to emulate slave devices with its parameters. Use official documentation for software to setup emulation of slave devices.

Option 2:
Other option is to have the modbus_slave example application flashed into ESP32 based board and connect boards together as showed on the Modbus connection schematic above. See the Modbus slave API documentation to configure communication parameters and slave addresses as defined in "Example parameters definition" table above.

### Build and flash software of master device
Build the project and flash it to the board, then run monitor tool to view serial output:
```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Connect to the device IP from external TCP Master

Connect the external Modbus TCP Master software to the slave IP address showed in the log. The mDNS service can be used to connect to the device.

## Example Output
Example output of the application:
```
I (4463) esp_netif_handlers: example_netif_sta ip: 192.168.88.247, mask: 255.255.255.0, gw: 192.168.88.1
I (4463) example_connect: Got IPv4 event: Interface "example_netif_sta" address: 192.168.88.247
I (4473) example_common: Connected to example_netif_sta
I (4473) example_common: - IPv4 address: 192.168.88.247,
I (4483) wifi:Set ps type: 0, coexist: 0

I (4483) MB_TCP_SLAVE_PORT: Socket (#54), listener  on port: 1502, errno=0
I (4493) MB_TCP_SLAVE_PORT: Protocol stack initialized.
I (4553) uart: queue free spaces: 20
I (4553) SLAVE_TEST: Modbus master stack initialized...
I (4553) SLAVE_TEST: Modbus slave stack initialized.
I (4553) SLAVE_TEST: Start modbus test...
I (11883) wifi:<ba-add>idx:0 (ifx:0, 64:d1:54:1a:23:5b), tid:0, ssn:5, winSize:64
I (12923) MB_TCP_SLAVE_PORT: Socket (#55), accept client connection from address: 192.168.88.249
W (13233) port_stub: callback __wrap_eMBRegHoldingCB, 0x3ffcd975, 1, 5
W (13233) GW_TCP_BUF: 54 f6 00 00 00 06 01 03 0a 00 00 05
I (13283) GW: Received response from serial slave.
I (13283) SLAVE_TEST: HOLDING READ (12756870 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4ec4, SIZE:5
W (14503) port_stub: callback __wrap_eMBRegHoldingCB, 0x3ffcd975, 1, 5
W (14503) GW_TCP_BUF: 54 f7 00 00 00 06 01 03 0a 00 00 05
I (14533) GW: Received response from serial slave.
I (14543) SLAVE_TEST: HOLDING READ (14010727 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4ec4, SIZE:5
W (15753) port_stub: callback __wrap_eMBRegHoldingCB, 0x3ffcd975, 1, 5
W (15753) GW_TCP_BUF: 54 f8 00 00 00 06 01 03 0a 00 00 05
I (15813) GW: Received response from serial slave.
I (15813) SLAVE_TEST: HOLDING READ (15284289 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4ec4, SIZE:5
W (17023) port_stub: callback __wrap_eMBRegHoldingCB, 0x3ffcd975, 1, 5
W (17023) GW_TCP_BUF: 54 f9 00 00 00 06 01 03 0a 00 00 05
I (17073) GW: Received response from serial slave.
I (17073) SLAVE_TEST: HOLDING READ (16544129 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4ec4, SIZE:5
W (18293) port_stub: callback __wrap_eMBRegHoldingCB, 0x3ffcd975, 1, 5
W (18293) GW_TCP_BUF: 54 fa 00 00 00 06 01 03 0a 00 00 05
I (18343) GW: Received response from serial slave.
I (18353) SLAVE_TEST: HOLDING READ (17818814 us), ADDR:0, TYPE:2, INST_ADDR:0x3ffb4ec4, SIZE:5
W (18663) port_stub: callback __wrap_eMBRegHoldingCB, 0x3ffcd979, 1, 2
W (18663) GW_TCP_BUF: 54 fb 00 00 00 0b 01 10 00 00 00 02 04 00 00 42
W (18673) GW_TCP_BUF: 5c
I (18713) GW: Received response from serial slave.
I (18723) SLAVE_TEST: HOLDING WRITE (18192158 us), ADDR:0, TYPE:1, INST_ADDR:0x3ffb4ec4, SIZE:2
I (18723) SLAVE_TEST: Modbus controller destroyed.
```
The example reads the characteristics from serial slave device(s) over TCP. The output line describes the characteristics read from serial slave device and reflected in this gateway.

Modbus Serial Slave Log:
Rx:003002-01 03 00 01 00 05 D4 09
Tx:003003-01 03 0A 5C 42 55 AA 55 AA 55 AA 55 AA 9A BC
Rx:003004-01 03 00 01 00 05 D4 09
Tx:003005-01 03 0A 5C 42 55 AA 55 AA 55 AA 55 AA 9A BC
Rx:003006-01 03 00 01 00 05 D4 09
Tx:003007-01 03 0A 5C 42 55 AA 55 AA 55 AA 55 AA 9A BC
Rx:003008-01 03 00 01 00 05 D4 09
Tx:003009-01 03 0A 5C 42 55 AA 55 AA 55 AA 55 AA 9A BC
Rx:003010-01 03 00 01 00 05 D4 09
Tx:003011-01 03 0A 5C 42 55 AA 55 AA 55 AA 55 AA 9A BC

Modbus TCP Master log:
Tx:007118-56 CF 00 00 00 06 01 03 00 00 00 05
Rx:007119-56 CF 00 00 00 0D 01 03 0A 14 7A 40 FE D7 0A 40 23 EB 85
Tx:007120-56 D0 00 00 00 06 01 03 00 00 00 05
Rx:007121-56 D0 00 00 00 0D 01 03 0A A3 D7 41 10 D7 0A 40 23 EB 85
Tx:007122-56 D1 00 00 00 0B 01 10 00 00 00 02 04 00 00 42 30
Rx:007123-56 D1 00 00 00 06 01 10 00 00 00 02
Tx:007124-56 D2 00 00 00 06 01 03 00 00 00 05
Rx:007125-56 D2 00 00 00 0D 01 03 0A CC CD 3F 8C D7 0A 40 23 EB 85
Tx:007126-56 D3 00 00 00 06 01 03 00 00 00 05
Rx:007127-56 D3 00 00 00 0D 01 03 0A CC CD 40 0C D7 0A 40 23 EB 85
Tx:007128-56 D4 00 00 00 06 01 03 00 00 00 05
Rx:007129-56 D4 00 00 00 0D 01 03 0A 33 34 40 53 D7 0A 40 23 EB 85





