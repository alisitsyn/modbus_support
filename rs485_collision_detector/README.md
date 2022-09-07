# Test RS485 collision detection

This directory contains a set of ESP-IDF projects to be used as tests only.
This test example provides the way to allow collision detection in RS485 based networks.
It sends the long packet of random data and controls collision of sent data.

Collision detection using the internal HW of ESP32 in following setup (only this one works).

```
    hal->dev->rs485_conf.tx_rx_en = 1;
    // Don't send while receiving => collision avoidance
    hal->dev->rs485_conf.rx_busy_tx_en = 0;
```

If the collision is detected (for example the other master on the bus transmits data), the exaple prints the message with the hex representation of buffer with collision (original data to send):
E (12114) UART_COLLISON: 0x3ffb6e70   00 01 02 03 04 05 06 07  08 09 0a 0b 0c 0d 0e 0f  |................|
E (12114) UART_COLLISON: 0x3ffb6e80   10 11 12 13 14 15 16 17  18 19 1a 1b 1c 1d 1e 1f  |................|
E (12124) UART_COLLISON: 0x3ffb6e90   20 21 22 23 24 25 26 27  28 29 2a 2b 2c 2d 2e 2f  | !"#$%&'()*+,-./|
E (12134) UART_COLLISON: 0x3ffb6ea0   30 31 32 33 34 35 36 37  38 39 3a 3b 3c 3d 3e 3f  |0123456789:;<=>?|
E (12144) UART_COLLISON: 0x3ffb6eb0   40 41 42 43 44 45 46 47  48 49 4a 4b 4c 4d 4e 4f  |@ABCDEFGHIJKLMNO|
E (12164) UART_COLLISON: 0x3ffb6ec0   50 51 52 53 54 55 56 57  58 59 5a 5b 5c 5d 5e 5f  |PQRSTUVWXYZ[\]^_|
E (12174) UART_COLLISON: 0x3ffb6ed0   60 61 62 63 64 65 66 67  68 69 6a 6b 6c 6d 6e 6f  |`abcdefghijklmno|
E (12184) UART_COLLISON: 0x3ffb6ee0   70 71 72 73 74 75 76 77  78 79 7a 7b 7c 7d 7e 7f  |pqrstuvwxyz{|}~.|

See the log: collision_detection_log.txt

This requires using of [Variant A of RS485 connection](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html#circuit-a-collision-detection-circuit).

RS485 example circuit schematic:
```
         VDD_USB------------+
                            |
                    +-------x-------+
         RXD <------| RO            | DIFFERENTIAL
                    |              B|--------+
         TXD ------>| DI   MAX485   |        |
PC USB-RS485        |               |        |
         RTS --+--->| DE            |        |
               |    |              A|----+   |
               +----| /RE           |    |   |
                    +-------x--------+   |   |
                            |            |   |
                           ---           |   |
                                         |   |
         VCC ---------------+            |   |              +--------------- VCC
                            |            |   |              |
                    +-------x-------+    |   |      +-------x-------+
         RXD <------| RO            | DIFFERENTIAL  |             RO|-----> RXD
                    |              B|----+---+------|B              |
         TXD ------>| DI   MAX485   |    \  /       |    MAX485   DI|<----- TXD
Example board       |               |   RS-485 side |               |    Modbus master ESP32
         RTS --+--->| DE            |    /  \       |             DE|---+
               |    |              A|---------------|A              |   |
               +----| /RE           |    PAIR       |            /RE|---+-- RTS
                    +-------x-------+              +-------x-------+
                            |                               |
                           ---                             ---
```
