This is an example Modbus slave project to demonstrate opportunity to redefine existing callback functions to do custom action on read/write access to any Modbus register.

See the file components/freemodbus/readme.md for more information on how to setup the example
The output of the example:
```
I (330) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (341) uart: queue free spaces: 20
I (351) SLAVE_TEST: Modbus slave stack initialized.
I (351) SLAVE_TEST: Start modbus test...
I (431) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (431) SLAVE_TEST: INPUT READ (196487 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (1471) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (1471) SLAVE_TEST: INPUT READ (1237035 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (2511) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (2511) SLAVE_TEST: INPUT READ (2274214 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (3551) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (3551) SLAVE_TEST: INPUT READ (3312719 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (4591) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (4591) SLAVE_TEST: INPUT READ (4353646 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (5631) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (5631) SLAVE_TEST: INPUT READ (5391527 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (6661) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (6671) SLAVE_TEST: INPUT READ (6429053 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (7701) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (7701) SLAVE_TEST: INPUT READ (7467004 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
I (8741) USER INPUT CALLBACK: RegStart: 1, Size: 5
I (8751) SLAVE_TEST: INPUT READ (8508069 us), ADDR:0, TYPE:8, INST_ADDR:0x3ffb3088, SIZE:5
```