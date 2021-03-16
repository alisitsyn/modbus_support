The folder `components/freemodbus contains` the files to override the modbus controller files represented in the freemodbus component folder of ESP-IDF.
The special CMakeLists.txt is used to override the private files in the component folder.
This technique can be used to allow definition of custom read/write callbacks for Modbus stack.

The callbacks can be defined in the appropriate modbus controller file (for example):
components/freemodbus/serial_slave/modbus_controller/mbc_serial_slave.c

New callbacks must be registered in the constructor of the concrete modbus controller source file:
`esp_err_t mbc_serial_slave_create(void** handler)` as below:
```
...
    // Initialize stack callback function pointers
    mbs_interface_ptr->slave_reg_cb_discrete = NULL;
    mbs_interface_ptr->slave_reg_cb_input = eMBRegInputCBSerialSlave;
    mbs_interface_ptr->slave_reg_cb_holding = NULL;
    mbs_interface_ptr->slave_reg_cb_coils = NULL;
...
```

NULL pointer means that the common callback shall be called for this register.

Example of definition for prototype of callback function to access Input registers:

pucRegBuffer - input data buffer
usAddress - register address of Input register
usNRegs - input number of registers

returns eMBErrorCode as a Modbus port layer error code
```
// Callback function for reading of MB Input Input Registers
eMBErrorCode eMBRegInputCBSerialSlave(UCHAR * pucRegBuffer, USHORT usAddress,
                                USHORT usNRegs)
{
    // <<<< The place to run user defined functionality of callback (Access external peripheral?)
    // Then call common read/write callback  
    eMBErrorCode xErr =  mbc_reg_input_slave_cb(pucRegBuffer, usAddress, usNRegs);
    return xErr;
}
```
Once the callback function is defined the stack will execute it each time host access the register.
Note: Special requirements must be applied to the user callback function.
The callback is executed in the context of Modbus stack task and must return as early as possible to allow stack to function correctly.
Possible algorithm is to add read/write request to the queue which will be processed in separate task, wait completion of request during the time < slave respond time then return status of request.

The above is just an example of using this approach but it can be extended to override any other functionality of Modbus stack in user project.

Note: This behavior is outside of official support for Modbus stack functionality because it is hard to ensure that the user callbacks are called correctly and do not prevent stack to function properly.
