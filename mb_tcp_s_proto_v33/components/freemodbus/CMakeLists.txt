# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(COMPONENT_SRCS 
                    "common/esp_modbus_common.c"
                    "common/esp_modbus_master.c"
                    "common/esp_modbus_slave.c"
                    "modbus/mb.c"
                    "modbus/mb_m.c"
                    "modbus/ascii/mbascii.c"
                    "modbus/rtu/mbrtu_m.c"
                    "modbus/rtu/mbrtu.c"
                    "modbus/rtu/mbcrc.c"
                    "modbus/tcp/mbtcp.c"
                    "modbus/port/port.c"
                    "modbus/functions/mbfunccoils.c"
                    "modbus/functions/mbfunccoils_m.c"
                    "modbus/functions/mbfuncdiag.c"
                    "modbus/functions/mbfuncdisc.c"
                    "modbus/functions/mbfuncdisc_m.c"
                    "modbus/functions/mbfuncholding.c"
                    "modbus/functions/mbfuncholding_m.c"
                    "modbus/functions/mbfuncinput.c"
                    "modbus/functions/mbfuncinput_m.c"
                    "modbus/functions/mbfuncother.c"
                    "modbus/functions/mbutils.c"
                    "serial_slave/port/portevent.c"
                    "serial_slave/port/portother.c"
                    "serial_slave/port/portserial.c"
                    "serial_slave/port/porttimer.c"
                    "serial_slave/modbus_controller/mbc_serial_slave.c"
                    "tcp_slave/port/portevent.c"
                    "tcp_slave/port/portother.c"
                    "tcp_slave/port/porttcp.c"
                    "tcp_slave/modbus_controller/mbc_tcp_slave.c"
                    "serial_master/port/portevent.c"
                    "serial_master/port/portother.c"
                    "serial_master/port/portserial.c"
                    "serial_master/port/porttimer.c"
                    "serial_master/modbus_controller/mbc_serial_master.c")
set(COMPONENT_ADD_INCLUDEDIRS common/include)
set(COMPONENT_PRIV_INCLUDEDIRS  common modbus/port modbus modbus/ascii modbus/functions
                                modbus/rtu modbus/tcp modbus/include)
list(APPEND COMPONENT_PRIV_INCLUDEDIRS serial_slave/port serial_slave/modbus_controller
                                        serial_master/port serial_master/modbus_controller
                                        tcp_slave/port tcp_slave/modbus_controller)
set(COMPONENT_REQUIRES "driver")

register_component()
