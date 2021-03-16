#if(${IDF_VERSION} MATCHES "v4.3")

set(MB_IDF_DIR "${IDF_PATH}/components/freemodbus")

set(MB_PROJ_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(srcs
    "${MB_IDF_DIR}/common/esp_modbus_master.c"
    "${MB_IDF_DIR}/common/esp_modbus_slave.c"
    "${MB_IDF_DIR}/modbus/mb.c"
    "${MB_IDF_DIR}/modbus/mb_m.c"
    "${MB_IDF_DIR}/modbus/ascii/mbascii.c"
    "${MB_IDF_DIR}/modbus/ascii/mbascii_m.c"
    "${MB_IDF_DIR}/modbus/rtu/mbrtu_m.c"
    "${MB_IDF_DIR}/modbus/rtu/mbrtu.c"
    "${MB_IDF_DIR}/modbus/rtu/mbcrc.c"
    "${MB_IDF_DIR}/modbus/tcp/mbtcp.c"
    "${MB_IDF_DIR}/port/port.c"
    "${MB_IDF_DIR}/port/portevent.c"
    "${MB_IDF_DIR}/port/portevent_m.c"
    "${MB_IDF_DIR}/port/portother.c"
    "${MB_IDF_DIR}/port/portother_m.c"
    "${MB_IDF_DIR}/port/portserial.c"
    "${MB_IDF_DIR}/port/portserial_m.c"
    "${MB_IDF_DIR}/port/porttimer.c"
    "${MB_IDF_DIR}/port/porttimer_m.c"
    "${MB_IDF_DIR}/modbus/functions/mbfunccoils.c"
    "${MB_IDF_DIR}/modbus/functions/mbfunccoils_m.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncdiag.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncdisc.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncdisc_m.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncholding.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncholding_m.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncinput.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncinput_m.c"
    "${MB_IDF_DIR}/modbus/functions/mbfuncother.c"
    "${MB_IDF_DIR}/modbus/functions/mbutils.c"
    "${MB_IDF_DIR}/modbus/functions/mbutils.c"
    "${MB_IDF_DIR}/tcp_slave/port/port_tcp_slave.c"
    "${MB_IDF_DIR}/tcp_master/port/port_tcp_master.c"
    "${MB_IDF_DIR}/common/esp_modbus_master_tcp.c"
    "${MB_IDF_DIR}/common/esp_modbus_slave_tcp.c"
    "${MB_IDF_DIR}/common/esp_modbus_master_serial.c"
    "${MB_IDF_DIR}/common/esp_modbus_slave_serial.c")

set(include_dirs "${MB_IDF_DIR}/common/include")

set(priv_include_dirs "${MB_IDF_DIR}/common" 
                        "${MB_IDF_DIR}/port"
                        "${MB_IDF_DIR}/modbus" 
                        "${MB_IDF_DIR}/modbus/ascii"
                        "${MB_IDF_DIR}/modbus/functions"
                        "${MB_IDF_DIR}/modbus/rtu"
                        "${MB_IDF_DIR}/modbus/tcp"
                        "${MB_IDF_DIR}/modbus/include")

list(APPEND priv_include_dirs "${MB_IDF_DIR}/serial_slave/port"
                                "${MB_IDF_DIR}/serial_master/port" 
                                "${MB_IDF_DIR}/tcp_slave/port"
                                "${MB_IDF_DIR}/tcp_master/port")
    
if(EXISTS "${MB_PROJ_DIR}/serial_slave/modbus_controller/mbc_serial_slave.c")
    list(APPEND srcs "${MB_PROJ_DIR}/serial_slave/modbus_controller/mbc_serial_slave.c")
    list(APPEND priv_include_dirs "${MB_PROJ_DIR}/serial_slave/modbus_controller")
    message(STATUS "Compile serial slave controller srcs from ${MB_PROJ_DIR} folder.")
else()
    list(APPEND srcs "${MB_IDF_DIR}/serial_slave/modbus_controller/mbc_serial_slave.c")
    list(APPEND priv_include_dirs "${MB_IDF_DIR}/serial_slave/modbus_controller")
endif()

if(EXISTS "${MB_PROJ_DIR}/serial_master/modbus_controller/mbc_serial_master.c")
    list(APPEND srcs "${MB_PROJ_DIR}/serial_master/modbus_controller/mbc_serial_master.c")
    list(APPEND priv_include_dirs "${MB_PROJ_DIR}/serial_master/modbus_controller")
    message(STATUS "Compile serial master controller srcs from ${MB_PROJ_DIR} folder.")
else()
    list(APPEND srcs "${MB_IDF_DIR}/serial_master/modbus_controller/mbc_serial_master.c")
    list(APPEND priv_include_dirs "${MB_IDF_DIR}/serial_master/modbus_controller")
endif()

if(EXISTS "${MB_PROJ_DIR}/tcp_slave/modbus_controller/mbc_tcp_slave.c")
    list(APPEND srcs "${MB_PROJ_DIR}/tcp_slave/modbus_controller/mbc_tcp_slave.c")
    list(APPEND priv_include_dirs "${MB_PROJ_DIR}/tcp_slave/modbus_controller")
    message(STATUS "Compile tcp slave controller srcs from ${MB_PROJ_DIR} folder.")
else()
    list(APPEND srcs "${MB_IDF_DIR}/tcp_slave/modbus_controller/mbc_tcp_slave.c")
    list(APPEND priv_include_dirs "${MB_IDF_DIR}/tcp_slave/modbus_controller")
endif()

if(EXISTS "${MB_PROJ_DIR}/tcp_master/modbus_controller/mbc_tcp_master.c")
    list(APPEND srcs "${MB_PROJ_DIR}/tcp_master/modbus_controller/mbc_tcp_master.c")
    list(APPEND priv_include_dirs "${MB_PROJ_DIR}/tcp_master/modbus_controller")
    message(STATUS "Compile tcp master controller srcs from ${MB_PROJ_DIR} folder.")
else()
    list(APPEND srcs "${MB_IDF_DIR}/tcp_master/modbus_controller/mbc_tcp_master.c")
    list(APPEND priv_include_dirs "${MB_IDF_DIR}/tcp_master/modbus_controller")
endif()

#add_prefix(priv_include_dirs "${MB_IDF_DIR}/" ${priv_include_dirs})
#add_prefix(include_dirs "${MB_IDF_DIR}/" ${include_dirs})

message(STATUS "Sources: ${srcs}.")
message(STATUS "Includes: ${priv_include_dirs}.")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    PRIV_INCLUDE_DIRS "${priv_include_dirs}"
                    REQUIRES driver)

#endif()