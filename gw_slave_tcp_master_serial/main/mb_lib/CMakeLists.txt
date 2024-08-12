add_library(mb_lib "${CMAKE_CURRENT_LIST_DIR}/mb_adapter.c")

idf_component_get_property(dir espressif__esp-modbus COMPONENT_DIR)
target_include_directories(mb_lib PUBLIC 
                            "${CMAKE_CURRENT_LIST_DIR}"
                            "${CMAKE_CURRENT_LIST_DIR}/include"
                            "${dir}/freemodbus/common"
                            "${dir}/freemodbus/common/include"
                            "${dir}/freemodbus/modbus/include"
                            "${dir}/freemodbus/modbus/rtu"
                            "${dir}/freemodbus/modbus/ascii"
                            "${dir}/freemodbus/modbus/tcp"
                            "${dir}/freemodbus/port/"
                            "${dir}/freemodbus/modbus/functions"
                            "${dir}/freemodbus/serial_slave/modbus_controller"
                            "${dir}/freemodbus/serial_master/modbus_controller"
                            "${dir}/freemodbus/tcp_slave/port"
                            "${dir}/freemodbus/tcp_slave/modbus_controller"
                            "${dir}/freemodbus/tcp_master/port"
                            "${dir}/freemodbus/tcp_master/modbus_controller"
                            )

idf_component_get_property(driver_lib driver COMPONENT_LIB)
target_link_libraries(mb_lib PUBLIC ${driver_lib})
idf_component_get_property(timer_lib esp_timer COMPONENT_LIB)
target_link_libraries(mb_lib PUBLIC ${timer_lib})
idf_component_get_property(netif_lib esp_netif COMPONENT_LIB)
target_link_libraries(mb_lib PUBLIC ${netif_lib})

# Wrap port functions to substitute port with port_adapter object
set(WRAP_FUNCTIONS
#    eMBPoll
    eMBRegInputCBSerialMaster
    eMBRegHoldingCBSerialMaster
    eMBRegCoilsCBSerialMaster
    eMBRegDiscreteCBSerialMaster
#    mbc_reg_input_slave_cb
#    mbc_reg_holding_slave_cb
#    mbc_reg_coils_slave_cb
#    mbc_reg_discrete_slave_cb
    eMBRegDiscreteCB
    eMBRegCoilsCB
    eMBRegHoldingCB
    eMBRegInputCB
)

message(STATUS "DEBUG: Use mb_lib library folder: ${CMAKE_CURRENT_LIST_DIR}.")

foreach(wrap ${WRAP_FUNCTIONS})
    message(STATUS "DEBUG: wrap function ${wrap}")
    target_link_libraries(mb_lib PUBLIC "-Wl,--undefined=${wrap}")
    target_link_libraries(mb_lib PUBLIC "-Wl,--wrap=${wrap}")
endforeach()

# allow multiple symbol definitions
target_link_libraries(mb_lib PUBLIC "-Wl,--allow-multiple-definition") #--allow-multiple-definition -z,multidefs