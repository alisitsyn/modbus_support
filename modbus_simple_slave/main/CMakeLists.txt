set(PROJECT_NAME "modbus_slave")

idf_component_register(SRCS "slave.cpp" "ModbusSlave.cpp"
                    INCLUDE_DIRS ".")