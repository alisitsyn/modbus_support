set(PROJECT_NAME "modbus_master_slave")

idf_component_register(SRCS "main.c" "slave.c" "master.c"
                        INCLUDE_DIRS ".")
