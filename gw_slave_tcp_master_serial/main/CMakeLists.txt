set(PROJECT_NAME "gw_slave_tcp_master_serial")

idf_component_register(SRCS "gw_slave_tcp_master_serial.c"
                        INCLUDE_DIRS ".")

add_subdirectory(mb_lib)
target_link_libraries(${COMPONENT_LIB} PUBLIC mb_lib) 

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
