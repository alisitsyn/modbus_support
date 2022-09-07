set(PROJECT_NAME "rs485_collision_detector")

idf_component_register(SRCS "rs485_collision_detector.c"
                        INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

