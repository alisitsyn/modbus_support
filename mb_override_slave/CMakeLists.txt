# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Include parameters from common modbus folder
set(MB_PARAMS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mb_example_common")
message(FAIL "Use example folder ${MB_PARAMS_DIR} for MB parameters.")
if(EXISTS ${MB_PARAMS_DIR}/include)
    list(APPEND EXTRA_COMPONENT_DIRS ${MB_PARAMS_DIR})
    message(STATUS "Use project folder ${MB_PARAMS_DIR} for MB parameters.")
else()
    list(APPEND EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/protocols/modbus/mb_example_common)
    message(STATUS "Use example folder ${MB_PARAMS_DIR} for MB parameters.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(modbus_slave)
