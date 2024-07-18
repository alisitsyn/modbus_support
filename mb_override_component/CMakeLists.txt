# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

#set(EXTRA_COMPONENT_DIRS "/home/esp-man/esp32/esp-modbus")
#    override_path:

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(modbus_master)
