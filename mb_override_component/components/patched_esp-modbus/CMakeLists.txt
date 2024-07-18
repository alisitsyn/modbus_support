# The following cmake file allows to patch the esp-modbus component files of original component.
# If the project needs to override any of the standard sources, the source 
# file needs to be added into freemodbus folder to its place where it is located
# in original component. The patched file will be compiled instead of original one
# located in the official component freemodbus folder.

set(MB_PATCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

set(override_srcs "${MB_PATCH_DIR}/freemodbus/patch_lib.c")
set(comp_dir)
set(comp_name)
set(comp_kconfig)
set(optional_reqs freemodbus espressif__esp-modbus)
idf_build_get_property(build_components BUILD_COMPONENTS)

foreach(req ${optional_reqs})
    if(req IN_LIST build_components)
        idf_component_get_property(comp_lib ${req} COMPONENT_LIB)
        idf_component_get_property(comp_srcs ${req} SRCS)
        idf_component_get_property(comp_dir ${req} COMPONENT_DIR)
        idf_component_get_property(comp_name ${req} COMPONENT_NAME)
        idf_component_get_property(comp_kconfig ${req} KCONFIG)
        message(STATUS "The library ${comp_lib}, name: [${comp_name}], path: ${comp_dir} is found and will be overriden by files from [${COMPONENT_NAME}] component.")
    endif()
endforeach()

set(include_dirs common/include)

set(priv_include_dirs common port modbus modbus/ascii modbus/functions
                      modbus/rtu modbus/tcp modbus/include)

if(EXISTS "${comp_dir}")

    list(APPEND priv_include_dirs serial_slave/port serial_slave/modbus_controller
            serial_master/port serial_master/modbus_controller
            tcp_slave/port tcp_slave/modbus_controller
            tcp_master/port tcp_master/modbus_controller)

    add_prefix(include_dirs "${comp_dir}/freemodbus/" ${include_dirs})
    add_prefix(priv_include_dirs "${comp_dir}/freemodbus/" ${priv_include_dirs})

    if(IS_DIRECTORY ${MB_PATCH_DIR})
        foreach(src ${comp_srcs})
            get_filename_component(src_wo_ext ${src} NAME_WE)
            file(RELATIVE_PATH src_rel "${comp_dir}" "${src}")
            get_filename_component(src_dir ${src_rel} DIRECTORY)
            # message(STATUS "Source relative path:${src_rel} ")
            if(EXISTS "${MB_PATCH_DIR}/${src_rel}")
                message(STATUS "Override source: ${MB_PATCH_DIR}/${src_dir}/${src_wo_ext}.c")
                list(APPEND override_srcs ${src_dir}/${src_wo_ext}.c)
                # message(STATUS "header check: ${src_dir}/${src_wo_ext}.h")
                if((EXISTS "${MB_PATCH_DIR}/${src_dir}/${src_wo_ext}.h") AND (NOT ${MB_PATCH_DIR}/src_dir IN_LIST priv_include_dirs))
                    list(APPEND include_dirs "${MB_PATCH_DIR}/${src_dir}")
                    message(STATUS "Add override include: ${MB_PATCH_DIR}/${src_dir}/${src_wo_ext}.h ")
                endif()
            else()
                if(NOT src_dir IN_LIST priv_include_dirs)
                    list(APPEND priv_include_dirs "${comp_dir}/${src_dir}")
                endif()
                list(APPEND override_srcs "${comp_dir}/${src_rel}")
            endif()
        endforeach()
    endif()

    message(STATUS "DEBUG: Use esp-modbus component folder: ${CMAKE_CURRENT_LIST_DIR}.")
endif()

set(requires driver lwip)

# esp_timer component was introduced in v4.2
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND requires esp_timer)
endif()

idf_component_register(SRCS "${override_srcs}"
                        INCLUDE_DIRS "${include_dirs}"
                        PRIV_INCLUDE_DIRS "${priv_include_dirs}"
                        KCONFIG "${comp_kconfig}"
                        REQUIRES "${requires}"
                        PRIV_REQUIRES esp_netif "${comp_name}")

# This workaround allows to link this patched library into project in spite of availability of espressif_esp-modbus library
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY INTERFACE_LINK_LIBRARIES "-u mb_patch_lib_include")

