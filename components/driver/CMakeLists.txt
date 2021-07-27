#if(${IDF_VERSION} MATCHES "v4.0")

set(DRV_SRC_DIR "${IDF_PATH}/components/driver")
set(DRV_INC_DIRS "${IDF_PATH}/components/driver/include/driver" 
                    "${IDF_PATH}/components/driver/include" 
                    "${IDF_PATH}/components/driver" 
                    "${IDF_PATH}/components/newlib/include"
                    )
                    
message(STATUS , "Driver dir: ${DRV_SRC_DIR}")
message(STATUS , "Driver include dir: ${DRV_INC_DIRS}")

set(srcs 
    "${DRV_SRC_DIR}/can.c"
    "${DRV_SRC_DIR}/gpio.c"
    "${DRV_SRC_DIR}/i2c.c"
    "${DRV_SRC_DIR}/i2s.c"
    "${DRV_SRC_DIR}/ledc.c"
    "${DRV_SRC_DIR}/mcpwm.c"
    "${DRV_SRC_DIR}/pcnt.c"
    "${DRV_SRC_DIR}/periph_ctrl.c"
    "${DRV_SRC_DIR}/rmt.c"
    "${DRV_SRC_DIR}/rtc_module.c"
    "${DRV_SRC_DIR}/sdio_slave.c"
    "${DRV_SRC_DIR}/sdmmc_host.c"
    "${DRV_SRC_DIR}/sdmmc_transaction.c"
    "${DRV_SRC_DIR}/sdspi_crc.c"
    "${DRV_SRC_DIR}/sdspi_host.c"
    "${DRV_SRC_DIR}/sdspi_transaction.c"
    "${DRV_SRC_DIR}/sigmadelta.c"
    "${DRV_SRC_DIR}/spi_common.c"
    "${DRV_SRC_DIR}/spi_master.c"
    "${DRV_SRC_DIR}/spi_slave.c"
    "${DRV_SRC_DIR}/timer.c"
    "uart.c")
message(STATUS , "Driver sources: ${srcs}")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include" "${IDF_PATH}/components/driver/include"
                    PRIV_INCLUDE_DIRS ${DRV_INC_DIRS}
                    REQUIRES esp_ringbuf soc) #cannot totally hide soc headers, since there are a lot arguments in the driver are chip-dependent

if(GCC_NOT_5_2_0)
    # uses C11 atomic feature
    set_source_files_properties(spi_master.c PROPERTIES COMPILE_FLAGS -std=gnu11)
endif()

#endif()