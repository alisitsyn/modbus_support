// This simple example demonstrates how to use UART HAL layer and collision detection HW to receive data in custom interrupt
#include <sys/param.h>
#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "soc/soc.h"
#include "soc/uart_struct.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "sdkconfig.h"
#include "freertos/ringbuf.h"

// RX TOUT interrupt after this number of symbols on current baudrate (end of message condition)
#define TEST_TOUT_SYMB          (5) 
#define TEST_RETRIES            (1000)
#define TEST_BUF_SIZE           (256)
#define TEST_UART_PORT_NUM      (CONFIG_UART_PORT_NUM)
#define TEST_TXD                (CONFIG_UART_TXD)
#define TEST_RXD                (CONFIG_UART_RXD)
#define TEST_RTS                (CONFIG_UART_RTS)
#define TEST_CTS                (UART_PIN_NO_CHANGE)
#define RXFIFO_FULL_THRD        (15)        // Set threshold for RX_FULL interrupt
#define TEST_RX_TIMEOUT_MS      (500)      // RX response timeout after sent request
#define TEST_RX_TOUT_TICKS      (pdMS_TO_TICKS(TEST_RX_TIMEOUT_MS))
#define TEST_TX_FIFO_SIZE       (256)

enum {
    PORT_NO_ERROR,
    PORT_COLLISION_DETECTED,
    PORT_RS485_FAIL,
    PORT_FRAME_TOUT,
    PORT_RESP_TOUT,
} port_flags;

typedef struct {
    uart_hal_context_t hal;        // UART HAL context
    portMUX_TYPE spin_lock;
    bool hw_enabled;
    uart_isr_handle_t uart_isr_handle;
    uint8_t uart_num;
    bool rx_state_enabled;
    bool tx_state_enabled;
    uint64_t send_time_stamp;
    uint64_t recv_time_stamp;
    uint32_t int_flags;
    RingbufHandle_t ring_buffer;
    bool collision_detected;
} port_context_t;

// The UART hardware context
static port_context_t port_context = {
        .hal.dev = NULL, \
        .spin_lock = portMUX_INITIALIZER_UNLOCKED, \
        .hw_enabled = false, \
        .uart_isr_handle = NULL, \
        .uart_num = (UART_NUM_MAX - 1), \
        .rx_state_enabled = false, \
        .tx_state_enabled = false, \
        .recv_time_stamp = 0, \
        .send_time_stamp = 0, \
        .int_flags = 0, \
        .ring_buffer = NULL, \
        .collision_detected = false
};

#define PORT_ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&port_context.spin_lock)
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&port_context.spin_lock)

// Workaround for hart_ll_get_rxfifo_len() which return incorrect number of bytes == 128 in some cases
FORCE_INLINE_ATTR uint32_t drv_get_rxfifo_len(uart_hal_context_t* hal)
{
    uint32_t fifo_cnt = HAL_FORCE_READ_U32_REG_FIELD(hal->dev->status, rxfifo_cnt);
    typeof(hal->dev->mem_rx_status) rx_status = hal->dev->mem_rx_status;
    uint32_t len = fifo_cnt;

    return len;
}

static IRAM_ATTR void drv_intr_handler(void *arg)
{
    static uint8_t data[TEST_BUF_SIZE + 1] = {0};
    uint32_t uart_status = 0;
    int size = 0;
    portBASE_TYPE HPTaskAwoken = pdTRUE;
    static int rxBytes = 0;
    
    portENTER_CRITICAL(&port_context.spin_lock);
    uart_status = uart_hal_get_intsts_mask(&port_context.hal);
    if (uart_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {
        uart_hal_clr_intsts_mask(&port_context.hal, UART_INTR_RXFIFO_FULL);
        //size = 0; // read all data from FIFO buffer
        size = drv_get_rxfifo_len(&port_context.hal);
        
        uart_hal_read_rxfifo(&port_context.hal, &data[rxBytes], &size);
        rxBytes += size;
        if (uart_status & UART_INTR_RXFIFO_FULL) {
            ESP_EARLY_LOGW("UART FULL INT", "UART get data %d bytes.", size);
        }
        if (uart_status & UART_INTR_RXFIFO_TOUT) {
            uart_hal_clr_intsts_mask(&port_context.hal, UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL);
            uart_hal_disable_intr_mask(&port_context.hal, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
            if (rxBytes) {
                if (pdFALSE == xRingbufferSendFromISR(port_context.ring_buffer, (const void*)data, (size_t)rxBytes, &HPTaskAwoken)) {
                    ESP_EARLY_LOGE("UART TOUT INT", "UART ring buffer overflow.");
                } else {
                    ESP_EARLY_LOGW("UART TOUT INT", "UART received %d bytes of data.", rxBytes);
                    rxBytes = 0;
                }
            }
        }
    } else if ((uart_status & UART_INTR_RS485_PARITY_ERR)
                    || (uart_status & UART_INTR_RS485_FRM_ERR)
                    || (uart_status & UART_INTR_RS485_CLASH)) {
            uart_hal_clr_intsts_mask(&port_context.hal, (UART_INTR_RS485_CLASH |
                                                    UART_INTR_RS485_FRM_ERR |
                                                    UART_INTR_RS485_PARITY_ERR));
            // RS485 collision or frame error interrupt triggered
            //ESP_EARLY_LOGW("UART_INT","RS485 ERR detected 0x%x.", uart_status);
            uart_hal_rxfifo_rst(&port_context.hal);
            // Set collision detection flag
            port_context.int_flags |= (uart_status & UART_INTR_RS485_CLASH) ?
                                        PORT_COLLISION_DETECTED : 0;
            port_context.int_flags |= (uart_status & (UART_INTR_RS485_FRM_ERR | UART_INTR_RS485_PARITY_ERR)) ?
                                        PORT_RS485_FAIL : 0;
            
    } else if (uart_status & UART_INTR_RXFIFO_OVF) {
        uart_hal_clr_intsts_mask(&port_context.hal, UART_INTR_RXFIFO_OVF);
        ESP_EARLY_LOGE("UART INT", "UART FIFO buffer overflow.");
        uart_hal_rxfifo_rst(&port_context.hal);
    } else if (uart_status & UART_INTR_BRK_DET) {
        uart_hal_clr_intsts_mask(&port_context.hal, UART_INTR_BRK_DET);
        //ESP_EARLY_LOGE("UART INT", "UART Brake condition detected.");
    } else {
        ESP_EARLY_LOGE("UART INT", "Unexpected UART interrupt %x triggered.", uart_status);
        uart_hal_clr_intsts_mask(&port_context.hal, uart_status);
    }
    portEXIT_CRITICAL(&port_context.spin_lock);
    
    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// Direction control over RTS pin
FORCE_INLINE_ATTR void drv_set_rts_rs485(uart_hal_context_t* hal, uint8_t level)
{
    // Enable receiver, sw_rts = 1  generates low level on RTS pin
    hal->dev->conf0.sw_rts = (level ^ 0x01);
}

// Overrides the set mode function to allow collision detection
FORCE_INLINE_ATTR void drv_set_mode_rs485_half_duplex(uart_hal_context_t* hal)
{
    // Enable receiver, sw_rts = 1  generates low level on RTS pin
    hal->dev->conf0.sw_rts = 1;
    // Must be set to 0 to automatically remove echo
    hal->dev->rs485_conf.tx_rx_en = 1;
    // Don't send while receiving => collision avoidance
    hal->dev->rs485_conf.rx_busy_tx_en = 0; 
    hal->dev->conf0.irda_en = 0;
    hal->dev->rs485_conf.en = 1;
}

bool drv_check_collision()
{
    PORT_ENTER_CRITICAL();
    bool collision_det = (bool)(port_context.int_flags & PORT_COLLISION_DETECTED);
    port_context.int_flags &= ~PORT_COLLISION_DETECTED;
    PORT_EXIT_CRITICAL();
    return collision_det;
}

esp_err_t drv_wait_tx_idle(uart_hal_context_t* hal)
{
    while (!uart_hal_is_tx_idle(hal));
    return ESP_OK;
}

void drv_switch_direction(uart_hal_context_t* hal, bool rx_enable) 
{
    if (rx_enable) {
        drv_set_rts_rs485(hal, 0);
        uart_hal_ena_intr_mask(hal, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    } else {
        drv_set_rts_rs485(hal, 1);
        uart_hal_disable_intr_mask(hal, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    }
}

// Sends the packet using HAL API and checks collisions.
// If the collision detected in the transmitted packet then returns true, otherwize false. 
bool drv_send_data_check_collision(uart_hal_context_t* hal, uint8_t* pbuf, size_t buf_size)
{
    uint32_t counter = buf_size;
    uint8_t* pdata = pbuf;
    
    drv_switch_direction(hal, false);
    port_context.collision_detected = false;
    // Write request to UART and then wait for response
    while(counter > 0) {
        size_t size = (counter > TEST_TX_FIFO_SIZE) ? TEST_TX_FIFO_SIZE : counter;
        size_t act_size = 0;
        hal->dev->conf0.rxfifo_rst = 1;
        uart_hal_write_txfifo(hal, (const uint8_t*)pdata, size, (uint32_t*)&act_size);
        if (act_size > 0) {
            drv_wait_tx_idle(hal);
            // clear the rx buffer to remove uart receive echo in current mode
            hal->dev->conf0.rxfifo_rst = 0;
            if (drv_check_collision()) {
                port_context.collision_detected = true;
                ESP_LOG_BUFFER_HEXDUMP("UART_COLLISON", pdata, act_size, ESP_LOG_ERROR);
            } else {
                //ESP_LOG_BUFFER_HEXDUMP("UART_SENT", pdata, act_size, ESP_LOG_INFO);
            }
            pdata = (uint8_t*)((void*)pdata + act_size);
            counter -= act_size;
        }
    }
    //
    hal->dev->conf0.rxfifo_rst = 0;
    drv_switch_direction(hal, true);
    
    return port_context.collision_detected;
}

static void drv_init(void)
{
    uart_config_t uart_conf;
    uart_conf.baud_rate = CONFIG_UART_BAUD_RATE;
    uart_conf.data_bits = UART_DATA_8_BITS;
    uart_conf.parity = UART_PARITY_DISABLE;
    uart_conf.stop_bits = UART_STOP_BITS_1;
    uart_conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_conf.source_clk = UART_SCLK_APB;
    port_context.hal.dev = UART_LL_GET_HW(TEST_UART_PORT_NUM); // initialize the driver HAL layer
    port_context.uart_num = TEST_UART_PORT_NUM;
    ESP_ERROR_CHECK(uart_driver_install(port_context.uart_num, 129, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port_context.uart_num, &uart_conf));
    ESP_ERROR_CHECK(uart_set_pin(port_context.uart_num, TEST_TXD, TEST_RXD, TEST_RTS, TEST_CTS));
    ESP_ERROR_CHECK(uart_isr_free(port_context.uart_num));
    ESP_ERROR_CHECK(uart_isr_register(port_context.uart_num, drv_intr_handler, NULL, ESP_INTR_FLAG_IRAM, &port_context.uart_isr_handle));
    uart_hal_set_rx_timeout(&port_context.hal, TEST_TOUT_SYMB);
    uart_hal_set_rxfifo_full_thr(&port_context.hal, RXFIFO_FULL_THRD);
    // enable RX interrupt
    uart_hal_ena_intr_mask(&port_context.hal, UART_INTR_RXFIFO_TOUT
                               | UART_INTR_RXFIFO_FULL
                               | UART_INTR_RS485_CLASH
                               | UART_INTR_RS485_FRM_ERR
                               | UART_INTR_RS485_PARITY_ERR);
    drv_set_mode_rs485_half_duplex(&port_context.hal);
    port_context.ring_buffer = xRingbufferCreate(TEST_BUF_SIZE * 2, RINGBUF_TYPE_NOSPLIT);
    if (!port_context.ring_buffer) {
        ESP_LOGE("UART_TASK", "Ring Buffer create fail.");
    }
}

void drv_close(void)
{
    vRingbufferDelete(port_context.ring_buffer);
    ESP_ERROR_CHECK(uart_driver_delete(port_context.uart_num));
    ESP_ERROR_CHECK(esp_intr_free(port_context.uart_isr_handle));
    portENTER_CRITICAL(&port_context.spin_lock);
    port_context.hal.dev = NULL;
    portEXIT_CRITICAL(&port_context.spin_lock);
}

void drv_fill_buffer(uint8_t* buf, size_t size)
{
    uint32_t random_word = 0;
    // for (uint32_t i = 0; i < size; i += 4) {
    //     //random_word = esp_random();
    //     //memcpy(buf + i, &random_word, MIN(size - i, 4));
    //     buf[i] = (uint8_t)(i & 0x000000FF);
    // }  
    for (uint32_t i = 0; i < size; i++) {
        buf[i] = (uint8_t)(i & 0x000000FF);
    }  
}

void app_main()
{
    size_t size = 0;
    drv_init();
    uint8_t* data_buffer = NULL;
    uint8_t* psend_buf = (uint8_t*) malloc(TEST_BUF_SIZE);
    int cycle_counter = 0;

    while(cycle_counter++ < TEST_RETRIES)
    {
        for(int i = 1; i < 3; i++) {
            drv_fill_buffer(psend_buf, TEST_TX_FIFO_SIZE);
            // We do not need to check the collision here, let us to it inside the send function
            drv_send_data_check_collision(&port_context.hal, (uint8_t*)psend_buf, TEST_TX_FIFO_SIZE);
        }
        // Trying to receive response from receive ring buffer
        data_buffer = (uint8_t*)xRingbufferReceive(port_context.ring_buffer, &size, (portTickType)TEST_RX_TOUT_TICKS);
        if (!data_buffer) {
            ESP_LOGW("UART_TASK", "Receive buffer is empty.");
        } else {
            // We get buffered data into the Ring Buffer, return the data and print
            vRingbufferReturnItem(port_context.ring_buffer, (void *)data_buffer);
            ESP_LOGI("UART_TASK", "Ring Buffer receive size: %x:(%d byte)", (uint32_t)data_buffer, (size_t)size);
            ESP_LOG_BUFFER_HEXDUMP("UART_TASK", data_buffer, size, ESP_LOG_INFO);
        }
        
        xRingbufferPrintInfo(port_context.ring_buffer);
    }
    ESP_LOGI("UART_TASK", "Close UART....");
    drv_close();
}


