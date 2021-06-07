// This simple example demonstrates how to use UART HAL layer to receive data in custom interrupt

#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/ringbuf.h"

// RX TOUT interrupt after this number of symbols on current baudrate (end of message condition)
#define TEST_TOUT_SYMB          (5) 
#define TEST_RETRIES            (15)
#define TEST_BUF_SIZE           (256)
#define TEST_UART_PORT_NUM      (CONFIG_UART_PORT_NUM)
#define TEST_TXD                (CONFIG_UART_TXD)
#define TEST_RXD                (CONFIG_UART_RXD)
#define TEST_RTS                (UART_PIN_NO_CHANGE)
#define TEST_CTS                (UART_PIN_NO_CHANGE)
#define RXFIFO_FULL_THRD        (15)        // Set threshold for RX_FULL interrupt
#define TEST_RX_TIMEOUT_MS      (1000)      // RX response timeout after sent request
#define TEST_RX_TOUT_TICKS      (pdMS_TO_TICKS(TEST_RX_TIMEOUT_MS))

static uart_hal_context_t hal;
static uart_isr_handle_t isr_handle;
static portMUX_TYPE spin_lock = portMUX_INITIALIZER_UNLOCKED ;
RingbufHandle_t ring_buffer;

static IRAM_ATTR void uart_intr_handle(void *arg)
{
	static uint8_t data[TEST_BUF_SIZE + 1] = {0};
    uint32_t uart_status = 0;
    int size = 0;
    portBASE_TYPE HPTaskAwoken = pdTRUE;
    static int rxBytes = 0;
    
    portENTER_CRITICAL(&spin_lock);
    uart_status = uart_hal_get_intsts_mask(&hal);
    if (uart_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {
        uart_hal_clr_intsts_mask(&hal, UART_INTR_RXFIFO_FULL);
        size = -1; // read all data from FIFO buffer
        uart_hal_read_rxfifo(&hal, &data[rxBytes], &size);
        rxBytes += size;
        if (uart_status & UART_INTR_RXFIFO_FULL) {
            ESP_EARLY_LOGI("UART FULL INT", "UART get data %d bytes.", size);
        }
        if (uart_status & UART_INTR_RXFIFO_TOUT) {
            uart_hal_clr_intsts_mask(&hal, UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL);
            uart_hal_disable_intr_mask(&hal, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
            if (pdFALSE == xRingbufferSendFromISR(ring_buffer, (const void*)data, (size_t)rxBytes, &HPTaskAwoken)) {
                ESP_EARLY_LOGE("UART TOUT INT", "UART ring buffer overflow.");
            } else {
                ESP_EARLY_LOGI("UART TOUT INT", "UART received %d bytes of data.", rxBytes);
                rxBytes = 0;
            }
        }
    } else if (uart_status & UART_INTR_RXFIFO_OVF) {
        ESP_EARLY_LOGE("UART INT", "UART FIFO buffer overflow.");
        uart_hal_rxfifo_rst(&hal);
    } else {
        ESP_EARLY_LOGE("UART INT", "Unexpected UART interrupt %x triggered.", uart_status);
        uart_hal_clr_intsts_mask(&hal, uart_status);
    }
    portEXIT_CRITICAL(&spin_lock);
    
    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void uart_init(void)
{
	uart_config_t uart_conf;
	uart_conf.baud_rate = CONFIG_UART_BAUD_RATE;
	uart_conf.data_bits = UART_DATA_8_BITS;
	uart_conf.parity = UART_PARITY_DISABLE;
	uart_conf.stop_bits = UART_STOP_BITS_1;
	uart_conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_conf.source_clk = UART_SCLK_APB;
    hal.dev = UART_LL_GET_HW(TEST_UART_PORT_NUM); // initialize the driver HAL layer
	ESP_ERROR_CHECK(uart_driver_install(TEST_UART_PORT_NUM, 129, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(TEST_UART_PORT_NUM, &uart_conf));
	ESP_ERROR_CHECK(uart_set_pin(TEST_UART_PORT_NUM, TEST_TXD, TEST_RXD, TEST_RTS, TEST_CTS));
	ESP_ERROR_CHECK(uart_isr_free(TEST_UART_PORT_NUM));
	ESP_ERROR_CHECK(uart_isr_register(TEST_UART_PORT_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &isr_handle)); 
    uart_hal_set_rx_timeout(&hal, TEST_TOUT_SYMB);
    uart_hal_set_rxfifo_full_thr(&hal, RXFIFO_FULL_THRD);
    uart_hal_ena_intr_mask(&hal, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT); // enable RX interrupt
    ring_buffer = xRingbufferCreate(TEST_BUF_SIZE * 2, RINGBUF_TYPE_NOSPLIT);
    if (!ring_buffer) {
        ESP_LOGE("UART_TASK", "Ring Buffer create fail.");
    }
}


void uart_close(void)
{
    vRingbufferDelete(ring_buffer);
    ESP_ERROR_CHECK(uart_driver_delete(TEST_UART_PORT_NUM));
    ESP_ERROR_CHECK(esp_intr_free(isr_handle));
    portENTER_CRITICAL(&spin_lock);
    hal.dev = NULL;
    portEXIT_CRITICAL(&spin_lock);
}
    
void app_main()
{
    size_t size = 0;
    uart_init();
    uint8_t* data_buffer = NULL;
    const char* send_str = "Hello world!!!!\r\n\0";
    int cycle_counter = 0;

    while(cycle_counter++ < TEST_RETRIES)
    {
        // Write request to UART and then wait for response
        uart_hal_write_txfifo(&hal, (const uint8_t*)send_str, strlen(send_str), &size);
        if (size > 0) {
            ESP_LOGI("UART_TASK", "Send string: %s", send_str);
            // Enable RX interrupts to get response
            uart_hal_ena_intr_mask(&hal, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
        }
        // Trying to receive response during timeout
        data_buffer = (uint8_t*)xRingbufferReceive(ring_buffer, &size, (portTickType)TEST_RX_TOUT_TICKS);
        if (!data_buffer) {
            ESP_LOGE("UART_TASK", "Ring Buffer receive fail or timeout.");
        } else {
            // We get buffered data into the Ring Buffer, return the data and print
            vRingbufferReturnItem(ring_buffer, (void *)data_buffer);
            ESP_LOGI("UART_TASK", "Ring Buffer receive size: %x:(%d byte)", (uint32_t)data_buffer, (size_t)size);
            ESP_LOG_BUFFER_HEXDUMP("UART_TASK", data_buffer, size, ESP_LOG_INFO);
        }
        xRingbufferPrintInfo(ring_buffer);
    }
    ESP_LOGI("UART_TASK", "Close UART....");
    uart_close();
}


