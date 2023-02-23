/* UART Terminal Example
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdi12_bus.h"

#include "sdkconfig.h"

/**
 * This is an example which any data receives on configured UART is send to SDI12 bus,
 * and SDI21 bus data received is sent back to UART. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define TERMINAL_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define TERMINAL_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define TERMINAL_TEST_RTS (UART_PIN_NO_CHANGE)
#define TERMINAL_TEST_CTS (UART_PIN_NO_CHANGE)

#define TERMINAL_UART_PORT_NUM   (CONFIG_EXAMPLE_UART_PORT_NUM)
#define TERMINAL_UART_BAUD_RATE  (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define TERMINAL_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define SDI12_DATA_GPIO CONFIG_EXAMPLE_SDI12_BUS_GPIO

#define BUF_SIZE (1024)

static const char *TAG = "SDI12-UART-TERMINAL";

static char response[85] = "";
static sdi12_bus_handle_t sdi12_bus;

static void uart_terminal_task(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    bool crc = false;

    while (1)
    {
        // Read data from the UART
        esp_err_t ret = ESP_FAIL;

        // int len = uart_read_bytes(TERMINAL_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        int len = uart_read_bytes(TERMINAL_UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(25));
        if (len > 0)
        {
            data[len] = '\0';
            ESP_LOGI(TAG, "Got data (%d bytes): %s", len, data);

            ret = sdi12_bus_send_cmd(sdi12_bus, (const char *)data, crc, response, sizeof(response), SDI12_DEFAULT_RESPONSE_TIMEOUT);
            uart_write_bytes(TERMINAL_UART_PORT_NUM, (const char *)response, strlen(response));

            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "SDI error: %s", esp_err_to_name(ret));
            }

            if (data[1] == 'M' || data[1] == 'C' || data[1] == 'R')
            {
                crc = data[2] == 'C' ? true : false;
            }
        }

        // Write data back to the UART
    }
}

void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = TERMINAL_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(TERMINAL_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(TERMINAL_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TERMINAL_UART_PORT_NUM, TERMINAL_TEST_TXD, TERMINAL_TEST_RXD, TERMINAL_TEST_RTS, TERMINAL_TEST_CTS));

    ESP_LOGI(TAG, "UART %d initialization DONE on pins TX: %d | RX: %d", TERMINAL_UART_PORT_NUM, TERMINAL_TEST_TXD, TERMINAL_TEST_RXD);

    sdi12_bus_config_t config = {
        .gpio_num = SDI12_DATA_GPIO,
        .bus_timing = { 
            .post_break_marking_us = 9000, 
        },
    };

    ESP_ERROR_CHECK(sdi12_new_bus(&config, &sdi12_bus));

    ESP_LOGI(TAG, "SDI12 BUS initialization DONE");

    xTaskCreate(uart_terminal_task, "uart_terminal_task", TERMINAL_TASK_STACK_SIZE, NULL, 10, NULL);
}
