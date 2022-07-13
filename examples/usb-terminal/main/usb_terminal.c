/* USB Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// DESCRIPTION:
// This example contains minimal code to make ESP32-S2 based device
// recognizable by USB-host devices as a USB Serial Device.

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "esp_log.h"

#include "sdi12.h"

#include "sdkconfig.h"

#define SDI12_DATA_GPIO GPIO_NUM_17
#define SDI12_RMT_TX_CHANNEL 0
#define SDI12_RMT_RX_CHANNEL 4

static sdi12_bus_t *sdi12_bus;

static const char *TAG = "sdi12-usb-terminal";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
static char response[85] = "";

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;
    memset(response, '\0', sizeof(response));
    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
    } else {
        ESP_LOGE(TAG, "Read error");
    }

    sdi12_bus->raw_cmd(sdi12_bus, (const char *)buf, response, sizeof(response), 0);
    tinyusb_cdcacm_write_queue(itf, (const uint8_t *)response, strlen(response));
    tinyusb_cdcacm_write_flush(itf, 50);
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! dtr:%d, rst:%d", dtr, rst);
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = {}; // the configuration using default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    sdi12_bus_config_t config = {
       .gpio_num = SDI12_DATA_GPIO,
       .rmt_tx_channel = SDI12_RMT_TX_CHANNEL,
       .rmt_rx_channel = SDI12_RMT_RX_CHANNEL
   };

   sdi12_bus = sdi12_bus_init(&config);

   ESP_LOGI(TAG, "SDI12 BUS initialization DONE");
}
