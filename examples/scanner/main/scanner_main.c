#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "sdi12_bus.h"

#define SDI12_DATA_GPIO GPIO_NUM_2
#define SDI12_RMT_TX_CHANNEL 0
#define SDI12_RMT_RX_CHANNEL 0

static const char *TAG = "SDI12-SCANNER";
static char response[85] = {0};

uint16_t find_devices_in_range(sdi12_bus_t *bus, char start_address, char end_address)
{
    uint16_t devs = 0;
    char cmd_ack[] = "_!";
    char cmd_id[] = "_I!";

    for (char addr = start_address; addr <= end_address; ++addr)
    {
        cmd_ack[0] = addr;
        cmd_id[0] = addr;

        esp_err_t ret = sdi12_bus_send_cmd(bus, cmd_ack, response, sizeof(response), false, 500);

        if (ret == ESP_OK && strlen(response) > 0 && response[0] == addr)
        {
            ++devs;

            ret = sdi12_bus_send_cmd(bus, cmd_id, response, sizeof(response), false, 0);

            if (ret == ESP_OK && strlen(response) > 0)
            {
                ESP_LOGI(TAG, "Address: %c\tId: %s", addr, response + 1);
            }
        }
    }

    return devs;
}

void app_main(void)
{
    sdi12_bus_config_t config = {
        .gpio_num = SDI12_DATA_GPIO,
        .rmt_tx_channel = SDI12_RMT_TX_CHANNEL,
        .rmt_rx_channel = SDI12_RMT_RX_CHANNEL,
        .bus_timing = {
            .post_break_marking_us = 9000}};

    sdi12_bus_t *bus = sdi12_bus_init(&config);

    ESP_LOGI(TAG, "Scanning...");

    uint16_t devs = 0;

    devs += find_devices_in_range(bus, '0', '9');
    devs += find_devices_in_range(bus, 'a', 'z');
    devs += find_devices_in_range(bus, 'A', 'Z');

    ESP_LOGI(TAG, "End scan. Found %d devices", devs);
}