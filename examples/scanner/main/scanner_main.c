#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

#include "sdi12_bus.h"

#define SDI12_DATA_GPIO CONFIG_EXAMPLE_SDI12_BUS_GPIO

static const char *TAG = "SDI12-SCANNER";
static char response[85] = { 0 };

uint16_t find_devices_in_range(sdi12_bus_handle_t bus, char start_address, char end_address)
{
    uint16_t devs = 0;
    char cmd_ack[] = "_!";
    char cmd_id[] = "_I!";

    for (char addr = start_address; addr <= end_address; ++addr)
    {
        cmd_ack[0] = addr;
        cmd_id[0] = addr;

        esp_err_t ret = sdi12_bus_send_cmd(bus, cmd_ack, false, response, sizeof(response), 500);

        if (ret == ESP_OK && strlen(response) > 0 && response[0] == addr)
        {
            ++devs;

            ret = sdi12_bus_send_cmd(bus, cmd_id, false, response, sizeof(response), 0);

            if (ret == ESP_OK && strlen(response) > 0)
            {
                ESP_LOGI(TAG, "Address: %c\tId: %s", addr, response + 1);
            }
        }
        else
        {
            ESP_LOGI(TAG, "Address: %c\tNo found", addr);
        }
    }

    return devs;
}

void app_main(void)
{
    sdi12_bus_config_t config = {
        .gpio_num = SDI12_DATA_GPIO,
        .bus_timing = { 
            .post_break_marking_us = 9000,
        },
    };

    sdi12_bus_handle_t sdi12_bus;

    ESP_ERROR_CHECK(sdi12_new_bus(&config, &sdi12_bus));

    ESP_LOGI(TAG, "Scanning...");

    uint16_t devs = 0;

    devs += find_devices_in_range(sdi12_bus, '0', '9');
    // devs += find_devices_in_range(sdi12_bus, 'a', 'z');
    // devs += find_devices_in_range(sdi12_bus, 'A', 'Z');

    ESP_LOGI(TAG, "End scan. Found %d devices", devs);
}