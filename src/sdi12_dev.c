#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "sdi12_defs.h"
#include "sdi12_dev.h"

#if CONFIG_SDI12_ENABLE_DEBUG_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif

typedef struct
{
    sdi12_version_t sdi12_version;
    char *vendor_id;
    char *model;
    char *model_version;
    char *optional;
} sdi12_dev_info_t;

typedef struct sdi12_dev
{
    char address;
    sdi12_dev_info_t info;
    sdi12_bus_handle_t bus;
} sdi12_dev_t;

static const char *TAG = "sdi12-dev";

static esp_err_t check_address(sdi12_dev_handle_t dev, char *buffer)
{
    return dev->address == buffer[0] ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t parse_info(sdi12_dev_handle_t dev, char *info_buffer)
{
    if (dev->info.vendor_id)
    {
        free(dev->info.vendor_id);
    }

    if (dev->info.model)
    {
        free(dev->info.model);
    }

    if (dev->info.model_version)
    {
        free(dev->info.model_version);
    }

    if (dev->info.optional)
    {
        free(dev->info.optional);
    }

    switch ((info_buffer[1] - '0') * 10 + (info_buffer[2] - '0'))
    {
        case 13:
            dev->info.sdi12_version = SDI12_VERSION_1_3;
            break;
        case 14:
            dev->info.sdi12_version = SDI12_VERSION_1_4;
            break;
        default:
            dev->info.sdi12_version = SDI12_VERSION_UNKNOWN;
            break;
    }

    dev->info.vendor_id = strndup(info_buffer + 3, 8);
    dev->info.model = strndup(info_buffer + 11, 6);
    dev->info.model_version = strndup(info_buffer + 17, 3);
    dev->info.optional = strdup(info_buffer + 20);

    return ESP_OK;
}

esp_err_t sdi12_dev_get_address(sdi12_dev_handle_t dev, char *out_address)
{
    ESP_RETURN_ON_FALSE(dev && out_address, ESP_ERR_INVALID_ARG, TAG, "invalid args");

    *out_address = dev->address;

    return ESP_OK;
}

esp_err_t sdi12_dev_get_sdi_version(sdi12_dev_handle_t dev, sdi12_version_t *out_version)
{
    ESP_RETURN_ON_FALSE(dev && out_version, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    *out_version = dev->info.sdi12_version;

    return ESP_OK;
}

esp_err_t sdi12_dev_get_vendor_id(sdi12_dev_handle_t dev, char *out_vendor_id)
{
    ESP_RETURN_ON_FALSE(dev && out_vendor_id, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    out_vendor_id = dev->info.vendor_id;

    return ESP_OK;
}

esp_err_t sdi12_dev_get_model(sdi12_dev_handle_t dev, char *out_model)
{
    ESP_RETURN_ON_FALSE(dev && out_model, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    out_model = dev->info.model;

    return ESP_OK;
}

esp_err_t sdi12_dev_get_model_version(sdi12_dev_handle_t dev, char *out_model_version)
{
    ESP_RETURN_ON_FALSE(dev && out_model_version, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    out_model_version = dev->info.model_version;

    return ESP_OK;
}

esp_err_t sdi12_dev_get_optional_info(sdi12_dev_handle_t dev, char *out_optional_field)
{
    ESP_RETURN_ON_FALSE(dev && out_optional_field, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    out_optional_field = dev->info.optional;

    return ESP_OK;
}

esp_err_t sdi12_dev_acknowledge_active(sdi12_dev_handle_t dev, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");

    char cmd[] = "_!";
    cmd[0] = dev->address;

    char out_buffer[3]; // Response should be a<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        return check_address(dev, out_buffer);
    }

    return ret;
}

esp_err_t sdi12_dev_change_address(sdi12_dev_handle_t dev, char new_address, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE(
        ((new_address >= '0' && new_address <= '9') || (new_address >= 'a' && new_address <= 'z') || (new_address >= 'A' && new_address <= 'Z')),
        ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalidad new sensor address", dev->address);

    char cmd[] = "_A_!";
    cmd[0] = dev->address;
    cmd[2] = new_address;

    char out_buffer[3]; // Response should be 'new address'<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        if (out_buffer[0] == new_address)
        {
            dev->address = new_address;
        }
        else
        {
            ret = ESP_ERR_INVALID_RESPONSE;
        }
    }

    return ret;
}

esp_err_t sdi12_dev_read_identification(sdi12_dev_handle_t dev, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");

    char cmd[] = "_I!";
    cmd[0] = dev->address;
    esp_err_t ret;

    // Response should be 'info'<CR><LF>. Info maximum length is 34
    if (out_buffer)
    {
        ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, out_buffer_length, timeout);

        if (ret == ESP_OK)
        {
            ret = check_address(dev, out_buffer);

            if (ret == ESP_OK)
            {
                parse_info(dev, out_buffer);
            }
        }
    }
    else
    {
        char temp_buf[38];
        ret = sdi12_bus_send_cmd(dev->bus, cmd, false, temp_buf, sizeof(temp_buf), timeout);

        if (ret == ESP_OK)
        {
            ret = check_address(dev, temp_buf);

            if (ret == ESP_OK)
            {
                parse_info(dev, temp_buf);
            }
        }
    }

    return ret;
}

esp_err_t sdi12_dev_address_query(sdi12_dev_handle_t dev, char *address, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE(address, ESP_ERR_INVALID_ARG, TAG, "out address is null");

    char cmd[] = "?!";

    char out_buffer[3]; // Response should be 'address'<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        *address = out_buffer[0];
    }

    return ret;
}

esp_err_t sdi12_dev_start_measurement(sdi12_dev_handle_t dev, uint8_t m_index, bool crc, uint8_t *n_params, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE((m_index <= 9), ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalid M index", dev->address);

    char cmd[6] = "";
    uint8_t index = 0;

    cmd[index++] = dev->address;
    cmd[index++] = 'M';

    if (crc)
    {
        cmd[index++] = 'C';
    }

    if (m_index != 0)
    {
        cmd[index++] = m_index + '0';
    }

    cmd[index++] = '!';
    cmd[index] = '\0';

    char out_buffer[8]; // Response should be 'atttn'<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, crc, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);

        if (ret == ESP_OK && n_params)
        {
            *n_params = out_buffer[4] - '0';
        }
    }

    return ret;
}

esp_err_t sdi12_dev_read_data(sdi12_dev_handle_t dev, uint8_t d_index, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE((d_index <= 9), ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalid D index", dev->address);

    char cmd[5] = "";
    uint8_t index = 0;

    cmd[index++] = dev->address;
    cmd[index++] = 'D';
    cmd[index++] = d_index + '0';
    cmd[index++] = '!';
    cmd[index] = '\0';

    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, crc, out_buffer, out_buffer_length, timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);
    }

    return ret;
}

esp_err_t sdi12_dev_start_verification(sdi12_dev_handle_t dev, uint8_t *n_params, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");

    char cmd[] = "_V!";
    cmd[0] = dev->address;

    char out_buffer[8]; // Response should be 'atttn'<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);

        if (ret == ESP_OK && n_params)
        {
            *n_params = out_buffer[4] - '0';
        }
    }

    return ret;
}

esp_err_t sdi12_dev_start_concurrent_measurement(sdi12_dev_handle_t dev, const uint8_t c_index, bool crc, uint8_t *n_params, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE(c_index <= 9, ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalid C index", dev->address);

    char cmd[6] = "";
    uint8_t index = 0;

    cmd[index++] = dev->address;
    cmd[index++] = 'C';

    if (crc)
    {
        cmd[index++] = 'C';
    }

    if (c_index != 0)
    {
        cmd[index++] = c_index + '0';
    }

    cmd[index++] = '!';
    cmd[index] = '\0';

    char out_buffer[8]; // Response should be 'atttnn'<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, crc, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);

        if (ret == ESP_OK && n_params)
        {
            *n_params = (uint8_t)strtol(out_buffer + 5, NULL, 0);
        }
    }

    return ret;
}

esp_err_t sdi12_dev_read_continuos_measurement(sdi12_dev_handle_t dev, const uint8_t r_index, bool crc, char *out_buffer, size_t out_buffer_length,
    uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE(r_index <= 9, ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalid R index", dev->address);

    char cmd[5] = "";
    uint8_t index = 0;

    cmd[index++] = dev->address;
    cmd[index++] = 'R';
    cmd[index++] = r_index + '0';
    cmd[index++] = '!';
    cmd[index] = '\0';

    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, crc, out_buffer, out_buffer_length, timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);
    }

    return ret;
}

esp_err_t sdi12_dev_extended_cmd(sdi12_dev_handle_t dev, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE(cmd, ESP_ERR_INVALID_ARG, TAG, "invalid cmd");

    uint8_t len = strlen(cmd);

    char full_cmd[10];
    snprintf(full_cmd, len + 3, "%c%s!", dev->address, cmd);

    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, full_cmd, crc, out_buffer, out_buffer_length, timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);
    }

    return ret;
}

esp_err_t sdi12_dev_read_identify_cmd(sdi12_dev_handle_t dev, const char *cmd, uint8_t *n_params, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "device is null");
    ESP_RETURN_ON_FALSE(n_params, ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalid n_params arg", dev->address);

    uint8_t len = strlen(cmd);
    ESP_RETURN_ON_FALSE(len > 3, ESP_ERR_INVALID_ARG, TAG, "addr: %c, invalid cmd", dev->address);

    char full_cmd[8];
    snprintf(full_cmd, 7, "%cI%s!", dev->address, cmd);

    char out_buffer[10]; // Response should be 'atttn', 'atttnn' or 'atttnnn' plus <CR><LF>

    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, full_cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);

        if (ret == ESP_OK && n_params)
        {
            *n_params = (uint8_t)strtol(out_buffer + 5, NULL, 10);
        }
    }

    return ret;
}

void sdi12_del_dev(sdi12_dev_handle_t dev)
{
    if (!dev)
    {
        return;
    }

    if (dev->info.vendor_id)
    {
        free(dev->info.vendor_id);
    }

    if (dev->info.model)
    {
        free(dev->info.model);
    }

    if (dev->info.model_version)
    {
        free(dev->info.model_version);
    }

    if (dev->info.optional)
    {
        free(dev->info.optional);
    }

    free(dev);
}

esp_err_t sdi12_new_dev(sdi12_bus_handle_t bus, char address, sdi12_dev_handle_t *dev_out)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "invalid sdi12 bus");
    ESP_RETURN_ON_FALSE(((address >= '0' && address <= '9') || (address >= 'a' && address <= 'z') || (address >= 'A' && address <= 'Z') || address == '?'),
        ESP_ERR_INVALID_ARG, TAG, "invalidad sensor address");

    sdi12_dev_t *dev = calloc(1, sizeof(sdi12_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "can't allocate SDI12 device");

    dev->bus = bus;

    if (address == '?')
    {
        if (sdi12_dev_address_query(dev, &dev->address, 500) != ESP_OK)
        {
            ESP_LOGE(TAG, "can't find address error");
            goto err_cmd;
        }
    }
    else
    {
        dev->address = address;

        if (sdi12_dev_acknowledge_active(dev, 500) != ESP_OK)
        {
            ESP_LOGE(TAG, "can't find sensor with address '%c'", address);
            goto err_cmd;
        }
    }

    *dev_out = dev;
    return ESP_OK;

err_cmd:
    sdi12_del_dev(dev);
    return ESP_FAIL;
}
