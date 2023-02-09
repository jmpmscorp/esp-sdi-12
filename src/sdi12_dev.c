#include <string.h>

#if CONFIG_SDI12_ENABLE_DEBUG_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif

#include "sdi12_defs.h"
#include "sdi12_dev.h"
#include "esp_log.h"

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
}sdi12_dev_t;

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

char sdi12_dev_get_address(sdi12_dev_handle_t dev)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    return dev->address;
err_args:
    return '?';
}

sdi12_version_t sdi12_dev_get_sdi_version(sdi12_dev_handle_t dev)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    return dev->info.sdi12_version;

err_args:
    return SDI12_VERSION_UNKNOWN;
}

char *sdi12_dev_get_vendor_id(sdi12_dev_handle_t dev)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    return dev->info.vendor_id;

err_args:
    return NULL;
}

char *sdi12_dev_get_model_version(sdi12_dev_handle_t dev)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    return dev->info.model_version;

err_args:
    return NULL;
}

char *sdi12_dev_get_model(sdi12_dev_handle_t dev)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    return dev->info.model;

err_args:
    return NULL;
}

char *sdi12_dev_get_optional_info(sdi12_dev_handle_t dev)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    return dev->info.optional;

err_args:
    return NULL;
}

esp_err_t sdi12_dev_acknowledge_active(sdi12_dev_handle_t dev, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);

    char cmd[] = "_!";
    cmd[0] = dev->address;

    char out_buffer[3]; // Response should be a<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        return check_address(dev, out_buffer);
    }

    return ret;
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_change_address(sdi12_dev_handle_t dev, char new_address, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK(((new_address >= '0' && new_address <= '9') || (new_address >= 'a' && new_address <= 'z') || (new_address >= 'A' && new_address <= 'Z')),
        "Invalidad new sensor address", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_read_identification(sdi12_dev_handle_t dev, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_address_query(sdi12_dev_handle_t dev, char *address, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK(address, "NULL address", err_args);

    char cmd[] = "?!";

    char out_buffer[3]; // Response should be 'address'<CR><LF>
    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, cmd, false, out_buffer, sizeof(out_buffer), timeout);

    if (ret == ESP_OK)
    {
        *address = out_buffer[0];
    }

    return ret;
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_start_measurement(sdi12_dev_handle_t dev, uint8_t m_index, bool crc, uint8_t *n_params, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK((m_index <= 9), "Invalid M index", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_read_data(sdi12_dev_handle_t dev, uint8_t d_index, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK((d_index <= 9), "Invalid D index", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_start_verification(sdi12_dev_handle_t dev, uint8_t *n_params, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_start_concurrent_measurement(sdi12_dev_handle_t dev, const uint8_t c_index, bool crc, uint8_t *n_params, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK(c_index <= 9, "Invalid C index", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_read_continuos_measurement(sdi12_dev_handle_t dev, const uint8_t r_index, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK(r_index <= 9, "Invalid R index", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_extended_cmd(sdi12_dev_handle_t dev, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK(cmd, "Invalid CMD", err_args);
    uint8_t len = strlen(cmd);

    char full_cmd[10];
    snprintf(full_cmd, len + 3, "%c%s!", dev->address, cmd);

    esp_err_t ret = sdi12_bus_send_cmd(dev->bus, full_cmd, crc, out_buffer, out_buffer_length, timeout);

    if (ret == ESP_OK)
    {
        ret = check_address(dev, out_buffer);
    }

    return ret;

err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_dev_read_identify_cmd(sdi12_dev_handle_t dev, const char *cmd, uint8_t *n_params, uint32_t timeout)
{
    SDI12_CHECK(dev, "NULL dev", err_args);
    SDI12_CHECK(n_params, "Invalid n_params arg", err_args);

    uint8_t len = strlen(cmd);
    SDI12_CHECK(len > 3, "Invalid cmd", err_args);

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
err_args:
    return ESP_ERR_INVALID_ARG;
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

sdi12_dev_handle_t sdi12_new_dev(sdi12_bus_handle_t bus, char address)
{
    SDI12_CHECK(bus, "Invalid bus", err);
    SDI12_CHECK(((address >= '0' && address <= '9') || (address >= 'a' && address <= 'z') || (address >= 'A' && address <= 'Z') || address == '?'),
        "Invalidad sensor address", err);

    sdi12_dev_handle_t dev = calloc(1, sizeof(sdi12_dev_t));
    SDI12_CHECK(bus, "Can't allocate SDI12 device", err);

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

    return dev;

err_cmd:
    sdi12_del_dev(dev);
err:
    return NULL;
}
