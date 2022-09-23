#pragma once

#include "sdi12_bus.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif
    typedef enum
    {
        SDI12_VERSION_UNKNOWN = 0,
        SDI12_VERSION_1_3 = 13,
        SDI12_VERSION_1_4 = 14
    } sdi12_version_t;

    typedef struct sdi12_dev sdi12_dev_t;

    sdi12_version_t sdi12_dev_get_sdi_version(sdi12_dev_t *dev);
    char * sdi12_dev_get_vendor_id(sdi12_dev_t *dev);
    char * sdi12_dev_get_model(sdi12_dev_t *dev);
    char * sdi12_dev_get_model_version(sdi12_dev_t *dev);
    char * sdi12_dev_get_optional_info(sdi12_dev_t *dev);

    esp_err_t sdi12_dev_ack_active(sdi12_dev_t *dev);
    esp_err_t sdi12_dev_change_address(sdi12_dev_t *dev, char new_address);
    esp_err_t sdi12_dev_read_identification(sdi12_dev_t *dev, char *out_buffer, size_t out_buffer_length);
    esp_err_t sdi12_dev_address_query(sdi12_dev_t *dev, char *address);
    esp_err_t sdi12_dev_start_measurement(sdi12_dev_t *dev, uint8_t m_index, bool crc, uint8_t *n_params);
    esp_err_t sdi12_dev_read_data(sdi12_dev_t *dev, uint8_t d_index, bool crc, char *out_buffer, size_t out_buffer_length);
    esp_err_t sdi12_dev_start_verification(sdi12_dev_t *dev, uint8_t *n_params);
    esp_err_t sdi12_dev_start_concurrent_measurement(sdi12_dev_t *dev, uint8_t c_index, bool crc, uint8_t *n_params);
    esp_err_t sdi12_dev_read_continuos_measurement(sdi12_dev_t *dev, uint8_t r_index, bool crc, char *out_buffer, size_t out_buffer_length);

    // TODO implement
    // esp_err_t sdi12_dev_start_high_volume_ascii_measurement(sdi12_dev_t *dev, uint8_t measures_index, uint8_t *n_params);
    // esp_err_t sdi12_dev_start_high_volume_bin_measurement(sdi12_dev_t *dev, uint8_t measures_index, uint8_t *n_params);

    esp_err_t sdi12_dev_read_identify_cmd(sdi12_dev_t *dev, const char *cmd, uint8_t *n_params);

    esp_err_t sdi12_dev_extended_cmd(sdi12_dev_t *dev, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout);

    void sdi12_dev_deinit(sdi12_dev_t *dev);
    sdi12_dev_t *sdi12_dev_init(sdi12_bus_t *bus, char address);

#ifdef __cplusplus
}
#endif