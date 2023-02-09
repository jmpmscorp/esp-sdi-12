#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define SDI12_DEFAULT_RESPONSE_TIMEOUT (1000) // Milliseconds

    typedef struct sdi12_bus *sdi12_bus_handle_t;

    typedef struct
    {
        uint16_t break_us;
        uint16_t post_break_marking_us;
    } sdi12_bus_timing_t;

    typedef struct
    {
        uint8_t gpio_num;
        sdi12_bus_timing_t bus_timing;
    } sdi12_bus_config_t;


    /**
     * @brief Send command over the bus and waits ONLY for first response line (first <LF><CR> found).
     *
     * @details AM!, AMx!, aMC!, aMCx!, aV! commands require a service request.
     * When service request command is issued, timeout is used for wait 'atttn', 'atttnn' or 'atttnnn' response line. However, send cmd proccess can
     * take more time due to 'ttt' seconds. Function automatically calculates elapsed time and waits for it. If there is no response, ESP_TIMEOUT is returned.
     * On the other side, response (device address) is checked with cmd address. If comparison fails, ESP_FAIL is returned.
     *
     *
     *
     * @param[in] bus                   bus object
     * @param[in] cmd                   cmd to send
     * @param[in] check_crc             true if crc check is needed. false otherwise
     * @param[out] out_buffer           buffer to save response
     * @param[out] out_buffer_length    response buffer length
     * @param[in] timeout               time to wait for response
     *
     * @return esp_err_t
     *      ESP_OK
     *      ESP_ERR_TIMEOUT
     *      ESP_ERR_FAIL
     *      ESP_ERR_INVALID_ARG
     *      ESP_ERR_INVALID_SIZE
     */
    esp_err_t sdi12_bus_send_cmd(sdi12_bus_handle_t bus, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout);

    /**
     * @brief Deallocate and free bus resources
     *
     * @param[in] bus       bus object to deallocate
     * @return esp_err_t
     */

    esp_err_t sdi12_del_bus(sdi12_bus_handle_t bus);

    /**
     * @brief Initialize SDI12 bus object
     *
     * @param[in] sdi12_bus_config       See sdi12_bus_config_t
     * @param[out] sdi12_bus_out         Created object
     * @return esp_err_t
     *      ESP_OK
     *      ESP_ERR_INVALID_ARG
     */
    esp_err_t sdi12_new_bus(sdi12_bus_config_t *sdi12_bus_config, sdi12_bus_handle_t *sdi12_bus_out);

#ifdef __cplusplus
}
#endif