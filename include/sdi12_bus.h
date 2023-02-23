#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * By 1.4 specs, commonly response time is:
 *  - 380 msec most commands
 *  - 780 msec for a D command after concurrent measurement
 *  - 810 msec for a D command after concurrent measurement with CRC
 *
 *  Maximum response time is 10.1 seconds to a DB command.
 */
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
     * When service request command is issued, timeout param is used for wait 'atttn', 'atttnn' or 'atttnnn' response line.
     * Function automatically calculates elapsed time and waits for it.
     *
     *
     * @param[in] bus                   bus object
     * @param[in] cmd                   cmd to send
     * @param[in] check_crc             true if crc check is needed. false otherwise
     * @param[out] out_buffer           buffer to save response
     * @param[out] out_buffer_length    response buffer length
     * @param[in] timeout               time to wait for response
     *
     * @return
     *      - ESP_OK on success
     *      - ESP_ERR_TIMEOUT cmd timeout expires
     *      - ESP_ERR_INVALID_ARG any invalid argument
     *      - ESP_ERR_INVALID_SIZE when out buffer length isn't enough
     *      - ESP_ERR_INVALID_RESPONSE when response first char is different than cmd first char
     *      - ESP_ERR_NOT_FINISHED when service request required but no address char is received
     *      - ESP_FAIL all other cases
     */
    esp_err_t sdi12_bus_send_cmd(sdi12_bus_handle_t bus, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout);

    /**
     * @brief Deallocate and free bus resources
     *
     * @param[in] bus       bus object to deallocate
     * @return
     *      - ESP_OK always
     */

    esp_err_t sdi12_del_bus(sdi12_bus_handle_t bus);

    /**
     * @brief Initialize SDI12 bus object
     *
     * @param[in] sdi12_bus_config       See sdi12_bus_config_t
     * @param[out] sdi12_bus_out         Created object
     * @return
     *      - ESP_OK when bus can be allocated
     *      - ESP_ERR_INVALID_ARG when sdi12_bus_config is NULL
     */
    esp_err_t sdi12_new_bus(sdi12_bus_config_t *bus_config, sdi12_bus_handle_t *sdi12_bus_out);

#ifdef __cplusplus
}
#endif