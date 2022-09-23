#pragma once

#include "driver/rmt.h"
#include "driver/gpio.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif
    #define SDI12_DEFAULT_RESPONSE_TIMEOUT (1000) // Milliseconds

    typedef struct sdi12_bus sdi12_bus_t;

    typedef struct
    {
        uint16_t break_us;
        uint16_t post_break_marking_us;
    } sdi12_bus_timing_t;

    typedef struct
    {
        uint8_t gpio_num;
        rmt_channel_t rmt_tx_channel;
        rmt_channel_t rmt_rx_channel;
        sdi12_bus_timing_t bus_timing;
    } sdi12_bus_config_t;

    /**
     * @brief Send command over the bus and waits ONLY for first response line (first <LF><CR> found).
     * 
     * @details AM!, AMx!, aMC!, aMCx!, aV! commands require a service request.
     *          When service request command is issued, timeout is used for wait 'atttn', 'atttnn' or 'atttnnn' response line. However, send cmd proccess can take more time due to
     *          'ttt' seconds. Function automatically calculates elapse time and waits for it. If there is no response, ESP_TIMEOUT is returned. On the other side,
     *          response (device address) is checked with cmd address. If comparison fails, ESP_FAIL is returned. 
     * 
     * 
     * 
     * @param bus[in]                   bus object
     * @param cmd[in]                   cmd to send
     * @param out_buffer[out]           buffer to save response
     * @param out_buffer_legnth[out]    
     * @param check_crc[in]             
     * @param timeout[in]               time to wait for response
     * 
     * @return esp_err_t
     *      ESP_OK
     *      ESP_ERR_TIMEOUT
     *      ESP_ERR_FAIL  
     *      ESP_ERR_INVALID_ARG
     *      ESP_ERR_INVALID_SIZE
     */
    esp_err_t sdi12_bus_send_cmd(sdi12_bus_t *bus, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout);


    /**
     * @brief Deallocate and free bus resources
     *
     * @param bus[in]       bus object to deallocate
     * @return esp_err_t
     */

    esp_err_t sdi12_bus_deinit(sdi12_bus_t *bus);

    /**
     * @brief Initialize SDI12 bus object
     *
     * @param config            See sdi12_bus_config_t
     * @return sdi12_bus_t*     NULL if error. SDI12 object if success.
     */
    sdi12_bus_t *sdi12_bus_init(sdi12_bus_config_t *config);

#ifdef __cplusplus
}
#endif