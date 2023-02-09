# ESP-IDF SDI-12

SDI-12 bus implementation for ESP-IDF framework. This component use RMT to code and decode SDI-12 frames.

Master branch is compatible with esp-idf v5.0. If you need esp-idf v4.x compatible version, see v4 branch.

## BUS API

Bus api provide 'raw' communication between master (mcu) and device. There are only 3 simple operations: create bus, send cmd over bus and delete bus.

They are defined as:

```
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
```

On bus creation, bus timing is optional parameter to modify break or postbreak timings. In 1.4 specs this values are 12.2ms for break and 8.333ms for postbreak. However, I have found some sensor/probes that are not adjusted to this values, so I add the ability to change them. If unmodified or 0 are set on this struct, default values are used. Timings can only be adjusted on bus creation and can't be changed during operation. Take in mind that, if bus is shared among devices, timing values configured are shared too.

Check example folder.

## DEVICE API

There is higher API to communicate with devices. It provides all 1.4 specs operations.

**TO DO**: *add docs to device API. Check sdi12_dev.h meanwhile.*
