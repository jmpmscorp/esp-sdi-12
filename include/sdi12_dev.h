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

    typedef struct sdi12_dev *sdi12_dev_handle_t;

    /**
     * @brief Get address stored in object
     *
     * @note No bus interaction
     *
     * @param[in] dev           device object
     * @param[out] out_address  device address
     * @return
     *      - ESP_INVALID_ARG if NULL dev
     *      - ESP_OK on success
     */
    esp_err_t sdi12_dev_get_address(sdi12_dev_handle_t dev, char *out_address);

    /**
     * @brief Get SDI12 version stored in object
     *
     * @note No bus interaction. Returns address passed on device creation.
     *
     * @param[in] dev   device object
     * @param[out] out_version  device sdi12 version compliant
     * @return
     *      - ESP_INVALID_ARG if NULL dev
     *      - ESP_OK on success
     */
    esp_err_t sdi12_dev_get_sdi_version(sdi12_dev_handle_t dev, sdi12_version_t *out_version);

    /**
     * @brief Get address stored in object
     *
     * @note No bus interaction. Returns SDI12 version stored before aI! success command
     *
     * @param[in] dev   device object
     * @param[out] out_vendor_id  device vendor id
     * @return
     *      - ESP_INVALID_ARG if NULL dev
     *      - ESP_OK on success
     */
    esp_err_t sdi12_dev_get_vendor_id(sdi12_dev_handle_t dev, char *out_vendor_id);

    /**
     * @brief Get model stored in object
     *
     * @note No bus interaction. Returns sensor model stored before aI! success command
     *
     * @param[in] dev   device object
     * @param[out] out_model  device model
     *
     * @return
     *      - ESP_INVALID_ARG if NULL dev
     *      - ESP_OK on success
     */
    esp_err_t sdi12_dev_get_model(sdi12_dev_handle_t dev, char *out_model);

    /**
     * @brief Get model version stored in object
     *
     * @note No bus interaction. Returns sensor model version stored before aI! success command
     *
     * @param[in] dev   Device object
     * @param[out] out_model_version  device model version
     *
     * @return 
     *      - ESP_INVALID_ARG if NULL dev
     *      - ESP_OK on success
     */
    esp_err_t sdi12_dev_get_model_version(sdi12_dev_handle_t dev, char *out_model_version);

    /**
     * @brief Get optional field stored in object
     *
     * @note No bus interaction. Returns optional field stored before aI! success command
     *
     * @param[in] dev   Device object
     * @return
     *      - ESP_INVALID_ARG if NULL dev
     *      - ESP_OK on success
     */
    esp_err_t sdi12_dev_get_optional_info(sdi12_dev_handle_t dev, char *out_optional_field);

    /**
     * @brief Send a! command.
     *
     * @param[in] dev       Device object
     * @param[in] timeout   Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_acknowledge_active(sdi12_dev_handle_t dev, uint32_t timeout);

    /**
     * @brief Send aAb! command.
     *
     * @param[in] dev           Device object
     * @param[in] new_address   New device address
     * @param[in] timeout
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev or invalid new address
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_change_address(sdi12_dev_handle_t dev, char new_address, uint32_t timeout);

    /**
     * @brief Send aI! command.
     *
     * @details this command reads and store on memory information of sensor. If no out_buffer is passed, command only stores information. You can read stored
     * information with provided getters.
     *
     * @param[in] dev                   Device object
     * @param[out] out_buffer           Buffer for response
     * @param[out] out_buffer_length    Response buffer length
     * @param[in] timeout               Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_read_identification(sdi12_dev_handle_t dev, char *out_buffer, size_t out_buffer_length, uint32_t timeout);

    /**
     * @brief Send ?! command.
     *
     * @warning This command could be used if ONLY ONE device is connected on bus.
     *
     * @param[in] dev       Device object
     * @param[out] address  Returned sensor address
     * @param[in] timeout   Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev or invalid new address
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_address_query(sdi12_dev_handle_t dev, char *address, uint32_t timeout);

    /**
     * @brief Send aMx! or aMCx! command.
     *
     * @details To send aM! or aMC! command, set m_index 0.
     *
     * @param[in] dev           Device object
     * @param[in] m_index       x on aMx!
     * @param[in] crc           True to send CRC version 'aMCx!'. False to send 'aMx!'
     * @param[out] n_params     Number of values returned on measurements. n on response 'atttn'
     * @param[in] timeout       Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev or m_index > 9
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_start_measurement(sdi12_dev_handle_t dev, uint8_t m_index, bool crc, uint8_t *n_params, uint32_t timeout);

    /**
     * @brief Send aDx! command.
     *
     * @details To send aD!, set d_index 0.
     *
     * @param[in] dev                   Device object
     * @param[in] d_index               x on aDx!
     * @param[in] crc                   True to check CRC. False otherwise.
     * @param[out] out_buffer           Buffer to store response
     * @param[out] out_buffer_length    Response buffer length
     * @param[in] timeout               Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev or d_index > 9
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_read_data(sdi12_dev_handle_t dev, uint8_t d_index, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout);

    /**
     * @brief Send aV! command.
     *
     * @param[in] dev           Device object
     * @param[out] n_params     Number of values returned on measurements. n on response 'atttn'
     * @param[in] timeout       Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_start_verification(sdi12_dev_handle_t dev, uint8_t *n_params, uint32_t timeout);

    /**
     * @brief Send aCx! or aCCx! command.
     *
     * @details To send aC! or aCC! command, set c_index 0.
     *
     * @param[in] dev       Device object
     * @param[in] c_index   x on aCx! or aCCx!
     * @param[in] crc       True to send CRC version 'aCCx!'. False to send 'aCx!'
     * @param[out] n_params Number of values returned on measurements. n on response 'atttn'
     * @param[in] timeout   Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_start_concurrent_measurement(sdi12_dev_handle_t dev, uint8_t c_index, bool crc, uint8_t *n_params, uint32_t timeout);

    /**
     * @brief Send aRx! command.
     *
     * @details To send aR!, set d_index 0.
     *
     * @param[in] dev                   Device object
     * @param[in] r_index               x on aRx!
     * @param[in] crc                   True to check CRC. False otherwise
     * @param[out] out_buffer           Buffer to store response
     * @param[out] out_buffer_length    Response buffer length
     * @param[in] timeout               Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_read_continuos_measurement(sdi12_dev_handle_t dev, uint8_t r_index, bool crc, char *out_buffer, size_t out_buffer_length,
        uint32_t timeout);

    // TODO implement
    // esp_err_t sdi12_dev_start_high_volume_ascii_measurement(sdi12_dev_t *dev, uint8_t measures_index, uint8_t *n_params);
    // esp_err_t sdi12_dev_start_high_volume_bin_measurement(sdi12_dev_t *dev, uint8_t measures_index, uint8_t *n_params);

    /**
     * @brief Send any identity command, aIX!
     *
     * @details Function add I automatically. You only need to provide which command you want to send. i.e. To send aIMC!, you need to pass MC through cmd.
     *
     * @param[in] dev           Device object
     * @param[in] cmd           CMD to send
     * @param[out] n_params     Number of values returned on measurements. n on response 'atttn', nn on 'atttnn' or nnn on 'atttnnn'
     * @param[in] timeout       Time to wait for response
     * @return esp_err_t
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev or cmd length > 3
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_read_identify_cmd(sdi12_dev_handle_t dev, const char *cmd, uint8_t *n_params, uint32_t timeout);

    /**
     * @brief Send any command not provided in this api.
     *
     * @details Take in mind that device address and '!' is append automatically to cmd. i.e. To send aHB!, set cmd as HB
     *
     * @param[in] dev                   Device object
     * @param[in] cmd                   Cmd to send
     * @param[in] crc                   Set to true if response needs to check CRC. False otherwise
     * @param[out] out_buffer           Buffer to store response
     * @param[out] out_buffer_length    Response buffer length
     * @param[in] timeout               Time to wait for response
     * @return esp_err_t
     *      - ESP_OK if no error
     *      - ESP_ERR_TIMEOUT if timeout expires
     *      - ESP_ERR_INVALID_ARG if invalid dev
     *      - ESP_FAIL any other error
     */
    esp_err_t sdi12_dev_extended_cmd(sdi12_dev_handle_t dev, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout);

    /**
     * @brief Free device memory resources
     *
     * @param[in] dev   Device object
     */
    void sdi12_del_dev(sdi12_dev_handle_t dev);

    /**
     * @brief Allocate resources for device.
     *
     * @details 
     *  Before allocate device resources and if address is different from '?', acknowledge active command is sent. If response is OK, device object is
     *  returned. If there is any error or response to acknowledge active is not received, NULL is returned. 
     * 
     *  On other hand, if address passed is '?', instead of acknowledge active, address query command is sent.
     *
     * @param[in] bus           SDI12 bus object
     * @param[in] address       Device address or '?' to query it. Ensure only 1 device is on bus if you use '?'.
     * @param[out] dev_out      Device object or NULL
     * 
     * @return
     *      - ESP_OK on creation success
     *      - ESP_INVALID_ARG if invalid bus or address is not in allowed range
     *      - ESP_ERR_NO_MEM if resources can't be allocated
     *      - ESP_FAIL if response to acknowledge active or address command is invalid
     */
    esp_err_t sdi12_new_dev(sdi12_bus_handle_t bus, char address, sdi12_dev_handle_t *dev_out);

#ifdef __cplusplus
}
#endif