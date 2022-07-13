#pragma once

#include "driver/rmt.h"
#include "driver/gpio.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct sdi12_bus sdi12_bus_t;

    typedef enum 
    {
        SDI12_D_READ,
        SDI12_R_READ,
    }sdi12_read_type_t;

    typedef struct
    {
        uint8_t gpio_num;
        rmt_channel_t rmt_tx_channel;
        rmt_channel_t rmt_rx_channel;
        uint8_t response_buffer_length;
    } sdi12_bus_config_t;

    struct sdi12_bus
    {   
        /**
         * @brief Send command a!, where a is sensor address
         * 
         * @param bus[in]           bus object
         * @param address[in]       sensor address
         * @param timeout[in]       timeout in milliseconds
         * @return esp_err_t 
         */
        esp_err_t (*acknowledge_active)(sdi12_bus_t *bus, char address, uint32_t timeout);

        /**
         * @brief Send address query command ?!.
         * 
         * @details If more than one sensor is on bus, this command will not be reply.
         * 
         * @param bus[in]           bus object
         * @param address[in]       sensor address
         * @param timeout[in]       timeout in milliseconds
         * @return esp_err_t 
         */
        esp_err_t (*address_query)(sdi12_bus_t *bus, char *address, uint32_t timeout);

        /**
         * @brief Send aA! command
         * @param bus[in]           bus object
         * @param old_address[in]   sensor address has
         * @param timeout[in]       new sensor address
         * @return esp_err_t
         */
        esp_err_t (*change_address)(sdi12_bus_t *bus, char old_address, char new_address, uint32_t timeout);

        /**
         * @brief Use to send raw commands to sensor. It can be usefull to send extended commands.
         * 
         * @details Any check and parser are applied to response. You'll get get raw response in response_buffer.
         *  
         * @param bus[in]                       bus object
         * @param cmd[in]                       command to send without address and !. The will be concated by function
         * @param response_buffer[out]          buffer to store sensor response
         * @param response_buffer_length[in]    maximum length allowed to store in response buffer
         * @param timeout[in]                   timeout in milliseconds
         * @return esp_err_t 
         * 
         */
        esp_err_t (*raw_cmd)(sdi12_bus_t *bus, const char *cmd, char *response_buffer, size_t response_buffer_length, uint32_t timeout);


        /**
         * @brief Send command aI!
         * 
         * @param bus[in]               bus object           
         * @param address[in]           device address       
         * @param out_buffer[out]       buffer to store identification response
         * @param out_buffer_length[in] maximum available size in buffer values
         * @param timeout[in]           timeout in milliseconds              
         * @return esp_err_t 
         * 
         */
        esp_err_t (*read_identification)(sdi12_bus_t *bus, char address, const char *out_buffer, size_t out_buffer_length, uint32_t timeout);

        /**
         * @brief Send command aDx!
         * 
         * @param bus[in]               bus object
         * @param address[in]           device address       
         * @param d_index[in]           measuremente index(0..9)
         * @param out_buffer[out]       buffer to store read values
         * @param out_buffer_length[in] maximum available size in buffer values
         * @param timeout[in]           timeout in milliseconds              
         * @return esp_err_t 
         */
        esp_err_t (*read_values)(sdi12_bus_t *bus, char address, uint8_t d_index, const char *out_buffer, size_t out_buffer_length, uint32_t timeout);

        /**
         * @brief Send command aRx!
         * 
         * @param bus[in]               bus object
         * @param address[in]           device address       
         * @param r_index[in]           measuremente index(0..9)
         * @param out_buffer[out]       buffer to store read values
         * @param out_buffer_length[in] maximum available size in buffer values
         * @param timeout[in]           timeout in milliseconds              
         * @return esp_err_t 
         */
        esp_err_t (*read_continuos_values)(sdi12_bus_t *bus, char address, uint8_t r_index, const char *out_buffer, size_t out_buffer_length, uint32_t timeout);


        /**
         * @brief Send command aM!
         * 
         * @param bus[in]               bus object           
         * @param address[in]           device address       
         * @param measurements[out]     numbers of measurement sensor will provide
         * @param timeout[in]           timeout in milliseconds              
         * @return esp_err_t 
         */
        esp_err_t (*start_measurement)(sdi12_bus_t *bus, char address, uint8_t *measurements, uint32_t timeout);

        /**
         * @brief Send aMx! command
         * 
         * @param bus[in]               bus object
         * @param address[in]           sensor address
         * @param m_index[in]           measurement index
         * @param measurements[out]     number of values, sensor will provide
         * @param timeout[in]           timeout in milliseconds       
         * @return esp_err_t 
         */
        esp_err_t (*start_additional_measurement)(sdi12_bus_t *bus, char address, uint8_t m_index, uint8_t *measurements, uint32_t timeout);

        /**
         * @brief Send aC! command
         * 
         * @param bus[in]               bus object
         * @param address[in]           sensor address
         * @param ready_seconds[out]    seconds you must wait until sensor can provide values
         * @param measurements[out]     number of values, sensor will provide
         * @param timeout[in]           timeout in milliseconds       
         * @return esp_err_t 
         */
        esp_err_t (*start_concurrent_measurement)(sdi12_bus_t *bus, char address, uint8_t *ready_seconds, uint8_t *measurements, uint32_t timeout);

        /**
         * @brief Send aMx! command
         * 
         * @param bus[in]               bus object
         * @param address[in]           sensor address
         * @param c_index[in]           measurement index
         * @param ready_seconds[out]    seconds you must wait until sensor can provide values
         * @param measurements[out]     number of values, sensor will provide
         * @param timeout[in]           timeout in milliseconds       
         * @return esp_err_t 
         */
        esp_err_t (*start_additional_concurrent_measurement)(sdi12_bus_t *bus, char address, uint8_t c_index, uint8_t *ready_seconds, uint8_t *measurements, uint32_t timeout);


        /**
         * @brief Send aV! command
         * @param bus[in]               bus object
         * @param address[in]           sensor address
         * @param timeout[in]           timeout in milliseconds       
         * @return esp_err_t 
         */
        esp_err_t (*start_verification)(sdi12_bus_t *bus, char address, uint32_t timeout);

        /**
         * @brief Wait for a service request, when needed
         * 
         * @param bus[in]           bus object
         * @param address[in]       device address
         * @param timeout[in]       timeout in milliseconds
         * @return esp_err_t    
         */
        esp_err_t (*wait_service_request)(sdi12_bus_t *bus, char address, uint32_t timeout);

        /**
         * @brief Deallocate and free resources taken by the bus
         * 
         * @param bus[in]       bus object
         * 
         */
        esp_err_t (*deinit)(sdi12_bus_t *bus);
    };

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