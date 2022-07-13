#include "sdi12.h"

#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "esp_idf_version.h"
#include "esp_pm.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

typedef struct
{
    uint8_t gpio_num;
    rmt_channel_t rmt_tx_channel;
    rmt_channel_t rmt_rx_channel;
    rmt_mode_t rmt_mode;
    sdi12_bus_t parent;
    uint16_t response_buffer_length;
    char *response_buffer;
    xSemaphoreHandle api_mutex;
    // With 4.0 and 4.1 versions, RMT can't be configured to use REF_TICK as source clock. This feature is added in 4.2+ version, so we need lock APB during RMT operation if DFS is enabled
#if CONFIG_PM_ENABLE && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0))
    esp_pm_lock_handle_t pm_lock;
#endif

} p_sdi12_bus_t;

#define DEFAULT_RESPONSE_TIMEOUT (1000) // Milliseconds

/**
 * Largest response, excluding response to extended commands, are receive from aDx! or aRx! commands
 * From specs 1.4, maximum number of characters returned in <values> field is limited to 75 bytes.
 * To this 75, we must add possible CRC(3 bytes), start address(1 bytes), <CR><LF> end and '\0' C string terminator.
 * Total is 82.
 */
#define RESPONSE_BUFFER_DEFAULT_SIZE (85)

#define SDI12_BREAK_US (12200)
// #define SDI12_POST_BREAK_MARKING_US (8333)
#define SDI12_POST_BREAK_MARKING_US (9000)
#define SDI12_BIT_WIDTH_US (833)

#define SDI12_MARKING (0)
#define SDI12_SPACING (1)

#define SDI12_BUS_LOCK(b) \
    if (b->api_mutex)     \
    xSemaphoreTake(b->api_mutex, portMAX_DELAY)

#define SDI12_BUS_UNLOCK(b) \
    if (b->api_mutex)       \
    xSemaphoreGive(b->api_mutex)

#if CONFIG_PM_ENABLE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
#define SDI12_APB_LOCK(b) esp_pm_lock_acquire(b->pm_lock)
#else
#define SDI12_APB_LOCK(b)
#endif

#if CONFIG_PM_ENABLE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
#define SDI12_APB_UNLOCK(b) esp_pm_lock_release(b->pm_lock)
#else
#define SDI12_APB_UNLOCK(b)
#endif

#define SDI12_CHECK_ADDRESS(bus, address) (bus->response_buffer[0] == address ? ESP_OK : ESP_FAIL)

#define SDI12_CHECK(a, str, goto_tag, ...)                                        \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define IS_VALID_PIN(p) ((p >= 0 && p < 6) || (p > 11 && p < 34))

static const char *TAG = "SDI12";

/******************************************************************************************************
 * *************************            INTERNAL UTILS      *******************************************
 * ****************************************************************************************************/

/**
 * @brief Configure RMT channel as Transmissor
 *
 * @param p_bus         bus object
 * @return esp_err_t
 *      - ESP_FAIL RMT config install error
 *      - ESP_OK  configuration and installation OK
 */
static esp_err_t config_rmt_as_tx(p_sdi12_bus_t *p_bus)
{
    if (p_bus->rmt_mode == RMT_MODE_TX)
    {
        return ESP_OK;
    }
    else if (p_bus->rmt_mode == RMT_MODE_RX)
    {
        rmt_driver_uninstall(p_bus->rmt_rx_channel);
    }

    rmt_config_t rmt_tx = RMT_DEFAULT_CONFIG_TX(p_bus->gpio_num, p_bus->rmt_tx_channel);

// Configure REF_TICKS as clk source
#if CONFIG_PM_ENABLE
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 3, 0)
    rmt_tx.flags = RMT_CHANNEL_FLAGS_ALWAYS_ON;
#elif ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(4, 3, 0)
    rmt_tx.flags = RMT_CHANNEL_FLAGS_AWARE_DFS;
#endif

    // REF_TICKS runs at 1MHz. RMT_DEFAULT_CONFIG_XX configure clk_div to 80, so we must ovewrite clk_div to 1
    rmt_tx.clk_div = 1;
#endif

    SDI12_CHECK(rmt_config(&rmt_tx) == ESP_OK, "Error on RMT TX config", err);
    SDI12_CHECK(rmt_driver_install(p_bus->rmt_tx_channel, 0, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED) == ESP_OK, "RMT TX install error", err);
    p_bus->rmt_mode = RMT_MODE_TX;

    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Configure RMT channel as Receptor
 *
 * @param p_bus         bus object
 * @return esp_err_t
 *      - ESP_FAIL RMT config install error
 *      - ESP_OK  configuration and installation OK
 */
static esp_err_t config_rmt_as_rx(p_sdi12_bus_t *p_bus)
{
    if (p_bus->rmt_mode == RMT_MODE_RX)
    {
        return ESP_OK;
    }
    else if (p_bus->rmt_mode == RMT_MODE_TX)
    {
        rmt_driver_uninstall(p_bus->rmt_tx_channel);
    }

    rmt_config_t rmt_rx = RMT_DEFAULT_CONFIG_RX(p_bus->gpio_num, p_bus->rmt_rx_channel);
    rmt_rx.mem_block_num = 2;

#if CONFIG_PM_ENABLE
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 3, 0)
    rmt_rx.flags = RMT_CHANNEL_FLAGS_ALWAYS_ON;
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
    rmt_rx.flags = RMT_CHANNEL_FLAGS_AWARE_DFS;
#endif

    rmt_rx.clk_div = 1;
#endif

    SDI12_CHECK(rmt_config(&rmt_rx) == ESP_OK, "Error on RMT RX config", err);
    SDI12_CHECK(rmt_driver_install(p_bus->rmt_rx_channel, 1024, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED) == ESP_OK, "RMT RX install error", err);
    p_bus->rmt_mode = RMT_MODE_RX;

    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Parse RMT items into char buffer. Stop when SDI12 response end (\r\n) is found
 *
 * @param p_bus         bus object
 * @param items         received rmt items
 * @param items_length  received rmt items length
 * @return esp_err_t
 *      - ESP_ERR_INVALID_ARG bus or items are NULL or items_length <= 0
 *      - ESP_ERR_NOT_FOUND SDI12 end isn't found
 *      - ESP_OK SDI12 end is found and parse ok
 */
static esp_err_t parse_response(p_sdi12_bus_t *p_bus, rmt_item32_t *items, size_t items_length)
{
    SDI12_CHECK(p_bus, "", err_arg);
    SDI12_CHECK(items && items_length > 0, "", err_arg);

    memset(p_bus->response_buffer, '\0', p_bus->response_buffer_length);

    size_t char_index = 0;
    size_t item_index = 0;
    bool rmt_index1 = false; // False if level0, duration0 needed. True when level1, duration1
    uint8_t bit_counter = 0;
    uint8_t level;
    uint8_t number_of_bits;
    char c = 0;
    bool parity = false;

    while (item_index < items_length)
    {
        if (!rmt_index1)
        {
            level = items[item_index].level0;
            // (items[index].duration0 + SDI12_BIT_WIDTH_US / 2) / SDI12_BIT_WIDTH_US -> Solve integer division round.
            number_of_bits = (items[item_index].duration0 + SDI12_BIT_WIDTH_US / 2) / SDI12_BIT_WIDTH_US;
        }
        else
        {
            level = items[item_index].level1;
            number_of_bits = (items[item_index].duration1 + SDI12_BIT_WIDTH_US / 2) / SDI12_BIT_WIDTH_US;
            ++item_index;
        }

        rmt_index1 = !rmt_index1;

        ESP_LOGD(TAG, "Level: %d, Number of Bits: %d", level, number_of_bits);

        while (number_of_bits > 0 && number_of_bits < 10)
        {

            switch (bit_counter)
            {

            // start bit
            case 0:
                // We need to found start bit.
                if (level == 1)
                {
                    ++bit_counter;
                    parity = false;
                    c = 0;
                }

                break;

            // parity bit
            case 8:
                if (parity != level)
                {
                    ESP_LOGE(TAG, "Reception parity error");
                    return ESP_FAIL;
                }

                if (char_index < p_bus->response_buffer_length)
                {
                    p_bus->response_buffer[char_index] = c;

                    if (p_bus->response_buffer[char_index] == '\n' && p_bus->response_buffer[char_index - 1] == '\r')
                    {
                        p_bus->response_buffer[char_index - 1] = '\0'; // Delete \r\n from response buffer
                        ESP_LOGD(TAG, "Response: %s", p_bus->response_buffer);
                        return ESP_OK;
                    }

                    ++char_index;
                }

                ++bit_counter;
                break;

            // stop bit
            case 9:
                if (level != 0)
                {
                    ESP_LOGE(TAG, "Reception Stop bit error");
                    return ESP_FAIL;
                }

                bit_counter = 0;
                break;

            // data bits. Remember inverse logic
            default:
                if (level == 0)
                {
                    c |= (1 << (bit_counter - 1));
                }
                else
                {
                    parity = !parity;
                }

                ++bit_counter;
                break;
            }

            --number_of_bits;
        }
    }

    return ESP_ERR_NOT_FOUND;

err_arg:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t read_response_line(p_sdi12_bus_t *p_bus, uint32_t timeout)
{
    SDI12_CHECK(p_bus, "BUS is NULL", err);

    esp_err_t ret;

    ret = config_rmt_as_rx(p_bus);

    if (ret != ESP_OK)
    {
        return ret;
    }

    rmt_item32_t *items;
    size_t length = 0;
    uint32_t aux_timeout = timeout != 0 ? timeout : DEFAULT_RESPONSE_TIMEOUT;
    RingbufHandle_t rb = NULL;

    ret = rmt_get_ringbuf_handle(p_bus->rmt_rx_channel, &rb);
    ret = rmt_rx_start(p_bus->rmt_rx_channel, 1);
    if (ret == ESP_OK && rb)
    {
        do
        {
            items = (rmt_item32_t *)xRingbufferReceive(rb, &length, pdMS_TO_TICKS(aux_timeout));

            if (items)
            {

                // for (size_t i = 0; i < length; i++)
                // {
                //     printf("Level: %d | Duration: %d \n", items[i].level0, items[i].duration0);
                //     printf("Level: %d | Duration: %d \n", items[i].level1, items[i].duration1);
                // }
                // ESP_LOGI(TAG, "Length: %d", length);
                ret = parse_response(p_bus, items, length);
            }
            else
            {
                ret = ESP_ERR_TIMEOUT;
            }

        } while (ret == ESP_OK && strlen(p_bus->response_buffer) == 0); // Skip empty lines. Pure "\r\n" lines.
    }

    return ret;

err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t send_cmd(p_sdi12_bus_t *p_bus, const char *cmd)
{
    if (!cmd)
        return ESP_ERR_INVALID_ARG;

    uint8_t cursor = 0;
    // Initial Break & marking + chars. Every char need 10 bits transfers so it need 5 rmt_items32
    uint8_t rmt_items_length = 1 + strlen(cmd) * 5;
    rmt_item32_t rmt_items[rmt_items_length];

    // Break + marking
    rmt_items[cursor].level0 = 1;
    rmt_items[cursor].duration0 = SDI12_BREAK_US;
    rmt_items[cursor].level1 = SDI12_MARKING;
    rmt_items[cursor].duration1 = SDI12_POST_BREAK_MARKING_US;
    ++cursor;
    vTaskDelay(pdMS_TO_TICKS(80));
    // Data

    uint8_t index = 0;

    while (index < strlen(cmd))
    {
        char data_byte = cmd[index];
        bool parity = false;

        // Char frame (10 bits): start + 7 data bits + parity + stop bit. Inverse mode.

        // Start bit
        rmt_items[cursor].level0 = SDI12_SPACING;
        rmt_items[cursor].duration0 = SDI12_BIT_WIDTH_US;

        // Char bits
        uint8_t mask = 0x01;
        uint8_t bit_value = 0;

        for (uint8_t i = 0; i < 7; ++i)
        {
            bit_value = data_byte & mask;
            uint8_t level_to_write;

            if (bit_value == mask) // bit_value == 1; Inverse -> 0 to write
            {
                level_to_write = SDI12_MARKING;
            }
            else // bit_value == 0; Inverse -> 1 to write
            {
                level_to_write = SDI12_SPACING;
                parity = !parity;
            }

            if (i % 2 == 0)
            {
                rmt_items[cursor].level1 = level_to_write;
                rmt_items[cursor].duration1 = SDI12_BIT_WIDTH_US;
                ++cursor;
            }
            else
            {
                rmt_items[cursor].level0 = level_to_write;
                rmt_items[cursor].duration0 = SDI12_BIT_WIDTH_US;
            }

            data_byte >>= 1;
        }

        // Here we are written 4 complete rmt items, so we know parity will be on next 0 slot and stop on 1 slot
        rmt_items[cursor].level0 = parity;
        rmt_items[cursor].duration0 = SDI12_BIT_WIDTH_US;
        rmt_items[cursor].level1 = SDI12_MARKING;
        rmt_items[cursor].duration1 = SDI12_BIT_WIDTH_US;
        ++cursor;
        ++index;
    }

    esp_err_t ret = config_rmt_as_tx(p_bus);

    if (ret == ESP_OK)
    {
        ret = rmt_write_items(p_bus->rmt_tx_channel, rmt_items, rmt_items_length, 1);
    }

    return ret;
}

static esp_err_t send_cmd_wait_response_line(p_sdi12_bus_t *p_bus, const char *cmd, uint32_t response_timeout)
{
    esp_err_t ret = send_cmd(p_bus, cmd);

    if (ret == ESP_OK)
    {
        ret = read_response_line(p_bus, response_timeout);
    }

    return ret;
}

/******************************************************************************************************
 * *************************            FUNCTION IMPLEMENTATIONS      **********************************
 * ****************************************************************************************************/

static esp_err_t wait_service_request(sdi12_bus_t *bus, char address, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);

    esp_err_t ret = read_response_line(p_bus, timeout);

    // Read Service Request
    if (ret == ESP_OK)
    {
        ret = SDI12_CHECK_ADDRESS(p_bus, address);
    }

    // config_rmt_as_tx(p_bus);
    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t acknowledge_active(sdi12_bus_t *bus, char address, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);

    char cmd[3] = "";
    sprintf(cmd, "%c!", address);

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);
    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    // Response must be a\r\n, where a is address
    if (ret == ESP_OK)
    {
        ret = SDI12_CHECK_ADDRESS(p_bus, address);
    }

    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t address_query(sdi12_bus_t *bus, char *address, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);

    char cmd[3] = "?!";

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);

    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    // Response must be a\r\n, where a is address
    if (ret == ESP_OK)
    {
        *address = p_bus->response_buffer[0];
    }

    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t change_address(sdi12_bus_t *bus, char old_address, char new_address, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);

    char cmd[5] = "";
    sprintf(cmd, "%cA%c!", old_address, new_address);

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);

    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    if (ret == ESP_OK)
    {
        ret = SDI12_CHECK_ADDRESS(p_bus, new_address);
    }
    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t read_identification(sdi12_bus_t *bus, char address, const char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);

    char cmd[4] = "";
    sprintf(cmd, "%cI!", address);

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);

    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    /**
     * Response must be allccccccccmmmmmmvvvxxx...xxx\r\n
     * a[1]: sensor address
     * ll[2]: SDI12 version
     * cccccccc[8]: vendor identification
     * mmmmmm[6]: sensor model number
     * vvv[3]: sensor version
     * xxx...xxx[0..13]: (optional) serial number or other specific sensor information
     */
    if (ret == ESP_OK)
    {
        ret = SDI12_CHECK_ADDRESS(p_bus, address);
        if (ret == ESP_OK && out_buffer)
        {
            snprintf((char *)out_buffer, out_buffer_length, p_bus->response_buffer + 1);
        }
    }

    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t read_any_values(sdi12_bus_t *bus, char address, char read_type, uint8_t index, const char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);
    SDI12_CHECK(index <= 9, "d_index out of range", err);

    char cmd[5] = "";
    sprintf(cmd, "%c%c%d!", address, read_type, index);

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);

    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    // Response must be a<values>\r\n, where a is device address. We will check address and copy values into out_buffer without \r\n
    if (ret == ESP_OK)
    {
        ret = SDI12_CHECK_ADDRESS(p_bus, address);
        if (ret == ESP_OK)
        {
            if (out_buffer)
            {
                // Skip first character (address) and copy only values.
                snprintf((char *)out_buffer, out_buffer_length, p_bus->response_buffer + 1);
            }
        }
    }

    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t read_values(sdi12_bus_t *bus, char address, uint8_t d_index, const char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    return read_any_values(bus, address, 'D', d_index, out_buffer, out_buffer_length, timeout);
}

static esp_err_t read_continuos_values(sdi12_bus_t *bus, char address, uint8_t r_index, const char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    return read_any_values(bus, address, 'R', r_index, out_buffer, out_buffer_length, timeout);
}

static esp_err_t raw_cmd(sdi12_bus_t *bus, const char *cmd, char *response_buffer, size_t response_buffer_length, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "BUS is NULL", err);
    SDI12_CHECK(cmd, "NULL cmd", err);

    // size_t full_cmd_size = 3 + strlen(command); // 3(address + '!' + '\0' string end)
    // char *cmd = calloc(full_cmd_size, sizeof(char));
    // *cmd = '\0';
    // sprintf(cmd, "%c%.*s!", address, strlen(command), command);

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);

    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    if (ret == ESP_OK)
    {
        if (strlcpy(response_buffer, p_bus->response_buffer, response_buffer_length) >= response_buffer_length)
        {
            response_buffer[response_buffer_length - 1] = '\0';
        }
    }

    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;

err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t start_any_measurement(sdi12_bus_t *bus, char address, char measurement_type, uint8_t index, uint8_t *ready_seconds, uint8_t *measurements, uint32_t timeout)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "Invalid or NULL bus", err);

    char cmd[5] = "";

    if (index == 0)
    {
        sprintf(cmd, "%c%c!", address, measurement_type);
    }
    else if (index <= 9)
    {
        sprintf(cmd, "%c%c%d!", address, measurement_type, index);
    }

    SDI12_BUS_LOCK(p_bus);
    SDI12_APB_LOCK(p_bus);

    esp_err_t ret = send_cmd_wait_response_line(p_bus, cmd, timeout);

    /** Response should be atttn<CR><LF>.
     *   a: sensor address
     *   ttt: seconds until measurement(s) will be ready
     *   n: number of measurement values
     */
    if (ret == ESP_OK)
    {
        ret = SDI12_CHECK_ADDRESS(p_bus, address);

        if (ret == ESP_OK)
        {
            uint16_t seconds = 0;
            uint8_t factor = 100;

            for (uint8_t i = 1; i < 4; i++)
            {
                seconds += (p_bus->response_buffer[i] - '0') * factor;
                factor /= 10;
            }

            if (ready_seconds)
            {
                *ready_seconds = seconds;
            }

            if (measurements)
            {
                *measurements = p_bus->response_buffer[4] - '0';
            }

            // Wait for Service Request if necessary. Only necessary if ready seconds is equal or greather than 1
            if (seconds > 0 && measurement_type != 'C')
            {
                wait_service_request(bus, address, seconds * 1000);
            }
        }
    }

    config_rmt_as_tx(p_bus);
    SDI12_APB_UNLOCK(p_bus);
    SDI12_BUS_UNLOCK(p_bus);

    return ret;

err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t start_measurement(sdi12_bus_t *bus, char address, uint8_t *measurements, uint32_t timeout)
{
    return start_any_measurement(bus, address, 'M', 0, NULL, measurements, timeout);
}

static esp_err_t start_additional_measurement(sdi12_bus_t *bus, char address, uint8_t m_index, uint8_t *measurements, uint32_t timeout)
{
    SDI12_CHECK(m_index <= 9, "m_index out of range", err);

    return start_any_measurement(bus, address, 'M', m_index, NULL, measurements, timeout);
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t start_concurrent_measurement(sdi12_bus_t *bus, char address, uint8_t *ready_seconds, uint8_t *measurements, uint32_t timeout)
{
    return start_any_measurement(bus, address, 'C', 0, ready_seconds, measurements, timeout);
}

static esp_err_t start_additional_concurrent_measurement(sdi12_bus_t *bus, char address, uint8_t c_index, uint8_t *ready_seconds, uint8_t *measurements, uint32_t timeout)
{
    SDI12_CHECK(c_index <= 9, "c_index out of range", err);

    return start_any_measurement(bus, address, 'C', c_index, ready_seconds, measurements, timeout);
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t start_verification(sdi12_bus_t *bus, char address, uint32_t timeout)
{
    return start_any_measurement(bus, address, 'V', 0, NULL, NULL, timeout);
}

static esp_err_t deinit(sdi12_bus_t *bus)
{
    p_sdi12_bus_t *p_bus = __containerof(bus, p_sdi12_bus_t, parent);
    SDI12_CHECK(p_bus, "Invalid or NULL bus", err);
    esp_err_t ret = ESP_OK;

    if (p_bus->rmt_mode == RMT_MODE_RX)
    {
        ret = rmt_driver_uninstall(p_bus->rmt_rx_channel);
    }
    else if(p_bus->rmt_mode == RMT_MODE_TX)
    {
        ret = rmt_driver_uninstall(p_bus->rmt_tx_channel);
    }

    if (p_bus->response_buffer)
    {
        free(p_bus->response_buffer);
    }

    free(p_bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

sdi12_bus_t *sdi12_bus_init(sdi12_bus_config_t *config)
{
    SDI12_CHECK(config, "Config is NULL", err);
    SDI12_CHECK(IS_VALID_PIN(config->gpio_num), "Invalid GPIO pin", err);

    p_sdi12_bus_t *p_sdi12_bus = calloc(1, sizeof(p_sdi12_bus_t));
    SDI12_CHECK(p_sdi12_bus, "Can't allocate bus", err);

    p_sdi12_bus->response_buffer_length = config->response_buffer_length > RESPONSE_BUFFER_DEFAULT_SIZE ? config->response_buffer_length : RESPONSE_BUFFER_DEFAULT_SIZE;
    p_sdi12_bus->response_buffer = calloc(p_sdi12_bus->response_buffer_length, sizeof(char));
    SDI12_CHECK(p_sdi12_bus, "Can't allocate response buffer", err_buf);

    p_sdi12_bus->gpio_num = config->gpio_num;
    p_sdi12_bus->rmt_tx_channel = config->rmt_tx_channel;
    p_sdi12_bus->rmt_rx_channel = config->rmt_rx_channel;
    p_sdi12_bus->rmt_mode = RMT_MODE_MAX; // Force firts time installation

    p_sdi12_bus->parent.acknowledge_active = acknowledge_active;
    p_sdi12_bus->parent.address_query = address_query;
    p_sdi12_bus->parent.change_address = change_address;
    p_sdi12_bus->parent.start_additional_measurement = start_additional_measurement;
    p_sdi12_bus->parent.start_measurement = start_measurement;
    p_sdi12_bus->parent.start_additional_concurrent_measurement = start_additional_concurrent_measurement;
    p_sdi12_bus->parent.start_concurrent_measurement = start_concurrent_measurement;
    p_sdi12_bus->parent.start_verification = start_verification;
    p_sdi12_bus->parent.raw_cmd = raw_cmd;
    p_sdi12_bus->parent.read_identification = read_identification;
    p_sdi12_bus->parent.read_values = read_values;
    p_sdi12_bus->parent.read_continuos_values = read_continuos_values;
    p_sdi12_bus->parent.wait_service_request = wait_service_request;
    p_sdi12_bus->parent.deinit = deinit;

    p_sdi12_bus->api_mutex = xSemaphoreCreateMutex();
    SDI12_CHECK(p_sdi12_bus->api_mutex, "Mutex allocation error", err_mutex);

#if CONFIG_PM_ENABLE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 1, "SDI12_PM_LOCK", &p_sdi12_bus->pm_lock);
#endif

    return &p_sdi12_bus->parent;

err_mutex:
    free(p_sdi12_bus->response_buffer);
err_buf:
    free(p_sdi12_bus);
err:
    return NULL;
}
