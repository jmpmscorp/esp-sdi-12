#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "esp_idf_version.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sdi12_bus.h"

struct sdi12_bus
{
    uint8_t gpio_num;
    rmt_channel_t rmt_tx_channel;
    rmt_channel_t rmt_rx_channel;
    rmt_mode_t rmt_mode;
    sdi12_bus_timing_t timing;
    xSemaphoreHandle mutex;
    // With 4.0 and 4.1 versions, RMT can't be configured to use REF_TICK as source clock. This feature is added in 4.2+
    // version, so we need lock APB during RMT operation if DFS is enabled
#if CONFIG_PM_ENABLE                                                                                                   \
    && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0))
    esp_pm_lock_handle_t pm_lock;
#endif
};

#define SDI12_BUS_LOCK(b)                                                                                              \
    if (b->mutex)                                                                                                  \
    xSemaphoreTake(b->mutex, portMAX_DELAY)

#define SDI12_BUS_UNLOCK(b)                                                                                            \
    if (b->mutex)                                                                                                  \
    xSemaphoreGive(b->mutex)

#if CONFIG_PM_ENABLE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)                                                \
    && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
#define SDI12_APB_LOCK(b) esp_pm_lock_acquire(b->pm_lock)
#else
#define SDI12_APB_LOCK(b)
#endif

#if CONFIG_PM_ENABLE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)                                                \
    && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
#define SDI12_APB_UNLOCK(b) esp_pm_lock_release(b->pm_lock)
#else
#define SDI12_APB_UNLOCK(b)
#endif

/**
 * Largest response, excluding response to extended commands, are receive from aDx! or aRx! commands
 * From specs 1.4, maximum number of characters returned in <values> field is limited to 75 bytes.
 * To this 75, we must add possible CRC(3 bytes), start address(1 bytes), <CR><LF> end and '\0' C string terminator.
 * Total is 82.
 */
#define RESPONSE_BUFFER_DEFAULT_SIZE (82)

#define SDI12_BREAK_US              (12200)
#define SDI12_POST_BREAK_MARKING_US (8333)
#define SDI12_BIT_WIDTH_US          (833)

#define SDI12_MARKING (0)
#define SDI12_SPACING (1)

#define SDI12_CRC_POLY 0xA001

#define SDI12_CHECK(a, str, goto_tag, ...)                                                                             \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(a))                                                                                                      \
        {                                                                                                              \
            ESP_LOGE(TAG, str, ##__VA_ARGS__);                                                                         \
            goto goto_tag;                                                                                             \
        }                                                                                                              \
    } while (0)

#if CONFIG_IDF_TARGET_ESP32
#define IS_VALID_PIN(p) ((p > 0 && p < 6) || (p > 11 && p < 34))
#elif CONFIG_IDF_TARGET_ESP32S2
#define IS_VALID_PIN(p) ((p > 0 && p < 26) || (p >= 32 && p < 45))
#elif CONFIG_IDF_TARGET_ESP32S3
#define IS_VALID_PIN(p) ((p > 0 && p < 3) || (p > 3 && p <= 21) || (p > 32 && p < 48))
#elif CONFIG_IDF_TARGET_ESP32C3
#define IS_VALID_PIN(p) (!(p >= 12 && p <= 17))
#endif

static const char *TAG = "SDI12 BUS";

/**
 * @brief Configure RMT channel as Transmissor
 *
 * @param bus         bus object
 * @return esp_err_t
 *      - ESP_FAIL RMT config install error
 *      - ESP_OK  configuration and installation OK
 */
static esp_err_t config_rmt_as_tx(sdi12_bus_t *bus)
{
    if (bus->rmt_mode == RMT_MODE_TX)
    {
        return ESP_OK;
    }
    else if (bus->rmt_mode == RMT_MODE_RX)
    {
        rmt_driver_uninstall(bus->rmt_rx_channel);
    }

    rmt_config_t rmt_tx = RMT_DEFAULT_CONFIG_TX(bus->gpio_num, bus->rmt_tx_channel);

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
    SDI12_CHECK(
        rmt_driver_install(bus->rmt_tx_channel, 0, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED)
            == ESP_OK,
        "RMT TX install error", err);
    bus->rmt_mode = RMT_MODE_TX;

    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Configure RMT channel as Receptor
 *
 * @param bus         bus object
 * @return esp_err_t
 *      - ESP_FAIL RMT config install error
 *      - ESP_OK  configuration and installation OK
 */
static esp_err_t config_rmt_as_rx(sdi12_bus_t *bus)
{
    if (bus->rmt_mode == RMT_MODE_RX)
    {
        return ESP_OK;
    }
    else if (bus->rmt_mode == RMT_MODE_TX)
    {
        rmt_driver_uninstall(bus->rmt_tx_channel);
    }

    rmt_config_t rmt_rx = RMT_DEFAULT_CONFIG_RX(bus->gpio_num, bus->rmt_rx_channel);
    rmt_rx.mem_block_num = 2;
    rmt_rx.rx_config.filter_ticks_thresh = 250;

#if CONFIG_PM_ENABLE
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 3, 0)
    rmt_rx.flags = RMT_CHANNEL_FLAGS_ALWAYS_ON | RMT_CHANNEL_FLAGS_INVERT_SIG;
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
    rmt_rx.flags = RMT_CHANNEL_FLAGS_AWARE_DFS | RMT_CHANNEL_FLAGS_INVERT_SIG;
#endif

    rmt_rx.clk_div = 1;
#endif

    SDI12_CHECK(rmt_config(&rmt_rx) == ESP_OK, "Error on RMT RX config", err);
    SDI12_CHECK(
        rmt_driver_install(bus->rmt_rx_channel, 1024, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED)
            == ESP_OK,
        "RMT RX install error", err);
    bus->rmt_mode = RMT_MODE_RX;

    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Parse RMT items into char buffer. Stop when SDI12 response end (\r\n) is found
 *
 * @param bus         bus object
 * @param items         received rmt items
 * @param items_length  received rmt items length
 * @return esp_err_t
 *      - ESP_ERR_INVALID_ARG bus or items are NULL or items_length <= 0
 *      - ESP_ERR_NOT_FOUND SDI12 end isn't found
 *      - ESP_OK SDI12 end is found and parse ok
 */
static esp_err_t parse_response(sdi12_bus_t *bus, rmt_item32_t *items, size_t items_length, char *out_buffer,
    size_t out_buffer_length)
{
    memset(out_buffer, '\0', out_buffer_length);

    size_t char_index = 0;
    size_t item_index = 0;
    bool level0 = 0; // False if level0, duration0 needed. True when level1, duration1
    uint8_t bit_counter = 0;
    uint8_t level;
    uint8_t number_of_bits;
    char c = 0;
    bool parity = false;

    while (item_index < items_length)
    {
        if (!level0)
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

        level0 = !level0;

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

                    if (char_index < out_buffer_length)
                    {
                        out_buffer[char_index] = c;

                        if (out_buffer[char_index] == '\n' && out_buffer[char_index - 1] == '\r')
                        {
                            out_buffer[char_index - 1] = '\0'; // Delete \r\n from response buffer
                            ESP_LOGD(TAG, "In: %s", out_buffer);
                            return ESP_OK;
                        }

                        ++char_index;
                    }
                    else
                    {
                        out_buffer[out_buffer_length - 1] = '\0';
                        ESP_LOGE(TAG, "Out buffer too small");
                        return ESP_ERR_INVALID_SIZE;
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
}

static esp_err_t read_response_line(sdi12_bus_t *bus, char *out_buffer, size_t out_buffer_length,
    uint32_t timeout)
{
    SDI12_CHECK(bus, "BUS is NULL", err);

    esp_err_t ret;

    ret = config_rmt_as_rx(bus);

    if (ret != ESP_OK)
    {
        return ret;
    }

    rmt_item32_t *items;
    size_t items_length = 0;
    uint32_t aux_timeout = timeout != 0 ? timeout : SDI12_DEFAULT_RESPONSE_TIMEOUT;
    RingbufHandle_t rb = NULL;

    ret = rmt_get_ringbuf_handle(bus->rmt_rx_channel, &rb);
    ret = rmt_rx_start(bus->rmt_rx_channel, 1);
    if (ret == ESP_OK && rb)
    {
        do
        {
            items = (rmt_item32_t *)xRingbufferReceive(rb, &items_length, pdMS_TO_TICKS(aux_timeout));

            if (items)
            {
                // for (size_t i = 0; i < length; i++)
                // {
                //     printf("Level: %d | Duration: %d \n", items[i].level0, items[i].duration0);
                //     printf("Level: %d | Duration: %d \n", items[i].level1, items[i].duration1);
                // }
                ret = parse_response(bus, items, items_length, out_buffer, out_buffer_length);
            }
            else
            {
                ret = ESP_ERR_TIMEOUT;
            }

        } while (ret == ESP_OK && strlen(out_buffer) == 0); // Skip empty lines. Pure "\r\n" lines.
    }

    return ret;

err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t write_cmd(sdi12_bus_t *bus, const char *cmd)
{
    if (!cmd)
        return ESP_ERR_INVALID_ARG;

    uint8_t cursor = 0;
    // Initial Break & marking + chars. Every char need 10 bits transfers so it need 5 rmt_items32
    uint8_t rmt_items_length = 1 + strlen(cmd) * 5;
    rmt_item32_t rmt_items[rmt_items_length];

    // Break + marking
    rmt_items[cursor].level0 = 1;
    rmt_items[cursor].duration0 = bus->timing.break_us;
    rmt_items[cursor].level1 = SDI12_MARKING;
    rmt_items[cursor].duration1 = bus->timing.post_break_marking_us;
    ++cursor;

    // vTaskDelay(pdMS_TO_TICKS(80));
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

    esp_err_t ret = config_rmt_as_tx(bus);

    if (ret == ESP_OK)
    {
        ret = rmt_write_items(bus->rmt_tx_channel, rmt_items, rmt_items_length, 1);
    }

    return ret;
}

static esp_err_t sdi12_check_crc(const char *response)
{
    const uint8_t response_len = strlen(response);

    if (response_len <= 3)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t crc = 0;
    char crc_str[4] = { 0 };

    for (uint8_t i = 0; i < response_len - 3; ++i)
    {
        crc ^= (uint16_t)response[i];

        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= SDI12_CRC_POLY;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    crc_str[0] = (char)(0x0040 | (crc >> 12));
    crc_str[1] = (char)(0x0040 | ((crc >> 6) & 0x003F));
    crc_str[2] = (char)(0x0040 | (crc & 0x003F));

    if (strncmp(crc_str, response + response_len - 3, 3) == 0)
    {
        ESP_LOGD(TAG, "CRC: %s, Valid!", crc_str);
        return ESP_OK;
    }
    else
    {
        ESP_LOGD(TAG, "CRC: %s, Invalid!", crc_str);
        return ESP_ERR_INVALID_CRC;
    }
}

esp_err_t sdi12_bus_send_cmd(sdi12_bus_t *bus, const char *cmd, char *out_buffer, size_t out_buffer_length,
    bool check_crc, uint32_t timeout)
{
    uint8_t cmd_len = strlen(cmd);

    SDI12_CHECK(cmd, "CMD error", err_args);
    SDI12_CHECK(out_buffer, "No out buffer", err_args);
    SDI12_CHECK(out_buffer_length > 0, "Out buffer length error", err_args);

    SDI12_CHECK(((cmd[0] >= '0' && cmd[0] <= '9') || (cmd[0] >= 'a' && cmd[0] <= 'z')
                    || (cmd[0] >= 'A' && cmd[0] <= 'Z') || cmd[0] == '?'),
        "Invalidad sensor address", err_args);

    SDI12_CHECK(cmd[cmd_len - 1] == '!', "Invalid CMD terminator", err_args);
    ESP_LOGD(TAG, "Out: %s", cmd);

    SDI12_BUS_LOCK(bus);
    SDI12_APB_LOCK(bus);

    esp_err_t ret = write_cmd(bus, cmd);

    if (ret == ESP_OK)
    {
        ret = read_response_line(bus, out_buffer, out_buffer_length, timeout);

        if (ret == ESP_OK)
        {
            if ((cmd[1] == 'D' || cmd[1] == 'R') && check_crc)
            {
                ret = sdi12_check_crc(out_buffer);

                if (ret == ESP_OK)
                {
                    uint8_t response_len = strlen(out_buffer);
                    out_buffer[response_len - 3] = '\0'; // Clear CRC string
                }
            }

            // Command aM..! and aV..! require service request
            // Response should be "atttn"
            if (cmd[1] == 'M' || cmd[1] == 'V')
            {
                uint16_t seconds = 0;
                uint8_t factor = 100;

                for (uint8_t i = 1; i < 4; i++)
                {
                    seconds += (out_buffer[i] - '0') * factor;
                    factor /= 10;
                }

                // Only necessary if seconds is equal or greather than 1
                if (seconds > 0)
                {
                    char temp_buf[4] = { 0 };

                    ret = read_response_line(bus, temp_buf, sizeof(temp_buf), seconds * 1000);

                    if (ret == ESP_OK && strlen(temp_buf) > 0)
                    {
                        ret = temp_buf[0] == cmd[0] ? ESP_OK : ESP_FAIL;
                    }
                }
            }
        }
    }

    // Bus is always master and must be in low state while no transmissions, so keep it as TX.
    config_rmt_as_tx(bus);
    SDI12_APB_UNLOCK(bus);
    SDI12_BUS_UNLOCK(bus);

    return ret;

err_args:
    return ESP_ERR_INVALID_ARG;
}

esp_err_t sdi12_bus_deinit(sdi12_bus_t *bus)
{
    SDI12_CHECK(bus, "Invalid or NULL bus", err);
    esp_err_t ret = ESP_FAIL;

    if (bus->rmt_mode == RMT_MODE_TX)
    {
        ret = rmt_driver_uninstall(bus->rmt_tx_channel);
    }
    else
    {
        ret = rmt_driver_uninstall(bus->rmt_rx_channel);
    }

    free(bus);

    return ret;
err:
    return ESP_ERR_INVALID_ARG;
}

sdi12_bus_t *sdi12_bus_init(sdi12_bus_config_t *config)
{
#if CONFIG_SDI12_ENABLE_DEBUG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    SDI12_CHECK(config, "Config is NULL", err);
    SDI12_CHECK(IS_VALID_PIN(config->gpio_num), "Invalid GPIO pin", err);

    sdi12_bus_t *bus = calloc(1, sizeof(sdi12_bus_t));
    SDI12_CHECK(bus, "Can't allocate bus", err);

    bus->gpio_num = config->gpio_num;
    bus->rmt_tx_channel = config->rmt_tx_channel;
    bus->rmt_rx_channel = config->rmt_rx_channel;
    bus->rmt_mode = RMT_MODE_MAX; // Force firts time installation
    bus->timing.break_us = config->bus_timing.break_us != 0 ? config->bus_timing.break_us : SDI12_BREAK_US;
    bus->timing.post_break_marking_us = config->bus_timing.post_break_marking_us != 0
                                            ? config->bus_timing.post_break_marking_us
                                            : SDI12_POST_BREAK_MARKING_US;

    bus->mutex = xSemaphoreCreateMutex();

    SDI12_CHECK(bus->mutex, "Mutex allocation error", err_mutex);

#if CONFIG_PM_ENABLE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)                                                \
    && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 1, "SDI12_PM_LOCK", &bus->pm_lock);
#endif

    return bus;

err_mutex:
    free(bus);
err:
    return NULL;
}
