#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_check.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"

#include "driver/gpio.h"

#include "esp_log.h"

#include "sdi12_defs.h"
#include "sdi12_bus.h"

typedef struct sdi12_bus
{
    uint8_t gpio_num;
    sdi12_bus_timing_t timing;
    rmt_channel_handle_t rmt_tx_channel;
    rmt_channel_handle_t rmt_rx_channel;
    rmt_encoder_t *copy_encoder;
    QueueHandle_t receive_queue;
    SemaphoreHandle_t mutex;
} sdi12_bus_t;

#define SDI12_BUS_LOCK(b)                                                                                                                                      \
    if (b->mutex)                                                                                                                                              \
    xSemaphoreTake(b->mutex, portMAX_DELAY)

#define SDI12_BUS_UNLOCK(b)                                                                                                                                    \
    if (b->mutex)                                                                                                                                              \
    xSemaphoreGive(b->mutex)

/**
 * Largest response, excluding response to extended commands, are receive from aDx! or aRx! commands
 * From specs 1.4, maximum number of characters returned in <values> field is limited to 75 bytes.
 * To this 75, we must add possible CRC(3 bytes), start address(1 bytes), <CR><LF> end and '\0' C string terminator.
 * Total is 82.
 */
#define SDI12_MAX_RESPONSE_CHARS (82)

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define SDI12_RMT_CLK_SRC RMT_CLK_SRC_REF_TICK
#else
#define SDI12_RMT_CLK_SRC RMT_CLK_SRC_DEFAULT
#endif

static const char *TAG = "sdi12 bus";

/**
 * @brief Configure RMT channel as transmisor
 *
 * @param bus         bus object
 * @return esp_err_t
 *      - ESP_FAIL RMT config install error
 *      - ESP_OK  configuration and installation OK
 */
static esp_err_t config_rmt_as_tx(sdi12_bus_t *bus)
{
    rmt_tx_channel_config_t tx_channel_config = {
        .gpio_num = bus->gpio_num,        // GPIO number
        .clk_src = SDI12_RMT_CLK_SRC,   // select source clock
        .resolution_hz = 1 * 1000 * 1000, // 1MHz tick resolution, i.e. 1 tick = 1us
        .mem_block_symbols = 64,          // memory block size, 64 * 4 = 256Bytes
        .trans_queue_depth = 6,
        .flags  = {
            .io_loop_back = false,
            .invert_out = false, // don't invert input signal
            .with_dma = false,  // don't need DMA backend
        }, 
    };

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_channel_config, &bus->rmt_tx_channel), TAG, "create rmt tx channel error");

    ESP_RETURN_ON_ERROR(rmt_enable(bus->rmt_tx_channel), TAG, "rmt tx enable error");

    return ESP_OK;
}

static bool sdi12_rmt_receive_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *data, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, data, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
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
    rmt_rx_channel_config_t rx_channel_config = {
        .gpio_num = bus->gpio_num,
        .clk_src = SDI12_RMT_CLK_SRC,
        .mem_block_symbols = 128,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz tick resolution, i.e. 1 tick = 1us
        .flags = {
            .io_loop_back = false,
            .invert_in = false,
            .with_dma = false,
        },
    };

    // Workaround to enable PULLDOWN on pin. rmt_new_rx_channel enable by default pull up
    // and there is no way to change it.
    gpio_hold_en(bus->gpio_num);
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_channel_config, &bus->rmt_rx_channel), TAG, "create rmt rx channel failed");
    gpio_hold_dis(bus->gpio_num);
    gpio_set_pull_mode(bus->gpio_num, GPIO_PULLDOWN_ONLY);

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = sdi12_rmt_receive_done_callback,
    };

    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(bus->rmt_rx_channel, &cbs, bus->receive_queue), TAG, "error registering rx callback");
    ESP_RETURN_ON_ERROR(rmt_enable(bus->rmt_rx_channel), TAG, "error enabling rx channel");

    return ESP_OK;
}

static esp_err_t set_idle_bus(sdi12_bus_t *bus)
{
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        // also enable the input path is `io_loop_back` is on, this is useful for debug
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = false,
        .pull_up_en = false,
        .pin_bit_mask = 1ULL << bus->gpio_num,
    };

    gpio_hold_dis(bus->gpio_num);

    ESP_RETURN_ON_ERROR(gpio_config(&gpio_conf), TAG, "set idle bus error");

    return gpio_set_level(bus->gpio_num, 0);
}

/**
 * @brief Parse RMT symbols into char buffer. Stop when SDI12 response end (\r\n) is found
 *
 * @param bus         bus object
 * @param raw_symbols         received rmt symbols
 * @param symbols_length  received rmt symbols length
 * @return esp_err_t
 *      - ESP_ERR_INVALID_ARG bus or symbol are NULL or symbol_length <= 0
 *      - ESP_ERR_NOT_FOUND SDI12 end isn't found
 *      - ESP_OK SDI12 end is found and parse ok
 */
static esp_err_t parse_response(sdi12_bus_t *bus, rmt_symbol_word_t *raw_symbols, size_t symbols_length, char *out_buffer, size_t out_buffer_length)
{
    memset(out_buffer, '\0', out_buffer_length);

    size_t char_index = 0;
    size_t symbol_index = 0;
    bool level0 = 0; // False if level0, duration0 needed. True when level1, duration1
    uint8_t bit_counter = 0;
    uint8_t level;
    uint8_t number_of_bits;
    char c = 0;
    bool parity = false;

    while (symbol_index < symbols_length)
    {
        if (!level0)
        {
            level = raw_symbols[symbol_index].level0;
            // (raw_symbols[index].duration0 + SDI12_BIT_WIDTH_US / 2) / SDI12_BIT_WIDTH_US -> Solve integer division round.
            number_of_bits = (raw_symbols[symbol_index].duration0 + SDI12_BIT_WIDTH_US / 2) / SDI12_BIT_WIDTH_US;
        }
        else
        {
            level = raw_symbols[symbol_index].level1;
            number_of_bits = (raw_symbols[symbol_index].duration1 + SDI12_BIT_WIDTH_US / 2) / SDI12_BIT_WIDTH_US;
            ++symbol_index;
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
                            ESP_LOGD(TAG, "RX: %s", out_buffer);
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

static esp_err_t read_response_line(sdi12_bus_t *bus, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "bus is NULL");

    esp_err_t ret;

    ret = config_rmt_as_rx(bus);
    // ret = rmt_enable(bus->rmt_rx_channel);

    if (ret != ESP_OK)
    {
        return ret;
    }

    rmt_symbol_word_t raw_symbols[128];
    rmt_rx_done_event_data_t rx_data;
    uint32_t aux_timeout = timeout != 0 ? timeout : SDI12_DEFAULT_RESPONSE_TIMEOUT;

    rmt_receive_config_t receive_config = {
    
        // Check @link https://github.com/espressif/esp-idf/issues/11262.
        // Max range_min_ns value use rmt group resolution and must be a value allocatable in a 8-bit width reg.
        // Group resolution is the same as RMT source clock

        // Group resolution = 80Mhz
        .signal_range_min_ns = 3186,
        .signal_range_max_ns = (SDI12_BREAK_US + 500) * 1000, // the longest duration for SDI12 signal is break signal
    };

    ret = rmt_receive(bus->rmt_rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);

    if (ret == ESP_OK)
    {
        if (xQueueReceive(bus->receive_queue, &rx_data, pdMS_TO_TICKS(aux_timeout)) == pdPASS)
        {
            if (rx_data.num_symbols > 0)
            {
                // for (size_t i = 0; i < rx_data.num_symbols; i++)
                // {
                //     printf("Level: %d | Duration: %d \n", rx_data.received_symbols[i].level0, rx_data.received_symbols[i].duration0);
                //     printf("Level: %d | Duration: %d \n", rx_data.received_symbols[i].level1, rx_data.received_symbols[i].duration1);
                // }

                ret = parse_response(bus, rx_data.received_symbols, rx_data.num_symbols, out_buffer, out_buffer_length);
            }
        }
        else
        {
            ESP_LOGD(TAG, "no rmt symbols received");

            ret = ESP_ERR_TIMEOUT;
        }
    }

    // Skip gpio reset on disable rmt_disable()
    gpio_hold_en(bus->gpio_num);
    rmt_disable(bus->rmt_rx_channel);
    rmt_del_channel(bus->rmt_rx_channel);
    set_idle_bus(bus);

    return ret;
}

static void encode_cmd(sdi12_bus_timing_t *timing, const char *cmd, rmt_symbol_word_t *rmt_symbols_out, size_t rmt_symbols_len)
{
    size_t rmt_symbol_index = 0;
    // Break + marking
    rmt_symbols_out[rmt_symbol_index].level0 = 1;
    rmt_symbols_out[rmt_symbol_index].duration0 = timing->break_us;
    rmt_symbols_out[rmt_symbol_index].level1 = SDI12_MARKING;
    rmt_symbols_out[rmt_symbol_index].duration1 = timing->post_break_marking_us;
    ++rmt_symbol_index;

    uint8_t char_index = 0;
    size_t encode_len = rmt_symbols_len - 1; // Remove break + marking symbol

    while (encode_len > 0)
    {
        // start from last time truncated encoding
        char cur_byte = cmd[char_index];
        uint8_t bit_index = 0;
        uint8_t level_to_write;
        bool parity_bit = false;

        while ((encode_len > 0) && (bit_index < 10))
        {
            switch (bit_index)
            {
                case 0: // start bit
                    rmt_symbols_out[rmt_symbol_index].level0 = SDI12_SPACING;
                    rmt_symbols_out[rmt_symbol_index].duration0 = SDI12_BIT_WIDTH_US;
                    break;

                case 8: // parity bit
                    rmt_symbols_out[rmt_symbol_index].level0 = parity_bit;
                    rmt_symbols_out[rmt_symbol_index].duration0 = SDI12_BIT_WIDTH_US;

                    break;

                case 9: // stop bit
                    rmt_symbols_out[rmt_symbol_index].level1 = SDI12_MARKING;
                    rmt_symbols_out[rmt_symbol_index].duration1 = SDI12_BIT_WIDTH_US;
                    break;

                default:                 // case 1 to 7, char bits

                    if (cur_byte & 0x01) // bit == 1; Inverse -> 0 to write
                    {
                        level_to_write = SDI12_MARKING;
                    }
                    else // bit == 1; Inverse -> 1 to write
                    {
                        level_to_write = SDI12_SPACING;
                        parity_bit = !parity_bit;
                    }

                    if (bit_index % 2 == 0)
                    {
                        rmt_symbols_out[rmt_symbol_index].level0 = level_to_write;
                        rmt_symbols_out[rmt_symbol_index].duration0 = SDI12_BIT_WIDTH_US;
                    }
                    else
                    {
                        rmt_symbols_out[rmt_symbol_index].level1 = level_to_write;
                        rmt_symbols_out[rmt_symbol_index].duration1 = SDI12_BIT_WIDTH_US;
                    }

                    cur_byte >>= 1;

                    break;
            }

            ++bit_index;
            if (bit_index % 2 == 0)
            {
                ++rmt_symbol_index;
                --encode_len;
            }
        }

        ++char_index;
    }
}

static esp_err_t write_cmd(sdi12_bus_t *bus, const char *cmd)
{
    ESP_RETURN_ON_ERROR(config_rmt_as_tx(bus), TAG, "error on tx config");

    // Initial Break & marking + chars. Every char need 10 bits transfers so it needs 5 rmt_symbol_word
    size_t rmt_symbols_len = 1 + strlen(cmd) * 5;
    rmt_symbol_word_t rmt_symbols[rmt_symbols_len];

    encode_cmd(&bus->timing, cmd, rmt_symbols, rmt_symbols_len);

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };

    esp_err_t ret = rmt_transmit(bus->rmt_tx_channel, bus->copy_encoder, rmt_symbols, sizeof(rmt_symbol_word_t) * rmt_symbols_len, &tx_config);

    if (ret == ESP_OK)
    {
        ret = rmt_tx_wait_all_done(bus->rmt_tx_channel, 1000);

        gpio_hold_en(bus->gpio_num);
        rmt_disable(bus->rmt_tx_channel);
        rmt_del_channel(bus->rmt_tx_channel);

        set_idle_bus(bus);

        // gpio_set_level(bus->gpio_num, 0);
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

esp_err_t sdi12_bus_send_cmd(sdi12_bus_handle_t bus, const char *cmd, bool crc, char *out_buffer, size_t out_buffer_length, uint32_t timeout)
{
    uint8_t cmd_len = strlen(cmd);

    ESP_RETURN_ON_FALSE(cmd, ESP_ERR_INVALID_ARG, TAG, "invalid command");
    ESP_RETURN_ON_FALSE(out_buffer, ESP_ERR_INVALID_ARG, TAG, "no out buffer");
    ESP_RETURN_ON_FALSE(out_buffer_length > 0, ESP_ERR_INVALID_ARG, TAG, "out buffer length error");

    ESP_RETURN_ON_FALSE(((cmd[0] >= '0' && cmd[0] <= '9') || (cmd[0] >= 'a' && cmd[0] <= 'z') || (cmd[0] >= 'A' && cmd[0] <= 'Z') || cmd[0] == '?'),
        ESP_ERR_INVALID_ARG, TAG, "Invalidad sensor address");

    ESP_RETURN_ON_FALSE(cmd[cmd_len - 1] == '!', ESP_ERR_INVALID_ARG, TAG, "Invalid CMD terminator");
    ESP_LOGD(TAG, "TX: %s", cmd);

    // each time RMT is installed/uninstalled INFO message is printed from GPIO component, so it is disabled during cmd time to clean up log messages
    esp_log_level_set("gpio", ESP_LOG_WARN);

    SDI12_BUS_LOCK(bus);

    esp_err_t ret = write_cmd(bus, cmd);

    if (ret == ESP_OK)
    {
        ret = read_response_line(bus, out_buffer, out_buffer_length, timeout);
        // gpio_set_pull_mode(bus->gpio_num, GPIO_PULLDOWN_ONLY);

        if (ret == ESP_OK)
        {
            if ((cmd[1] == 'D' || cmd[1] == 'R') && crc)
            {
                ret = sdi12_check_crc(out_buffer);

                if (ret == ESP_OK)
                {
                    uint8_t response_len = strlen(out_buffer);
                    out_buffer[response_len - 3] = '\0'; // Clear CRC string
                }
            }
            else if (cmd[1] == 'M' || cmd[1] == 'V' || cmd[1] == 'H')
            {
                // Command aM..! and aV..! require service request
                // Response should be "atttn", "atttnn" or "atttnnn"
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
                        ret = temp_buf[0] == cmd[0] ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
                    }
                    else if (ret == ESP_ERR_TIMEOUT)
                    {
                        ret = ESP_ERR_NOT_FINISHED;
                    }
                }
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "write error");
    }

    // Bus is always master and must be in low state while no transmissions, so keep it as TX.
    // config_rmt_as_tx(bus);
    // ret = set_idle_bus(bus);
    SDI12_BUS_UNLOCK(bus);

    esp_log_level_set("gpio", CONFIG_LOG_DEFAULT_LEVEL);

    return ret;
}

esp_err_t sdi12_del_bus(sdi12_bus_handle_t bus)
{
    esp_err_t ret = ESP_FAIL;

    if (bus->rmt_tx_channel)
    {
        rmt_del_channel(bus->rmt_tx_channel);
    }

    if (bus->rmt_rx_channel)
    {
        rmt_del_channel(bus->rmt_rx_channel);
    }

    if (bus->copy_encoder) { }

    if (bus->mutex)
    {
        vSemaphoreDelete(bus->mutex);
    }

    free(bus);

    return ret;
}

esp_err_t sdi12_new_bus(sdi12_bus_config_t *config, sdi12_bus_handle_t *sdi12_bus_out)
{
#if CONFIG_SDI12_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif

    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config is NULL");

    // GPIO selected must be input and output capable.
    // ESP_RETURN_ON_FALSE(GPIO_IS_VALID_DIGITAL_IO_PAD(config->gpio_num), ESP_ERR_INVALID_ARG, TAG, "Invalid GPIO pin");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->gpio_num), ESP_ERR_INVALID_ARG, TAG, "Invalid GPIO pin");

    sdi12_bus_t *bus = calloc(1, sizeof(sdi12_bus_t));

    ESP_RETURN_ON_FALSE(bus, ESP_ERR_NO_MEM, TAG, "can't allocate bus");

    bus->gpio_num = config->gpio_num;
    bus->timing.break_us = config->bus_timing.break_us != 0 ? config->bus_timing.break_us : SDI12_BREAK_US;
    bus->timing.post_break_marking_us = config->bus_timing.post_break_marking_us != 0 ? config->bus_timing.post_break_marking_us : SDI12_POST_BREAK_MARKING_US;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &bus->copy_encoder), err_encoder, TAG, "can't allocate copy encoder");
    bus->mutex = xSemaphoreCreateMutex();

    ESP_GOTO_ON_FALSE(bus->mutex, ESP_ERR_NO_MEM, err_mutex, TAG, "can't allocate bus mutex");

    bus->receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    ESP_GOTO_ON_FALSE(bus->receive_queue, ESP_ERR_NO_MEM, err_queue, TAG, "can't allocate receive queue");

    set_idle_bus(bus);

    *sdi12_bus_out = bus;
    return ret;

err_queue:
    vSemaphoreDelete(bus->mutex);
err_mutex:
    rmt_del_encoder(bus->copy_encoder);
err_encoder:
    free(bus);
    return ret;
}
