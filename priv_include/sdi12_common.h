#pragma once

#define SDI12_CHECK(a, str, goto_tag, ...)                                                                                                                     \
    do                                                                                                                                                         \
    {                                                                                                                                                          \
        if (!(a))                                                                                                                                              \
        {                                                                                                                                                      \
            ESP_LOGE(TAG, str, ##__VA_ARGS__);                                                                                                                 \
            goto goto_tag;                                                                                                                                     \
        }                                                                                                                                                      \
    } while (0)
