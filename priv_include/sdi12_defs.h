#pragma once

#include "soc/soc_caps.h"
#include "hal/rmt_types.h"
#include "hal/rmt_hal.h"
#include "hal/dma_types.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"


#if CONFIG_RMT_ISR_IRAM_SAFE
#define RMT_MEM_ALLOC_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define RMT_MEM_ALLOC_CAPS MALLOC_CAP_DEFAULT
#endif

// RMT driver object is per-channel, the interrupt source is shared between channels
#if CONFIG_RMT_ISR_IRAM_SAFE
#define RMT_INTR_ALLOC_FLAG (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM)
#else
#define RMT_INTR_ALLOC_FLAG (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED)
#endif

#define SDI12_BREAK_US              (12200)
#define SDI12_POST_BREAK_MARKING_US (8333)
#define SDI12_BIT_WIDTH_US          (833)

#define SDI12_MARKING (0)
#define SDI12_SPACING (1)

#define SDI12_CRC_POLY 0xA001