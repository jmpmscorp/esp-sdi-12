#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "esp_log.h"

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include "sdi12.h"

#define SDI12_DATA_GPIO 19
#define SDI12_RMT_CHANNEL 0

static const char *TAG = "[MAIN]";

static void read_sensor_task(void *arg)
{
   sdi12_bus_t *bus = (sdi12_bus_t *)arg;
   char address;
   char buffer[50] = "";
   esp_err_t ret;

   do
   {
      ret = bus->address_query(bus, &address, 5000);

      if (ret != ESP_OK)
      {
         /** If you want to find address sensor, ensure you only have one sensor connected on bus.
          *  If there are many sensors, they will all respond to command causing bus contention.
          */
         char err_buf[50];
         esp_err_to_name_r(ret, err_buf, sizeof(err_buf));
         ESP_LOGI(TAG, "Can't found sensor. Error: %s", err_buf);
         vTaskDelay(pdMS_TO_TICKS(2500));
         ESP_LOGI(TAG, "Address Query Retry");
      }
   } while (ret != ESP_OK);

   ESP_LOGI(TAG, "Sensor Address: %c", address);

   // There is query information command implement but this shows raw_cmd
   ret = bus->raw_cmd(bus, address, "I", buffer, sizeof(buffer), 0);

   if (ret == ESP_OK)
   {
      ESP_LOGI(TAG, "Sensor Identification: %s", buffer);
   }
   else
   {
      ESP_LOGI(TAG, "Can't read sensor identification");
   }

   while (true)
   {
      uint8_t num_values;

      ESP_LOGI(TAG, "----  READING SENSOR  -----");
      if (bus->start_measurement(bus, address, &num_values, 0) == ESP_OK)
      {
         ESP_LOGI(TAG, "Sensor provide %d values", num_values);
         ret = bus->read_values(bus, address, 0, buffer, sizeof(buffer), 0);
         ESP_LOGI(TAG, "Ret: %d, Buffer: %s", ret, buffer);
      }
      // vTaskDelay(portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(60000));
   }
}

void app_main(void)
{

#if CONFIG_PM_ENABLE
   esp_pm_config_esp32_t pm_config = {
      .max_freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ,
      .min_freq_mhz = CONFIG_ESP32_XTAL_FREQ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
      .light_sleep_enable = false
#endif
   };

   ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

   sdi12_bus_config_t config = {
       .gpio_num = SDI12_DATA_GPIO,
       .rmt_channel = SDI12_RMT_CHANNEL,
   };

   sdi12_bus_t *sdi12_bus = sdi12_bus_init(&config);

   ESP_LOGI(TAG, "Init");

   xTaskCreatePinnedToCore(read_sensor_task, "", configMINIMAL_STACK_SIZE * 3, sdi12_bus, 6, NULL, 1);

   // ESP_LOGI(TAG, "Aqui 2");
}
