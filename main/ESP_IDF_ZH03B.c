#include <stdio.h>
#include "zh03b.h"
#include "esp_err.h"
#include "esp_log.h"

static const uint8_t RX_PIN = 16;
static const uint8_t TX_PIN = 17;

static const char *TAG = "ZH03B_EXAMPLE";

void app_main(void)
{
    zh03b_dev_t dev;
    uint16_t pm1_val, pm2_5_val, pm10_val;

    ESP_ERROR_CHECK(zh03b_init(&dev, UART_NUM_1, TX_PIN, RX_PIN));

    while (true)
    {
        ESP_ERROR_CHECK(zh03b_read_pm_value(&dev, &pm1_val, &pm2_5_val, &pm10_val));
        printf("--*--\n");
        ESP_LOGI(TAG, "PM1.0 Value:    %dppm", pm1_val);
        ESP_LOGI(TAG, "PM2.5 Value:  %dppm", pm2_5_val);
        ESP_LOGI(TAG, "PM10 Value:   %dppm", pm10_val);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
