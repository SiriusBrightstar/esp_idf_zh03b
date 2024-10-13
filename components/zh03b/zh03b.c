#include "string.h"

#include "esp_err.h"
#include "esp_log.h"

#include "zh03b.h"

static const char *TAG = "ZH03B";

esp_err_t zh03b_init(zh03b_dev_t *dev, uart_port_t uart_port,
                     gpio_num_t tx_gpio, gpio_num_t rx_gpio)
{
    esp_err_t err;

    CHECK_ARG(dev);
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    err = uart_driver_install(uart_port, ZH03B_BUFFER_SIZE, ZH03B_BUFFER_SIZE, 0, NULL, 0);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_param_config(uart_port, &uart_config);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_set_pin(uart_port, tx_gpio, rx_gpio, GPIO_NUM_NC, GPIO_NUM_NC);
    if (err != ESP_OK)
    {
        return err;
    }

    dev->uart_port = uart_port;
    dev->buf = (uint8_t *)malloc(ZH03B_DATA_FRAME_SIZE + 1);
    if (!dev->buf)
    {
        return ESP_ERR_NO_MEM;
    }
    uint8_t write_cmd[] = {0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};
    int bytes_written = uart_write_bytes(dev->uart_port, (const char *)write_cmd, sizeof(write_cmd));
    if (bytes_written < 0)
    {
        ESP_LOGE(TAG, "Failed to write bytes to ZH03B Sensor");
        zh03b_deinit(dev);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Successfully initialized UART driver for ZH03B");
    return ESP_OK;
}

esp_err_t zh03b_deinit(zh03b_dev_t *dev)
{
    CHECK_ARG(dev && dev->buf);

    free(dev->buf);
    dev->buf = NULL;
    return uart_driver_delete(dev->uart_port);
}

esp_err_t zh03b_read_pm_value(zh03b_dev_t *dev, uint16_t *pm1, uint16_t *pm2_5, uint16_t *pm10)
{
    CHECK_ARG(dev && pm1 && pm2_5 && pm10);
    memset(dev->buf, 0, ZH03B_DATA_FRAME_SIZE);

    // uart_flush(dev->uart_port);
    int len = uart_read_bytes(dev->uart_port, dev->buf, ZH03B_DATA_FRAME_SIZE, ZH03B_RX_TIMEOUT / portTICK_PERIOD_MS);
    ESP_LOG_BUFFER_HEXDUMP(TAG, dev->buf, len, ESP_LOG_INFO);

    if ((len == ZH03B_DATA_FRAME_SIZE) && (dev->buf[0] == ZH03B_DATA_FRAME_BYTE_1) && (dev->buf[1] == ZH03B_DATA_FRAME_BYTE_2))
    {
        if (zh03b_calc_checksum(dev->buf) != ESP_OK)
        {
            ESP_LOG_BUFFER_HEXDUMP(TAG, dev->buf, len, ESP_LOG_ERROR);
            return ESP_ERR_INVALID_CRC;
        }
        *pm1 = (dev->buf[10] << 8) | dev->buf[11];
        ESP_LOGD(TAG, "PM1: %dug/m3", *pm1);
        *pm2_5 = (dev->buf[12] << 8) | dev->buf[13];
        ESP_LOGD(TAG, "PM2.5: %dug/m3", *pm2_5);
        *pm10 = (dev->buf[14] << 8) | dev->buf[15];
        ESP_LOGD(TAG, "PM10: %dug/m3", *pm10);

        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "UART Frame Length: %d", len);
        ESP_LOGE(TAG, "%s", esp_err_to_name(ESP_ERR_INVALID_RESPONSE));
        ESP_LOG_BUFFER_HEXDUMP(TAG, dev->buf, len, ESP_LOG_ERROR);
        return ESP_ERR_INVALID_RESPONSE;
    }
}

esp_err_t zh03b_calc_checksum(uint8_t *data)
{
    uint16_t calc_checksum = 0;

    for (uint8_t i = 0; i < ZH03B_DATA_FRAME_SIZE - 2; i++)
    {
        calc_checksum += data[i];
    }

    uint16_t data_checksum_high_byte = data[ZH03B_DATA_FRAME_SIZE - 2];
    uint16_t data_checksum_low_byte = data[ZH03B_DATA_FRAME_SIZE - 1];
    uint16_t data_checksum = (data_checksum_high_byte << 8) | data_checksum_low_byte;

    return (calc_checksum == data_checksum) ? ESP_OK : ESP_ERR_INVALID_CRC;
}