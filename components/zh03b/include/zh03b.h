#ifndef __ZH03B_H__
#define __ZH03B_H__

#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ZH03B_RX_TIMEOUT 1000
#define ZH03B_BUFFER_SIZE 256
#define ZH03B_DATA_FRAME_SIZE 24
#define ZH03B_DATA_FRAME_BYTE_1 0x42
#define ZH03B_DATA_FRAME_BYTE_2 0x4D

/**
 * @brief Checks argument for validity
 *
 */
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

    /**
     * @brief Structure for ZH03B Sensor
     */
    typedef struct
    {
        uart_port_t uart_port; /* UART port used to communicate */
        uint8_t *buf;          /* Read buffer attached to this device */
    } zh03b_dev_t;

    /**
     * @brief               Initialize ZH03B device descriptor
     *
     * @param dev           Pointer to the sensor device data structure
     * @param uart_port     UART port number
     * @param tx_gpio       UART Tx pin number
     * @param rx_gpio       UART Rx pin number
     *
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t zh03b_init(zh03b_dev_t *dev, uart_port_t uart_port,
                         gpio_num_t tx_gpio, gpio_num_t rx_gpio);

    /**
     * @brief               Free ZH03B device descriptor
     *
     * @param dev           Pointer to the sensor device data structure
     * @return esp_err_t    ESP_OK on success
     */
    esp_err_t zh03b_deinit(zh03b_dev_t *dev);

    /**
     * @brief                   Read PM data from sensor
     *
     * @param dev               Pointer to the sensor device data structure
     * @param[out] pm1          PM1 value
     * @param[out] pm2_5        PM2.5 value
     * @param[out] pm10         PM10 value
     * @return esp_err_t        ESP_OK on success
     */
    esp_err_t zh03b_read_pm_value(zh03b_dev_t *dev, uint16_t *pm1, uint16_t *pm2_5, uint16_t *pm10);

    /**
     * @brief               Calculate Checksum
     *
     * @param data          Buffer pointer
     *
     * @return esp_err_t    ESP_OK for Checksum verified or ESP_ERR_INVALID_CRC for checksum fail
     */
    esp_err_t zh03b_calc_checksum(uint8_t *data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif