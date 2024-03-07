#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"

#include "pm1006.h"


const char PM1006_CMD[4] = {0x11, 0x01, 0x02, 0xec};


esp_err_t pm1006_create(pm1006_config_t *config, pm1006_handle_t *new_handle) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .parity    = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_RETURN_ON_ERROR(
        uart_param_config(config->uart_num, &uart_config),
        "PM1006", "Failed: uart_param_config"
    );
    ESP_RETURN_ON_ERROR(
        uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
        "PM1006", "Failed: uart_set_pin"
    );
    ESP_RETURN_ON_ERROR(
        uart_driver_install(config->uart_num, config->rx_buf_size, config->tx_buf_size, 0, NULL, 0),
        "PM1006", "Failed: uart_driver_install"
    );

    pm1006_handle_t handle = malloc(sizeof(pm1006_handle_t));
    handle->uart_num = config->uart_num;
    handle->timeout = config->timeout;
    *new_handle = handle;

    return ESP_OK;
}

esp_err_t pm1006_delete(pm1006_handle_t handle) {
    uart_port_t port = handle->uart_num;
    free(handle);
    return uart_driver_delete(port);
}

esp_err_t pm1006_get_value(pm1006_handle_t handle, int *result) {
    static char buf[24];
    uart_flush(handle->uart_num);
    uart_write_bytes(handle->uart_num, PM1006_CMD, 4);
    uart_read_bytes(handle->uart_num, buf, 16, pdMS_TO_TICKS(handle->timeout));

    if (buf[0] == 0x16)
        goto PASS;
    char *data = buf;
    for (int i = 1; i < 8; i++) {
        if (*data == 0x16) {
            uart_read_bytes(handle->uart_num, buf, i, pdMS_TO_TICKS(handle->timeout));
            goto PASS;
        } else {
            ESP_LOGI("PM1006:get_value", "Invalid value 0x%02x", *data);
        }
    }
    ESP_LOGI("PM1006:get_value", "Cannot find valid PM1006 header.");
    return ESP_ERR_NOT_FOUND;

PASS:
    ESP_LOGD(
        "PM1006:get_value", "Raw data: %02x %02x %02x | %02x %02x %02x %02x | %02x %02x %02x %02x | %02x %02x %02x %02x | %02x",
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
        data[10], data[11], data[12], data[13], data[14], data[15]
    );
    ESP_RETURN_ON_FALSE(
        data[1] == 0x0d && data[2] == 0x02, ESP_ERR_INVALID_RESPONSE,
        "PM1006:get_value", "Invalid PM1006 header received."
    );

    /*
    uint8_t checksum = 0;
    for (int i = 0; i < 14; i++)
        checksum += data[i];
    ESP_RETURN_ON_FALSE(
        checksum == data[15], ESP_ERR_INVALID_RESPONSE,
        "PM1006:get_value", "Checksum mismatch."
    );
    */

    *result = ((int) data[5]) << 8 | data[6];
    return ESP_OK;
}

