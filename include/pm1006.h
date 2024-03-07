#ifndef __PM1006_MODULE_H_INCLUDED__
#define __PM1006_MODULE_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

#include "driver/uart.h"

typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int tx_buf_size;
    int rx_buf_size;
    int timeout;
} pm1006_config_t;

typedef struct pm1006_handle {
    uart_port_t uart_num;
    int timeout;
} *pm1006_handle_t;

esp_err_t pm1006_create(pm1006_config_t *config, pm1006_handle_t *new_handle);
esp_err_t pm1006_delete(pm1006_handle_t handle);
esp_err_t pm1006_get_value(pm1006_handle_t handle, int *result);

#ifdef __cplusplus
}
#endif

#endif