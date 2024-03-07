#ifndef __DHT_MODULE_H_INCLUDED__
#define __DHT_MODULE_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/queue.h"
#include "esp_err.h"

#include "driver/rmt_types.h"

typedef struct dht_config {
    int pin;
    unsigned int timeout;
} dht_config_t;

typedef struct {
    rmt_channel_handle_t rx_channel;
    rmt_channel_handle_t tx_channel;
    QueueHandle_t recv_queue;
    TickType_t timeout;
    rmt_encoder_handle_t copy_encoder;
} dht_handle;

typedef dht_handle *dht_handle_t;

esp_err_t dht_create(dht_config_t *config, dht_handle_t *new_handle);
esp_err_t dht_delete(dht_handle_t handle);
esp_err_t dht_get_value(dht_handle_t handle, float *temp, int *humi);

#ifdef __cplusplus
}
#endif

#endif
