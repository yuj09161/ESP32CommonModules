#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"
#include "driver/rmt_types.h"
#include "esp_log.h"
#include "esp_check.h"

#include "dht_rmt.h"

#define SYMBOLS_CNT 42
#define BUFFER_SIZE (SYMBOLS_CNT + 5)
#define RX_DONE_ARRAY_INDEX ((UBaseType_t)0)


const rmt_symbol_word_t DHT11_START_SIGNAL = {
    .duration0 = 24000,
    .level0 = 0,
    .duration1 = 30,
    .level1 = 1
};
const rmt_transmit_config_t DHT11_TX_CFG = {
    .loop_count = 0,
    .flags.eot_level = 1
};
const rmt_receive_config_t DHT11_RX_CFG = {
    .signal_range_min_ns = 2500,  // 2.5us
    .signal_range_max_ns = 500000 // 500us
};

bool _rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *evt, void *user_dat) {
    BaseType_t do_yield = pdFALSE;
    ESP_DRAM_LOGI("DHT:rx_done_callback", "Event: count: %d | addr: %p", evt->num_symbols, evt->received_symbols);
    xQueueSendFromISR(((dht_handle_t) user_dat)->recv_queue, evt, &do_yield);
    return do_yield;
}

esp_err_t dht_create(dht_config_t *config, dht_handle_t *new_handle) {
    // Handle
    dht_handle_t handle = malloc(sizeof(dht_handle));
    ESP_RETURN_ON_FALSE(
        handle->recv_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t)),
        ESP_ERR_NO_MEM, "DHT:create", "Failed to allocate receive queue."
    );
    ESP_LOGI("DHT:create", "Queue created. (Address: %p)", handle->recv_queue);
    handle->timeout = pdMS_TO_TICKS(config->timeout);
    *new_handle = handle;

    // Encoder
    const rmt_copy_encoder_config_t encoder_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_cfg, &handle->copy_encoder));

    // Rx Channel
    const rmt_rx_channel_config_t rx_cfg = {
        .gpio_num           = config->pin,
        .clk_src            = RMT_CLK_SRC_DEFAULT,
        .resolution_hz      = 1000000,  // 1MHz -> 1µs/sample
        .mem_block_symbols  = 64,
        .flags.io_loop_back = 0,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &handle->rx_channel));
    const rmt_rx_event_callbacks_t rx_callbacks = {
        .on_recv_done = _rx_done_callback
    };
    rmt_rx_register_event_callbacks(handle->rx_channel, &rx_callbacks, handle);
    ESP_ERROR_CHECK(rmt_enable(handle->rx_channel));

    // Tx Channel
    const rmt_tx_channel_config_t tx_cfg = {
        .gpio_num           = config->pin,
        .clk_src            = RMT_CLK_SRC_DEFAULT,
        .resolution_hz      = 1000000,  // 1MHz -> 1µs/sample
        .mem_block_symbols  = 64,
        .trans_queue_depth  = 4,
        .flags.io_loop_back = 1,
        .flags.io_od_mode   = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &handle->tx_channel));
    ESP_ERROR_CHECK(rmt_enable(handle->tx_channel));

    return ESP_OK;
}

esp_err_t dht_delete(dht_handle_t handle) {
    ESP_ERROR_CHECK(rmt_del_channel(handle->tx_channel));
    ESP_ERROR_CHECK(rmt_del_channel(handle->rx_channel));
    ESP_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder));
    vQueueDelete(handle->recv_queue);
    free(handle);
    return ESP_OK;
}

esp_err_t _dht_get_raw(dht_handle_t handle, rmt_symbol_word_t *buffer, rmt_rx_done_event_data_t *result) {
    // Configure RMT -> send request
    ESP_RETURN_ON_ERROR(
        rmt_receive(handle->rx_channel, buffer, sizeof(rmt_symbol_word_t) * BUFFER_SIZE, &DHT11_RX_CFG),
        "DHT:get_raw", "RX setting failed."
    );
    ESP_RETURN_ON_ERROR(
        rmt_transmit(handle->tx_channel, handle->copy_encoder, &DHT11_START_SIGNAL, sizeof(rmt_symbol_word_t), &DHT11_TX_CFG),
        "DHT:get_raw", "TX failed."
    );

    ESP_LOGI("DHT:get_raw", "Receive Queue: %p", handle->recv_queue);
    ESP_RETURN_ON_FALSE(
        xQueueReceive(handle->recv_queue, result, handle->timeout) == pdTRUE,
        ESP_ERR_INVALID_RESPONSE, "DHT:get_raw", "Cannot receive data from queue."
    );
    ESP_LOGI("DHT:get_raw", "Result: count: %d | addr: %p", result->num_symbols, result->received_symbols);
    return ESP_OK;
}

esp_err_t _dht_parse(dht_handle_t handle, rmt_rx_done_event_data_t *raw, float *temp, int *humi) {
    if (raw->num_symbols < SYMBOLS_CNT) {
        ESP_LOGW("DHT:get_value", "Too short response.");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_ERR_NOT_SUPPORTED;

    /*
    for (int i = 0; i < raw->num_symbols - SYMBOLS_CNT + 1; i++) {
        if () {
            goto PASS;
        } else {
            ESP_LOGI("PM1006:get_value", "Invalid. (level: %d, time: %d)", *data);
        }
    }

:PASS
    // Todo: Implement parse
    */

    return ESP_OK;
}

esp_err_t dht_get_value(dht_handle_t handle, float *temp, int *humi) {
    rmt_symbol_word_t buffer[BUFFER_SIZE];
    rmt_rx_done_event_data_t raw_result;
    ESP_RETURN_ON_ERROR(
        _dht_get_raw(handle, buffer, &raw_result),
        "DHT:get_value", "Failed to communicate DHT."
    );

    // Symbol to array
    ESP_LOGI("DHT:get", "%d symbol received.", raw_result.num_symbols);
    if (esp_log_level_get("DHT:get") >= ESP_LOG_INFO) {
        for (int i = 0; i < raw_result.num_symbols; i++) {
            ESP_LOGI(
                "DHT:get",
                "Symbol #%d | duration0: %d | level0: %d | duration1: %d | level1: %d", i,
                raw_result.received_symbols[i].duration0, raw_result.received_symbols[i].level0,
                raw_result.received_symbols[i].duration1, raw_result.received_symbols[i].level1
            );
        }
    }

    ESP_RETURN_ON_ERROR(
        _dht_parse(handle, &raw_result, temp, humi),
        "DHT:get_value", "Failed to parse response."
    );
    return ESP_OK;
}