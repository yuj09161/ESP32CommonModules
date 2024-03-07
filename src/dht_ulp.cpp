#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_check.h"
#include "ulp_common.h"
#include "ulp_riscv.h"

#include "ulp_read_sensor_values.h"
#include "ulp/ulp_defs.h"
#include "dht_ulp.h"


#define ULP_CYCLES_PER_US 8.5
#define DHT_RESPONSE_START_SHORT_THRESHOLD ((int)(10) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_START_LONG_THRESHOLD  ((int)(45) * ULP_CYCLES_PER_US)
#define DHT_ACK_RESPONSE_SHORT_THRESHOLD   ((int)(70) * ULP_CYCLES_PER_US)
#define DHT_ACK_RESPONSE_LONG_THRESHOLD    ((int)(95) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_L_SHORT_THRESHOLD     ((int)(40) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_L_LONG_THRESHOLD      ((int)(60) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_0_SHORT_THRESHOLD     ((int)(15) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_0_LONG_THRESHOLD      ((int)(30) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_1_SHORT_THRESHOLD     ((int)(60) * ULP_CYCLES_PER_US)
#define DHT_RESPONSE_1_LONG_THRESHOLD      ((int)(80) * ULP_CYCLES_PER_US)

extern const uint8_t ulp_bin_start[] asm("_binary_ulp_read_sensor_values_bin_start");
extern const uint8_t ulp_bin_end[]   asm("_binary_ulp_read_sensor_values_bin_end");


DHT::DHT(gpio_num_t pin) {
    ESP_ERROR_CHECK(ulp_riscv_load_binary(ulp_bin_start, ulp_bin_end - ulp_bin_start));
    ulp_set_wakeup_period(0, 1000000); // wakeup once per 1s
    ESP_ERROR_CHECK(ulp_riscv_run());

    ulp_pin = (uint32_t)pin;
    this->raw_times = (uint16_t *)(&ulp_raw_values);
}

esp_err_t DHT::get(DHTResult *result) {
    uint8_t raw_bytes[5];
    ESP_RETURN_ON_ERROR(
        getRawULP(),
        "DHT_ULP:get", ""
    );
    ESP_RETURN_ON_ERROR(
        decodeTime(raw_bytes),
        "DHT_ULP:get", ""
    );
    ESP_RETURN_ON_ERROR(
        decodeData(raw_bytes, result),
        "DHT_ULP:get", ""
    );
    return ESP_OK;
}

esp_err_t DHT::getRawULP() {
    ulp_status |= ULP_STATUS_DHT11_REQUESTED;
    while (!(ulp_status & ULP_STATUS_DHT11_COMPLETED)) {
        ESP_LOGD("DHT_ULP:getRaw", "Waiting... (status: 0x%lx)", ulp_status);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ulp_status &= ~(ULP_STATUS_DHT11_COMPLETED | ULP_STATUS_DHT11_REQUESTED);
    ESP_RETURN_ON_FALSE(
        ulp_err_code == ULP_SUCCESS, ESP_ERR_INVALID_RESPONSE,
        "DHT_ULP:getRaw", "Failed to get temperature/humidity. (Code: %lu)",
        ulp_err_code
    );
    if (esp_log_level_get("DHT_ULP:getRaw") > ESP_LOG_DEBUG) {
        for (int i = 0; i < 84; i++)
            ESP_LOGD("DHT_ULP:getRaw", "DHT11 Raw response[%2d] | LV: %d | Value: %u", i, !(i % 2), raw_times[i]);
    }
    return ESP_OK;
}

esp_err_t DHT::decodeTime(uint8_t *raw_bytes) {
    if (raw_times[0] < DHT_RESPONSE_START_SHORT_THRESHOLD || raw_times[0] > DHT_RESPONSE_START_LONG_THRESHOLD) {
        ESP_LOGW("DHT_ULP:decodeTime", "Response start time (%d) out of range.", (int)(raw_times[0] / ULP_CYCLES_PER_US));
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (raw_times[1] < DHT_ACK_RESPONSE_SHORT_THRESHOLD || raw_times[1] > DHT_ACK_RESPONSE_LONG_THRESHOLD) {
        ESP_LOGW("DHT_ULP:decodeTime", "Response low time (%d) out of range.", (int)(raw_times[1] / ULP_CYCLES_PER_US));
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (raw_times[2] < DHT_ACK_RESPONSE_SHORT_THRESHOLD || raw_times[2] > DHT_ACK_RESPONSE_LONG_THRESHOLD) {
        ESP_LOGW("DHT_ULP:decodeTime", "Response high time (%d) out of range.", (int)(raw_times[2] / ULP_CYCLES_PER_US));
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (int i = 0; i < 5; i++)
        raw_bytes[i] = 0;
    for (int index, byte = 0; byte < 5; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            index = (byte * 8 + 7 - bit) * 2 + 3;
            if (raw_times[index] < DHT_RESPONSE_L_SHORT_THRESHOLD || raw_times[index] > DHT_RESPONSE_L_LONG_THRESHOLD) {
                ESP_LOGW("DHT_ULP:decodeTime", "byte %d, bit %d: L time (%d) out of range.", byte, bit, (int)(raw_times[index] / ULP_CYCLES_PER_US));
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (raw_times[index + 1] > DHT_RESPONSE_0_SHORT_THRESHOLD && raw_times[index + 1] < DHT_RESPONSE_0_LONG_THRESHOLD) {
                // bit 0
            } else if (raw_times[index + 1] > DHT_RESPONSE_1_SHORT_THRESHOLD && raw_times[index + 1] < DHT_RESPONSE_1_LONG_THRESHOLD) {
                // bit 1
                raw_bytes[byte] |= 1 << bit;
            } else {
                ESP_LOGW("DHT_ULP:decodeTime", "byte %d, bit %d: H time (%d) out of range.", byte, bit, (int)(raw_times[index + 1] / ULP_CYCLES_PER_US));
                return ESP_ERR_INVALID_RESPONSE;
            }
        }
    }

    ESP_LOGD(
        "DHT_ULP:decodeTime", "Response: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
        raw_bytes[0], raw_bytes[1], raw_bytes[2], raw_bytes[3], raw_bytes[4]
    );

    return ESP_OK;
}

esp_err_t DHT::decodeData(uint8_t *raw_bytes, DHTResult *result) {
    // Checksum
    if ((uint8_t)(raw_bytes[0] + raw_bytes[1] + raw_bytes[2] + raw_bytes[3]) != raw_bytes[4]) {
        ESP_LOGW("DHT_ULP:decodeData", "CRC Error");
        return ESP_ERR_INVALID_CRC;
    }
    result->humidity = raw_bytes[0] + raw_bytes[1] * 0.1f;
    result->temperature = raw_bytes[2] + raw_bytes[3] * 0.1f;
    return ESP_OK;
}
