#ifndef __DHT_MODULE_H_INCLUDED__
#define __DHT_MODULE_H_INCLUDED__

#include <cstdint>

#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "driver/rmt_types.h"

typedef struct {
    float temperature;
    float humidity;
} DHTResult;

class DHT {
    public:
        DHT(gpio_num_t pin);
        esp_err_t get(DHTResult *result);
    private:
        uint32_t *pin;
        uint16_t *raw_times;

        esp_err_t getRawULP();
        esp_err_t decodeTime(uint8_t *raw_bytes);
        esp_err_t decodeData(uint8_t *raw_bytes, DHTResult *result);
};

#endif
