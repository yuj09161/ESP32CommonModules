#ifndef __LM35_MODULE_H_INCLUDED__
#define __LM35_MODULE_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"

typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;
    int sample_per_measurement;
} lm35_config_t;

typedef struct lm35_handle {
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t channel;
    adc_cali_handle_t cali_handle;
    int sample_per_measurement;
} *lm35_handle_t;

esp_err_t lm35_create(lm35_config_t *config, lm35_handle_t *new_handle);
esp_err_t lm35_delete(lm35_handle_t handle);
esp_err_t lm35_get_value(lm35_handle_t handle, float *result);

#ifdef __cplusplus
}
#endif

#endif