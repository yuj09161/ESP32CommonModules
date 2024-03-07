#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_check.h"

#include "lm35.h"

esp_err_t lm35_create(lm35_config_t *config, lm35_handle_t *new_handle) {
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;

    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {
        .unit_id = config->unit,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_chan_cfg_t adc_cfg = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = config->unit,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_cfg, &adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, config->channel, &adc_cfg));
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    lm35_handle_t handle = malloc(sizeof(lm35_handle_t));
    handle->adc_handle  = adc_handle;
    handle->channel     = config->channel;
    handle->cali_handle = cali_handle;

    *new_handle = handle;
    return ESP_OK;
}

esp_err_t lm35_delete(lm35_handle_t handle) {
    ESP_ERROR_CHECK(adc_oneshot_del_unit(handle->adc_handle));
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle->cali_handle));
    free(handle);
    return ESP_OK;
}

esp_err_t lm35_get_value(lm35_handle_t handle, float *result) {
    int raw_value, millivolt, average = 0;
    for (int i = 0; i < handle->sample_per_measurement; i++) {
        ESP_RETURN_ON_ERROR(
            adc_oneshot_read(handle->adc_handle, handle->channel, &raw_value),
            "LM35:get_value", "Reading value failed."
        );
        // millivolt = raw_value * 1100 / (1 << 12);
        ESP_RETURN_ON_ERROR(
            adc_cali_raw_to_voltage(handle->cali_handle, raw_value, &millivolt),
            "LM35:get_value", "Calibrating value failed."
        );
        average += millivolt;
    }
    average /= handle->sample_per_measurement;
    *result = average / 10.0f;

    ESP_LOGI("LM35:get_value", "Temperature: %.1f°C", *result);
    return ESP_OK;
}

