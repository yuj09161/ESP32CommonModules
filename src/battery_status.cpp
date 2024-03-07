#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_check.h"

#include "battery_status.h"


static const char *TAG = "BatteryStatus";

#define FULL_CHARGE_VOLTAGE    4200
#define FULL_DISCHARGE_VOLTAGE 3000

BatteryStatus::BatteryStatus(gpio_num_t adcPort, gpio_num_t chargeDetectPort, int samplePerMeasurement) {
    this->adcPort = adcPort;
    this->chargeDetectPort = chargeDetectPort;
    this->samplePerMeasurement = samplePerMeasurement;
}

BatteryStatus::~BatteryStatus() {
}

esp_err_t BatteryStatus::start() {
    assert(!isStarted);

    adc_unit_t adcUnit;
    ESP_RETURN_ON_ERROR(
        adc_continuous_io_to_channel(adcPort, &adcUnit, &adcChannel),
        TAG, ""
    );

    // ADC for battery voltage measurement
    adc_oneshot_unit_init_cfg_t unitConfig = {
        .unit_id = adcUnit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(&unitConfig, &adcHandle),
        TAG, "ADC unit creation failed."
    );
    adc_oneshot_chan_cfg_t channelConfig = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(adcHandle, adcChannel, &channelConfig),
        TAG, "ADC channel config failed."
    );
    adc_cali_line_fitting_config_t calibrationConfig = {
        .unit_id = adcUnit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(
        adc_cali_create_scheme_line_fitting(&calibrationConfig, &calibrationHandle),
        TAG, "ADC calibration handle creation error."
    );

    // Exernal power detection
    gpio_config_t chargeDetectionCfg = {
        .pin_bit_mask = (uint64_t)(1ll << chargeDetectPort),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&chargeDetectionCfg);

    isStarted = true;
    return ESP_OK;
}

esp_err_t BatteryStatus::stop() {
    assert(isStarted);
    isStarted = false;
    return ESP_OK;
}

bool BatteryStatus::isCharging() {
    assert(isStarted);
    return gpio_get_level(chargeDetectPort);
}

esp_err_t BatteryStatus::getBatteryStatus(int *batteryVoltage, int *percent) {
    assert(isStarted);

    int rawReading;
    long long rawReadingAvg = 0;
    for (int i = 0; i < samplePerMeasurement; i++) {
        ESP_RETURN_ON_ERROR(
            adc_oneshot_read(adcHandle, adcChannel, &rawReading),
            TAG, "Error on ADC reading."
        );
        rawReadingAvg += rawReading;
    }
    rawReadingAvg /= samplePerMeasurement;
    ESP_RETURN_ON_ERROR(
        rawToStatus(rawReadingAvg, batteryVoltage, percent),
        TAG, "Calculation failed."
    );
    return ESP_OK;
}

esp_err_t BatteryStatus::rawToStatus(int rawValue, int *batteryVoltage, int *percent) {
    assert(isStarted);

    int voltage;
    ESP_RETURN_ON_ERROR(
        adc_cali_raw_to_voltage(calibrationHandle, rawValue, &voltage),
        TAG, "Failed to calculate."
    );
    voltage = voltage * 2;
    *batteryVoltage = voltage;
    *percent = voltageToPercent(voltage);
    return ESP_OK;
}

int BatteryStatus::voltageToPercent(int batteryVoltage) {
    if (batteryVoltage > 4100) {
        return 100;
    } else if (batteryVoltage > 4060) {
        return 90;
    } else if (batteryVoltage > 3980) {
        return 80;
    } else if (batteryVoltage > 3920) {
        return 70;
    } else if (batteryVoltage > 3870) {
        return 60;
    } else if (batteryVoltage > 3820) {
        return 50;
    } else if (batteryVoltage > 3790) {
        return 40;
    } else if (batteryVoltage > 3770) {
        return 30;
    } else if (batteryVoltage > 3740) {
        return 20;
    } else if (batteryVoltage > 3680) {
        return 10;
    } else if (batteryVoltage > 3450) {
        return 5;
    } else {
        return 0;
    }
}
