#ifndef __BATTERY_STATUS_H_INCLUDED__
#define __BATTERY_STATUS_H_INCLUDED__

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class BatteryStatus {
    public:
        BatteryStatus(gpio_num_t adcPort, gpio_num_t chargeDetectPort, int samplePerMeasurement);
        ~BatteryStatus();
        esp_err_t start();
        esp_err_t stop();
        bool isCharging();
        esp_err_t getBatteryStatus(int *batteryVoltage, int *percent);
        esp_err_t rawToStatus(int rawValue, int *batteryVoltage, int *percent);
    private:
        bool isStarted = false;

        gpio_num_t adcPort, chargeDetectPort;
        int samplePerMeasurement;

        adc_channel_t adcChannel;
        adc_oneshot_unit_handle_t adcHandle;
        adc_cali_handle_t calibrationHandle;

        int voltageToPercent(int batteryVoltage);
};

#endif
