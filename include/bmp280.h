#ifndef __BMP280_MODULE_H_INCLUDED__
#define __BMP280_MODULE_H_INCLUDED__

#include <cstdint>

#include "freertos/portmacro.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "i2c_common.h"


typedef struct {
    float temperature;
    float pressure;
} BMP280Result;

class BMP280 : I2CCommon {
    public:
        enum PowerMode {PM_SLEEP, PM_FORCED, PM_NORMAL=3};
        enum PressureResolution {SKIP_PRESS, P_ULOW, P_LOW, P_STANDARD, P_HIGH, P_UHIGH};
        enum TemperatureResolution {SKIP_TEMP, T_LOW, T_STANDARD, T_HIGH, T_UHIGH, T_UUHIGH};
        enum FilterCoefficent {FILTER_OFF, COEFF_2X, COEFF_4X, COEFF_8X, COEFF16X};
        enum StandbyTime {STBY_500us, STBY_62_5ms, STBY_125ms, STBY_250ms, STBY_500ms, STBY_1s, STBY_2s, STBY_4s};

        BMP280(
            i2c_port_t i2c_num,
            uint8_t address,
            unsigned int timeout
        );
        esp_err_t begin(
            PowerMode powerMode,
            PressureResolution pressureResolution,
            TemperatureResolution temperatureResolution,
            FilterCoefficent filterCoefficient,
            StandbyTime standbyTime
        );
        esp_err_t get(BMP280Result *result);
        esp_err_t configure(
            PowerMode powerMode,
            PressureResolution pressureResolution,
            TemperatureResolution temperatureResolution,
            FilterCoefficent filterCoefficient,
            StandbyTime standbyTime
        );
    private:
        typedef union {
            struct {
                uint16_t T1;
                int16_t T2;
                int16_t T3;
                uint16_t P1;
                int16_t P2;
                int16_t P3;
                int16_t P4;
                int16_t P5;
                int16_t P6;
                int16_t P7;
                int16_t P8;
                int16_t P9;
            };
            uint16_t raw[12];
        } BMP280CalibrationValues;

        BMP280CalibrationValues calibration;

        bool began = false;

        esp_err_t readCalibrationData();

        esp_err_t readValue(int32_t *result);
        void calculate(int32_t *raw_values, BMP280Result *result);
        uint32_t tmpCalculatePressure(int32_t t_fine, int32_t adc_P);
};

#endif
