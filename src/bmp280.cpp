#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_check.h"

#include "i2c_common.h"
#include "bmp280.h"

#define REG_CTRL_ADDR 0xf4
#define REG_CTRL_SIZE 2
#define REG_RESULT_ADDR 0xf7
#define REG_RESULT_SIZE 6
#define REG_CALIBRATION_ADDR 0x88
#define REG_CALIBRATION_SIZE 24
#define REG_CHIP_ID_ADDR 0xd0


BMP280::BMP280(
    i2c_port_t i2c_num,
    uint8_t address,
    unsigned int timeout
) : I2CCommon(i2c_num, address, timeout) {}

esp_err_t BMP280::begin(
    PowerMode powerMode,
    PressureResolution pressureResolution,
    TemperatureResolution temperatureResolution,
    FilterCoefficent filterCoefficient,
    StandbyTime standbyTime
) {
    ESP_RETURN_ON_FALSE(
        !(this->began), ESP_ERR_INVALID_STATE,
        "BMP280:init", "Sensor already began."
    );
    
    ESP_RETURN_ON_ERROR(
        configure(powerMode, pressureResolution, temperatureResolution, filterCoefficient, standbyTime),
        "BMP280:init", "Failed to configure sensor."
    );
    ESP_LOGI("BMP280:init", "Successfully configured sensor.");

    ESP_RETURN_ON_ERROR(
        readCalibrationData(),
        "BMP280:init", "Failed to read calibration data."
    );
    ESP_LOGI("BMP280:init", "Successfully read calibration data.");

    uint8_t chipId;
    ESP_RETURN_ON_ERROR(
        read(REG_CHIP_ID_ADDR, &chipId, 1),
        "BMP280:init", "Failed to read ID."
    );
    ESP_RETURN_ON_FALSE(
        chipId == 0x58, ESP_ERR_INVALID_RESPONSE,
        "BMP280:init", "Invalid chip id: 0x%02x", chipId
    );

    this->began = true;
    return ESP_OK;
}

esp_err_t BMP280::get(BMP280Result *result) {
    int32_t raw_result[2];
    ESP_RETURN_ON_ERROR(
        readValue(raw_result),
        "BMP280:get", "Failed to get value."
    );
    calculate(raw_result, result);
    return ESP_OK;
}

esp_err_t BMP280::configure(
    PowerMode powerMode,
    PressureResolution pressureResolution,
    TemperatureResolution temperatureResolution,
    FilterCoefficent filterCoefficient,
    StandbyTime standbyTime
) {
    uint8_t data[2] = {
        (uint8_t)(temperatureResolution << 5 | pressureResolution << 2 | powerMode),
        (uint8_t)(standbyTime << 5 | filterCoefficient << 2)
    };
    ESP_RETURN_ON_ERROR(
        writeAlwaysWithAddr(REG_CTRL_ADDR, data, REG_CTRL_SIZE),
        "BMP280:configure", "Write error."
    );
    return ESP_OK;
}

esp_err_t BMP280::readCalibrationData() {
    uint8_t buf[REG_CALIBRATION_SIZE];
    ESP_RETURN_ON_ERROR(
        read(REG_CALIBRATION_ADDR, buf, REG_CALIBRATION_SIZE),
        "BMP280:readCalibrationData", "Read error."
    );

    for (int i = 0; i < REG_CALIBRATION_SIZE / 2; i++) {
        calibration.raw[i] = ((uint16_t)buf[i * 2 + 1] << 8) + buf[i * 2];
        ESP_LOGD("BMP280:readCalibrationData", "0x%02X: 0x%04x", REG_CALIBRATION_ADDR + i * 2, calibration.raw[i]);
    }
    ESP_LOGD("BMP280:readCalibrationData", "Calibration T1: 0x%02x", calibration.T1);

    return ESP_OK;
}

esp_err_t BMP280::readValue(int32_t *result) {
    uint8_t buf[REG_RESULT_SIZE];
    ESP_RETURN_ON_ERROR(
        read(REG_RESULT_ADDR, buf, REG_RESULT_SIZE),
        "BMP280:readValue", "R/W error."
    );

    result[0] = ((int32_t)buf[3] << 12) + ((int32_t)buf[4] << 4) + (buf[5] >> 4);
    result[1] = ((int32_t)buf[0] << 12) + ((int32_t)buf[1] << 4) + (buf[2] >> 4);

    ESP_LOGD("BMP280:readValue", "Raw Temp: 0x%08lx", result[0]);
    ESP_LOGD("BMP280:readValue", "Raw Press: 0x%08lx", result[1]);

    return ESP_OK;
}

void BMP280::calculate(int32_t *raw_values, BMP280Result *result) {
    int32_t tmp_t, temp_res;
    tmp_t = (raw_values[0] >> 4) - calibration.T1;
    temp_res = (
        ( ((raw_values[0] >> 3) - ((int32_t)calibration.T1 << 1)) * calibration.T2 ) >> 11
    ) + (
        ( ((tmp_t * tmp_t) >> 12) * calibration.T3 ) >> 14
    );
    result->temperature = ((temp_res * 5 + 128) >> 8) / 100.0f; // (Calculated)/100->Celsius

    int64_t tmp_p1, tmp_p2, tmp_p3;
    tmp_p1 = (int64_t)temp_res - 128000;
    tmp_p3 = tmp_p1 * tmp_p1;
    tmp_p2 = (
        (tmp_p3 * (int64_t)calibration.P6)
        + ((tmp_p1 * (int64_t)calibration.P5) << 17)
        + ((int64_t)calibration.P4 << 35)
    );
    tmp_p1 = (
        ((tmp_p3 * (int64_t)calibration.P3) >> 8)
        + ((tmp_p1 * (int64_t)calibration.P2) << 12)
    );
    tmp_p1 = ((tmp_p1 + (1ll << 47)) * (int64_t)calibration.P1) >> 33;
    if (tmp_p1 == 0) {
        result->pressure = 0;
    } else {
        tmp_p3 = (((int64_t)(1048576 - raw_values[1]) << 31) - tmp_p2) * 3125 / tmp_p1;
        tmp_p2 = tmp_p3 >> 13;
        tmp_p1 = ((int64_t)calibration.P9 * tmp_p2 * tmp_p2) >> 25;
        tmp_p2 = ((int64_t)calibration.P8 * tmp_p3) >> 19;
        tmp_p3 = ((tmp_p1 + tmp_p2 + tmp_p3) >> 8) + ((int64_t)calibration.P7 << 4);
        result->pressure = (uint32_t)tmp_p3 / 25600.0f; // (Calculated)/256->Pa, Pa/100->hPa
    }
}
