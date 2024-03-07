#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "rom/crc.h"
#include "esp_err.h"
#include "esp_check.h"

#include "i2c_common.h"
#include "ags02ma.h"

#define REG_FORMAT_ADDR 0x00
#define REG_RESULT_ADDR 0x00

#define UNIT_MASK 0x0e
#define NOT_READY_MASK 0x01


AGS02MA::AGS02MA(
    i2c_port_t i2c_num, uint8_t address, unsigned int timeout
) : I2CCommon(i2c_num, address, timeout) {
}

esp_err_t AGS02MA::begin() {
    ESP_RETURN_ON_FALSE(
        this->lastTick == 0, ESP_ERR_INVALID_STATE,
        "AGS02MA:init", "Already called begin."
    );

    buf[0] = buf[2] = UNIT_PPB;
    buf[1] = buf[3] = ~UNIT_PPB;
    buf[4] = crc(buf, 4);
    ESP_RETURN_ON_ERROR(
        write(REG_FORMAT_ADDR, buf, 5),
        "AGS02MA:get", "Failed to set unit."
    );

    // 2000ms (1500ms + 500ms) delay after set unit
    this->lastTick = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    return ESP_OK;
}

bool AGS02MA::isSensorPreHeated() {
    // Sensor preheat time after power on (=mcu starts)
    return xTaskGetTickCount() > pdMS_TO_TICKS(120000);
}

bool AGS02MA::isTimeLimitPassed() {
    return (
        this->lastTick != 0  // Sensor began
        && xTaskGetTickCount() - this->lastTick >= pdMS_TO_TICKS(1500)  // Interval between request
    );
}

esp_err_t AGS02MA::get(int *result) {
    ESP_RETURN_ON_FALSE(
        isSensorPreHeated(), ESP_ERR_INVALID_STATE,
        "AGS02MA:get", "The sensor needs preheat 120s after power on."
    );
    ESP_RETURN_ON_FALSE(
        isTimeLimitPassed(), ESP_ERR_NOT_FINISHED,
        "AGS02MA:get", "The sensor needs 1500ms delay between sample, and 2000ms startup delay."
    );

    ESP_RETURN_ON_ERROR(
        read(REG_RESULT_ADDR, buf, 5),
        "AGS02MA:get", "Failed to get value."
    );
    ESP_LOGD(
        "AGS02MA:get", "Raw: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
        buf[0], buf[1], buf[2], buf[3], buf[4]
    );

    ESP_RETURN_ON_FALSE(
        (buf[0] & NOT_READY_MASK) == 0, ESP_ERR_NOT_FINISHED,
        "AGS02MA:get", "Result is not available yet."
    );
    ESP_RETURN_ON_FALSE(
        (buf[0] & UNIT_MASK) == UNIT_PPB, ESP_ERR_NOT_SUPPORTED,
        "AGS02MA:get", "Got unsupported unit 0x%02x.", (buf[0] & UNIT_MASK) >> 1
    );

    ESP_RETURN_ON_FALSE(
        crc(buf, 4) == buf[4], ESP_ERR_INVALID_CRC,
        "AGS02MA:get", "CRC Mismatch."
    );

    *result = (((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3]); // Unit: ppb
    return ESP_OK;
}

uint8_t AGS02MA::crc(uint8_t *bytes, int size) {
    uint8_t crc = 0xff;
    for (int i = 0; i < size; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
