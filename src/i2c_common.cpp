#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_check.h"

#include "i2c_common.h"


#define ADDR_TO_READ(x)  ((x) << 1 | 1)
#define ADDR_TO_WRITE(x) ((x) << 1)


I2CCommon::I2CCommon(i2c_port_t i2cNum, uint8_t deviceAddr, unsigned int timeout) {
    this->deviceAddr = deviceAddr;
    this->i2cNum = i2cNum;
    this->timeout = pdMS_TO_TICKS(timeout);
};

esp_err_t I2CCommon::read(uint8_t *result, unsigned int bytesCnt) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(
        cmd != NULL,
        ESP_ERR_NO_MEM, "I2CCommon:readWithoutAddr", "No sufficent memory to create cmd link."
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_start(cmd),
        "I2CCommon:readWithoutAddr", "Fail: start"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_write_byte(cmd, ADDR_TO_READ(deviceAddr), true),
        "I2CCommon:readWithoutAddr", "Fail: write device addr (for read)"
    );
    if (bytesCnt > 1)
        ESP_RETURN_ON_ERROR(
            i2c_master_read(cmd, result, bytesCnt - 1, I2C_MASTER_ACK),
            "I2CCommon:readWithoutAddr", "Fail: read bytes"
        );
    ESP_RETURN_ON_ERROR(
        i2c_master_read_byte(cmd, result + bytesCnt - 1, I2C_MASTER_NACK),
        "I2CCommon:readWithoutAddr", "Fail: read last byte"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_stop(cmd),
        "I2CCommon:readWithoutAddr", "Fail: stop"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_cmd_begin(i2cNum, cmd, timeout),
        "I2CCommon:readWithoutAddr", "Fail: cmd_begin"
    );
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t I2CCommon::read(uint8_t regAddr, uint8_t *result, unsigned int bytesCnt) {
    ESP_RETURN_ON_ERROR(
        setAddr(regAddr),
        "I2CCommon:read", "Fail: set register addr"
    );
    ESP_RETURN_ON_ERROR(
        read(result, bytesCnt),
        "I2CCommon:read", "Fail: read data"
    );
    return ESP_OK;
}

esp_err_t I2CCommon::setAddr(uint8_t regAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(
        cmd != NULL,
        ESP_ERR_NO_MEM, "I2CCommon:read", "No sufficent memory to create link."
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_start(cmd),
        "I2CCommon:write", "Fail: start"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_write_byte(cmd, ADDR_TO_WRITE(deviceAddr), true),
        "I2CCommon:write", "Fail: write device addr"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_write_byte(cmd, regAddr, true),
        "I2CCommon:write", "Fail: write register address"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_stop(cmd),
        "I2CCommon:write", "Fail: stop"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_cmd_begin(i2cNum, cmd, timeout),
        "I2CCommon:write", "Fail: cmd_begin"
    );
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t I2CCommon::write(uint8_t regAddr, uint8_t *data, unsigned int bytesCnt) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(
        cmd != NULL,
        ESP_ERR_NO_MEM, "I2CCommon:read", "No sufficent memory to create link."
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_start(cmd),
        "I2CCommon:write", "Fail: start"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_write_byte(cmd, ADDR_TO_WRITE(deviceAddr), true),
        "I2CCommon:write", "Fail: write device addr"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_write_byte(cmd, regAddr, true),
        "I2CCommon:write", "Fail: write register address"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_write(cmd, data, bytesCnt, true),
        "I2CCommon:write", "Fail: write data"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_stop(cmd),
        "I2CCommon:write", "Fail: stop"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_cmd_begin(i2cNum, cmd, timeout),
        "I2CCommon:write", "Fail: cmd_begin"
    );
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t I2CCommon::writeAlwaysWithAddr(uint8_t regAddr, uint8_t *data, unsigned int bytesCnt) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(
        cmd != NULL,
        ESP_ERR_NO_MEM, "I2CCommon:read", "No sufficent memory to create link."
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_start(cmd),
        "I2CCommon:write", "Fail: start"
    );

    ESP_RETURN_ON_ERROR(
        i2c_master_write_byte(cmd, ADDR_TO_WRITE(deviceAddr), true),
        "I2CCommon:write", "Fail: write device addr"
    );
    for (int i = 0; i < bytesCnt; i++) {
        ESP_RETURN_ON_ERROR(
            i2c_master_write_byte(cmd, regAddr + i, true),
            "I2CCommon:write", "Fail: write addr #%d", i
        );
        ESP_RETURN_ON_ERROR(
            i2c_master_write_byte(cmd, data[i], true),
            "I2CCommon:write", "Fail: write data #%d", i
        );
    }

    ESP_RETURN_ON_ERROR(
        i2c_master_stop(cmd),
        "I2CCommon:write", "Fail: stop"
    );
    ESP_RETURN_ON_ERROR(
        i2c_master_cmd_begin(i2cNum, cmd, timeout),
        "I2CCommon:write", "Fail: cmd_begin"
    );
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}
