#ifndef __I2C_MODULE_COMMON_H_INCLUDED__
#define __I2C_MODULE_COMMON_H_INCLUDED__

#include <cstdint>

#include "freertos/portmacro.h"
#include "driver/i2c.h"
#include "esp_err.h"

class I2CCommon {
    public:
        I2CCommon(i2c_port_t i2c_num, uint8_t deviceAddr, unsigned int timeout);

    protected:
        esp_err_t setAddr(uint8_t regAddr);
        esp_err_t read(uint8_t *result, unsigned int bytesCnt);
        esp_err_t read(uint8_t regAddr, uint8_t *result, unsigned int bytesCnt);
        esp_err_t write(uint8_t regAddr, uint8_t *data, unsigned int bytesCnt);
        esp_err_t writeAlwaysWithAddr(uint8_t regAddr, uint8_t *data, unsigned int bytesCnt);

    private:
        uint8_t deviceAddr;
        i2c_port_t i2cNum;
        TickType_t timeout;
};

#endif
