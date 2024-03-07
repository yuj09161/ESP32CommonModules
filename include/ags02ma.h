#ifndef __AGS02MA_MODULE_H_INCLUDED__
#define __AGS02MA_MODULE_H_INCLUDED__

#include <stdint.h>

#include "freertos/portmacro.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "i2c_common.h"


class AGS02MA : I2CCommon {
    public:
        AGS02MA(i2c_port_t i2c_num, uint8_t address, unsigned int timeout);
        esp_err_t begin();
        bool isSensorPreHeated();
        bool isTimeLimitPassed();
        esp_err_t get(int *result);
    private:
        typedef enum {
            UNIT_PPB = 0x00,
            UNIT_UG_M_3 = 0x02,
            UNIT_UNKNOWN = 0xff
        } AGS02MA_UNIT;

        TickType_t lastTick = 0;
        uint8_t buf[5];

        uint8_t crc(uint8_t *bytes, int size);
};

#endif
