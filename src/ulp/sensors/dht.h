#ifndef __ULP_SENSORS_DHT_H_INCLUDED__
#define __ULP_SENSORS_DHT_H_INCLUDED__

#include <stdint.h>

#include "ulp_riscv.h"

#include "../ulp_defs.h"

#define TIMEOUT (500 * ULP_RISCV_CYCLES_PER_US)

ErrCodes dht_read_raw(gpio_num_t pin, uint16_t *raw_values);

#endif