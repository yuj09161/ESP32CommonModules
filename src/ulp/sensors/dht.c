#include <stdint.h>

#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "../ulp_defs.h"
#include "dht.h"


ErrCodes dht_read_raw(gpio_num_t pin, uint16_t *raw_values) {
    // Configure GPIO
    ulp_riscv_gpio_init(pin);
    ulp_riscv_gpio_input_enable(pin);
    ulp_riscv_gpio_output_enable(pin);
    ulp_riscv_gpio_set_output_mode(pin, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup(pin);
    ulp_riscv_gpio_pulldown_disable(pin);

    // Write request
    ulp_riscv_gpio_output_level(pin, 0);
    ulp_riscv_delay_cycles(20 * ULP_RISCV_CYCLES_PER_MS);
    ulp_riscv_gpio_output_level(pin, 1);

    // Read Signals
    uint32_t priv_cycle = ULP_RISCV_GET_CCOUNT();
    uint_fast8_t priv_level = 1;
    uint16_t raw_times[84] = {};
    for (int i = 0; i < 84; i++) {
        while (ulp_riscv_gpio_get_level(pin) == priv_level) {
            if (ULP_RISCV_GET_CCOUNT() - priv_cycle > TIMEOUT)
                return ULP_ERR_TIMEOUT;
        }
        priv_level ^= 1;
        raw_values[i] = raw_times[i] = ULP_RISCV_GET_CCOUNT() - priv_cycle;
        priv_cycle = ULP_RISCV_GET_CCOUNT();
    }
    return ULP_SUCCESS;
}
