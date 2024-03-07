#include "ulp_riscv_gpio.h"

#include "ulp_defs.h"
#include "sensors/dht.h"


ULPStatus status;
ErrCodes err_code;
gpio_num_t pin;
uint16_t raw_values[84];


int main (void) {
    if (status & ULP_STATUS_DHT11_REQUESTED) {
        status &= ~ULP_STATUS_DHT11_REQUESTED;
        err_code = dht_read_raw(pin, raw_values);
        status |= ULP_STATUS_DHT11_COMPLETED;
    }
    return 0;
}
