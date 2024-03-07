#ifndef __VINDRIKTNING_IO_H_INCLUDED__
#define __VINDRIKTNING_IO_H_INCLUDED__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fine_dust;
    float temperature;
    int humidity;
    float temperature2;
    float pressure;
    int tvoc;
    int illuminance;
} sensor_values;

void init_modules(void);
void init_gpio(void);

esp_err_t get_sensor_values(sensor_values *result);

void led_off(void);
void led_on(void);
void led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif