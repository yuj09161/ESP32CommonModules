#ifndef __VINDRIKTNING_PWM_LED_H_INCLUDED__
#define __VINDRIKTNING_PWM_LED_H_INCLUDED__

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

class PWMLed {
    public:
        PWMLed(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel);
        ~PWMLed();
        esp_err_t begin(bool enableWhenLightSleep = false);
        esp_err_t end();

        esp_err_t on();
        esp_err_t off();
        esp_err_t toggle();
        esp_err_t setBright(uint8_t bright);
    private:
        gpio_num_t pin;
        ledc_timer_t timer;
        ledc_channel_t channel;
        bool began = false;
        bool isOn = false;
        uint_fast8_t bright = 0;

        esp_err_t refresh();
};

#endif
