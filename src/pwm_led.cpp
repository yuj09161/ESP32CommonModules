#include <cstdlib>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "pwm_led.h"


static const char *TAG = "PWMLed";

PWMLed::PWMLed(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel) {
    this->pin = pin;
    this->timer = timer;
    this->channel = channel;
}

PWMLed::~PWMLed() {
    if (this->began) {
        ESP_LOGW(TAG, "Deconstructer called after begin(), before end()");
        abort();
    }
}

esp_err_t PWMLed::begin(bool enableWhenLightSleep) {
    ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode       = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution  = LEDC_TIMER_8_BIT;
        timer_cfg.timer_num        = timer;
        timer_cfg.freq_hz          = 5000;
    if (enableWhenLightSleep)
        timer_cfg.clk_cfg = LEDC_USE_RC_FAST_CLK;
    else
        timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    ledc_channel_config_t channel_cfg = {};
        channel_cfg.gpio_num   = pin;
        channel_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_cfg.channel    = channel;
        channel_cfg.intr_type  = LEDC_INTR_DISABLE;
        channel_cfg.timer_sel  = timer;
        channel_cfg.duty       = 0;
        channel_cfg.hpoint     = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    this->began = true;
    return ESP_OK;
}

esp_err_t PWMLed::end() {
    ESP_RETURN_ON_ERROR(
        ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0),
        TAG, "Failed to stop LEDC."
    );
    ESP_RETURN_ON_ERROR(
        ledc_timer_pause(LEDC_LOW_SPEED_MODE, timer),
        TAG, "Failed to pause LEDC timer."
    );
    ledc_timer_config_t timer_cfg;
        timer_cfg.speed_mode  = LEDC_LOW_SPEED_MODE;
        timer_cfg.timer_num   = timer;
        timer_cfg.deconfigure = true;
    ESP_RETURN_ON_ERROR(
        ledc_timer_config(&timer_cfg),
        TAG, "Failed to release LEDC timer."
    );
    this->began = false;
    return ESP_OK;
}

esp_err_t PWMLed::on() {
    if (isOn)
        return ESP_OK;
    isOn = true;
    return refresh();
}

esp_err_t PWMLed::off() {
    if (!isOn)
        return ESP_OK;
    isOn = false;
    return refresh();
}

esp_err_t PWMLed::toggle() {
    isOn = !isOn;
    return refresh();
}

esp_err_t PWMLed::setBright(uint8_t bright) {
    if (this->bright == bright)
        return ESP_OK;
    this->bright = bright;
    if (!isOn)
        return ESP_OK;
    return refresh();
}

esp_err_t PWMLed::refresh() {
    ESP_LOGD(TAG, "Set bright to %d/256", isOn * bright);
    ESP_RETURN_ON_ERROR(
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, isOn * bright),
        TAG, "Failed to set duty cycle."
    );
    ESP_RETURN_ON_ERROR(
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel),
        TAG, "Failed to update duty cycle."
    );
    ESP_LOGD(TAG, "Successfully updated.");
    return ESP_OK;
}
