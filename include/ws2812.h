#ifndef __VINDRIKTNING_WS2812_H_INCLUDED__
#define __VINDRIKTNING_WS2812_H_INCLUDED__

#include <stdint.h>

#include "driver/gpio.h"

#include "led_strip.h"


typedef union {
    struct {
        uint8_t bright;
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };
    uint32_t raw;
} WS2812Pixel;

const WS2812Pixel WS2812_OFF    = {.bright = 0, .r =   0, .g =   0, .b =   0};
const WS2812Pixel WS2812_RED    = {.bright = 0, .r = 255, .g =   0, .b =   0};
const WS2812Pixel WS2812_GREEN  = {.bright = 0, .r =   0, .g = 255, .b =   0};
const WS2812Pixel WS2812_BLUE   = {.bright = 0, .r =   0, .g =   0, .b = 255};
const WS2812Pixel WS2812_YELLOW = {.bright = 0, .r = 255, .g = 255, .b =   0};
const WS2812Pixel WS2812_ORANGE = {.bright = 0, .r = 255, .g = 165, .b =   0};
const WS2812Pixel WS2812_WHITE  = {.bright = 0, .r = 255, .g = 255, .b = 255};

class WS2812Strip {
    public:
        WS2812Strip(gpio_num_t pin, unsigned int numPixels);
        ~WS2812Strip();
        esp_err_t begin();
        esp_err_t end();
        esp_err_t setColors(WS2812Pixel *colors, bool refresh = true);
        esp_err_t setColor(int pixel, WS2812Pixel color, bool refresh = true);
        esp_err_t setBright(int bright, bool refresh = true);
        esp_err_t refresh(bool check = true);
    private:
        gpio_num_t pin;
        unsigned int numPixels;
        led_strip_handle_t handle;
        SemaphoreHandle_t pixelsSemaphore, handleMutex;
        unsigned int countToWritePixel;
        bool began = false;

        WS2812Pixel *pixels;
        WS2812Pixel *privPixels;

        esp_err_t _refresh(int startPixel);
};

#endif
