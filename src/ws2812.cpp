#include <cstdlib>
#include <cstring>
#include <cassert>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "led_strip.h"

#include "utils/semaphore.h"
#include "ws2812.h"


#define SEMAPHORE_MAX_WAIT pdMS_TO_TICKS(5000)
#define COUNT_TO_WRITE_PIXEL 3
static const char *TAG = "WS2812";


WS2812Strip::WS2812Strip(gpio_num_t pin, unsigned int numPixels) {
    this->pin = pin;
    this->numPixels = numPixels;
    this->pixels = (WS2812Pixel *)calloc(numPixels, sizeof(WS2812Pixel));
    this->privPixels = (WS2812Pixel *)calloc(numPixels, sizeof(WS2812Pixel));
    pixelsSemaphore = xSemaphoreCreateCounting(COUNT_TO_WRITE_PIXEL, COUNT_TO_WRITE_PIXEL);
    handleMutex = xSemaphoreCreateMutex();
}

WS2812Strip::~WS2812Strip() {
    if (this->began) {
        ESP_LOGW(TAG, "Deconstructer called after begin(), before end()");
        abort();
    }
    free(pixels);
    free(privPixels);
}

esp_err_t WS2812Strip::begin() {
    ESP_LOGI(TAG, "Initializing WS2812...");
    led_strip_config_t config = {};
        config.strip_gpio_num   = pin;
        config.max_leds         = 8;
        config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
        config.led_model        = LED_MODEL_WS2812;
        config.flags.invert_out = false;
    led_strip_rmt_config_t rmt_config = {};
        rmt_config.clk_src           = RMT_CLK_SRC_DEFAULT;
        rmt_config.resolution_hz     = 10000000;  // 1M
        rmt_config.mem_block_symbols = 0;
        rmt_config.flags.with_dma    = 0;
    ESP_RETURN_ON_ERROR(
        led_strip_new_rmt_device(&config, &rmt_config, &handle),
        TAG, "Failed to init WS2812"
    );
    began = true;
    return ESP_OK;
}

esp_err_t WS2812Strip::end() {
    ESP_RETURN_ON_FALSE(began, ESP_ERR_INVALID_STATE, TAG, "begin() is not called");
    ESP_RETURN_ON_ERROR(led_strip_del(handle), TAG, "Failed to delete WS2812");
    this->began = false;
    return ESP_OK;
}

esp_err_t WS2812Strip::setColors(WS2812Pixel *colors, bool refresh) {
    ESP_RETURN_ON_FALSE(began, ESP_ERR_INVALID_STATE, TAG, "begin() is not called");

    bool isSemaphoreTakeForWrite = false;
    if (!xSemaphoreTake(pixelsSemaphore, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }

    for (int p = 0; p < numPixels; p++) {
        if (
            pixels[p].r != colors[p].r
            || pixels[p].g != colors[p].g
            || pixels[p].b != colors[p].b
        ) {
            if (!isSemaphoreTakeForWrite) {
                if (xSemaphoreTakeN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL - 1, SEMAPHORE_MAX_WAIT)) {
                    isSemaphoreTakeForWrite = true;
                } else {
                    ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
                    abort();
                }
            }
            pixels[p].r = colors[p].r;
            pixels[p].g = colors[p].g;
            pixels[p].b = colors[p].b;
        }
    }

    if (isSemaphoreTakeForWrite) {
        if (!xSemaphoreGiveN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL)) {
            ESP_LOGE(TAG, "Failed to release semaphore.");
            abort();
        }
    }
    else {
        if (!xSemaphoreGive(pixelsSemaphore)) {
            ESP_LOGE(TAG, "Failed to release semaphore.");
            abort();
        }
    }

    if (refresh)
        return this->refresh();
    return ESP_OK;
}

esp_err_t WS2812Strip::setColor(int pixel, WS2812Pixel color, bool refresh) {
    ESP_RETURN_ON_FALSE(began, ESP_ERR_INVALID_STATE, TAG, "begin() is not called");

    if (!xSemaphoreTake(pixelsSemaphore, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }

    if (
        pixels[pixel].r != color.r
        || pixels[pixel].g != color.g
        || pixels[pixel].b != color.b
    ) {
        if (!xSemaphoreTakeN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL - 1, SEMAPHORE_MAX_WAIT)) {
            ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
            abort();
        }
        pixels[pixel].r = color.r;
        pixels[pixel].g = color.g;
        pixels[pixel].b = color.b;
        if (!xSemaphoreGiveN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL - 1)) {
            ESP_LOGE(TAG, "Failed to release semaphore.");
            abort();
        }
    }

    if (!xSemaphoreGive(pixelsSemaphore)) {
        ESP_LOGE(TAG, "Failed to release semaphore.");
        abort();
    }

    if (refresh)
        return this->refresh();
    return ESP_OK;
}

esp_err_t WS2812Strip::setBright(int bright, bool refresh) {
    static int privBright = 0;
    ESP_RETURN_ON_FALSE(began, ESP_ERR_INVALID_STATE, TAG, "begin() is not called");

    if (!xSemaphoreTake(pixelsSemaphore, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }

    if (bright == privBright) {
        if (!xSemaphoreGive(pixelsSemaphore)) {
        ESP_LOGE(TAG, "Failed to release semaphore.");
        abort();
    }
        return ESP_OK;
    }
    if (!xSemaphoreTakeN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL - 1, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }
    ESP_RETURN_ON_FALSE(
        bright >= 0 && bright <= 7, ESP_ERR_INVALID_ARG,
        TAG, "Bright out of range (must be one of 0-7)."
    );
    for (int i = 0; i < numPixels; i++)
        pixels[i].bright = bright;
    privBright = bright;
    if (!xSemaphoreGiveN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL)) {
        ESP_LOGE(TAG, "Failed to release semaphore.");
        abort();
    }

    if (refresh)
        return this->refresh(false);

    return ESP_OK;
}

esp_err_t WS2812Strip::refresh(bool check) {
    ESP_RETURN_ON_FALSE(
        began, ESP_ERR_INVALID_STATE,
        TAG, "begin() is not called"
    );
    ESP_LOGD(TAG, "Start update WS2812...");
    static bool isUpdateNeeded = !check;

    if (!xSemaphoreTake(pixelsSemaphore, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }

    int p;
    for (p = 0; p < 8 && !isUpdateNeeded; p++) {
        ESP_LOGD(
            TAG, "Bright: %d, Color: #%x%x%x",
            pixels[p].bright, pixels[p].r, pixels[p].g, pixels[p].b
        );
        if (pixels[p].raw != privPixels[p].raw)
            isUpdateNeeded = true;
    }

    if (isUpdateNeeded) {
        if (!xSemaphoreTakeN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL - 1, SEMAPHORE_MAX_WAIT)) {
            ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
            abort();
        }
        memcpy(privPixels + p, pixels + p, sizeof(WS2812Pixel) * (7 - p));
        if (!xSemaphoreGiveN(pixelsSemaphore, COUNT_TO_WRITE_PIXEL)) {
            ESP_LOGE(TAG, "Failed to release semaphore.");
            abort();
        }
        ESP_RETURN_ON_ERROR(_refresh(p), TAG, "Failed to refresh strip.");
    } else {
        if (!xSemaphoreGive(pixelsSemaphore)) {
            ESP_LOGE(TAG, "Failed to release semaphore.");
            abort();
        }
    }

    ESP_LOGD(TAG, "Done update WS2812.");
    return ESP_OK;
}

esp_err_t WS2812Strip::_refresh(int startPixel) {
    ESP_LOGD(TAG, "Starting refresh pixels...");
    WS2812Pixel pixel;
    int lShift;

    if (!xSemaphoreTake(handleMutex, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Mutex not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }
    if (!xSemaphoreTake(pixelsSemaphore, SEMAPHORE_MAX_WAIT)) {
        ESP_LOGE(TAG, "Semaphore not released after SEMAPHORE_MAX_WAIT. Rebooting...");
        abort();
    }
    for (int i = 0; i < 8; i++) {
        pixel = pixels[i];
        lShift = 8 - pixel.bright;
        ESP_RETURN_ON_ERROR(led_strip_set_pixel(
            handle, i, pixel.r >> lShift, pixel.g >> lShift, pixel.b >> lShift
        ), TAG, "Failed to set pixel #%d", i);
    }
    if (!xSemaphoreGive(pixelsSemaphore)) {
        ESP_LOGE(TAG, "Failed to release semaphore.");
        abort();
    }

    ESP_RETURN_ON_ERROR(led_strip_refresh(handle), TAG, "Failed to refresh.");
    if (!xSemaphoreGive(handleMutex)) {
        ESP_LOGE(TAG, "Failed to release mutex.");
        abort();
    }

    ESP_LOGD(TAG, "Done");
    return ESP_OK;
}
