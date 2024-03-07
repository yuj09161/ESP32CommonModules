#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "config.h"
#include "io.h"

#ifdef ENABLE_STATUS_LED
#include "pwm_led.h"
#endif
#ifdef ENABLE_WS2812
#include "ws2812.h"
#endif


#ifdef ENABLE_STATUS_LED
void led_task(void *) {
    statusLED->setBright(128);
    while (1) {
        statusLED->toggle();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#ifdef ENABLE_WS2812
void ws2812_task(void *) {
    ws2812->setBright(3, false);
    WS2812Pixel pixel = {};
    while (1) {
        for (int i = 0; i < 3; i++) {
            switch (i) {
                case 0:
                    pixel.r = 255;
                    pixel.g = 0;
                    pixel.b = 0;
                    break;
                case 1:
                    pixel.r = 0;
                    pixel.g = 255;
                    pixel.b = 0;
                    break;
                case 2:
                    pixel.r = 0;
                    pixel.g = 0;
                    pixel.b = 255;
                    break;
            }
            for (int j = 0; j < 8; j++) {
                ws2812->setColor(j, pixel);
                vTaskDelay(pdMS_TO_TICKS(100));
                ws2812->setColor(j, WS2812_OFF);
            }
        }
    }
}
#endif

void get_sensor_value_task(void *) {
    TickType_t last_start_time;
    sensor_values values;
    vTaskDelay(pdMS_TO_TICKS(SENSOR_STARTUP_DELAY));
    while (1) {
        last_start_time = xTaskGetTickCount();
        get_sensor_values(&values);
        vTaskDelayUntil(&last_start_time, pdMS_TO_TICKS(SENSOR_GET_INTERVAL));
    }
}

extern "C" void app_main(void) {
    init_gpio();
    init_modules();

#ifdef ENABLE_STATUS_LED
    xTaskCreate(led_task, "led_task", 2048, NULL, 10, NULL);
#endif
#ifdef ENABLE_WS2812
    xTaskCreate(ws2812_task, "ws2812_task", 4096, NULL, 11, NULL);
#endif
    xTaskCreate(get_sensor_value_task, "get_sensor_value_task", 4096, NULL, 12, NULL);
}