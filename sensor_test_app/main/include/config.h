#ifndef __VINDRIKTNING_CONFIG_H_INCLUDED__
#define __VINDRIKTNING_CONFIG_H_INCLUDED__

#include "driver/gpio.h"
#include "driver/i2c.h"

#define SENSOR_STARTUP_DELAY 2000
#define SENSOR_GET_INTERVAL  2000

// I2C
#define I2C_NUM0_CLOCK_SPEED 20000  // 20k
// #define I2C_NUM0_CLOCK_SPEED 100000 // 100k
// #define I2C_NUM1_CLOCK_SPEED 20000  // 20k

// Status LED
#define ENABLE_STATUS_LED

// WS2812 Strip
// #define ENABLE_WS2812
#define WS2812_PIXEL_CNT 8

// PM1006
// #define ENABLE_PM1006
#define PM1006_TIMEOUT     1000
#define PM1006_TX_BUF_SIZE 256
#define PM1006_RX_BUF_SIZE 256
#define PM1006_THRESHOLD   199

// DHT11
// #define ENABLE_DHT11
// #define ENABLE_DHT11_CUSTOM
// #define ENABLE_DHT11_ULP
#define DHT11_TIMEOUT 1000

// LM35
// #define ENABLE_LM35
#define LM35_ADC_UNIT    ADC_UNIT_1
#define LM35_ADC_CHANNEL ADC_CHANNEL_9
#define LM35_SAMPLE_PER_MEASUREMENT 100

// AHT20
// #define ENABLE_AHT20
#define AHT20_I2C_NUM  I2C_NUM_0
#define AHT20_I2C_ADDR 0x38

// BMP280
// #define ENABLE_BMP280
#define BMP280_I2C_NUM  I2C_NUM_0
#define BMP280_I2C_ADDR 0x77
#define BMP280_TIMEOUT  1000

// AGS02MA
// #define ENABLE_AGS02MA
#define AGS02MA_I2C_NUM  I2C_NUM_0
#define AGS02MA_I2C_ADDR 0x1A
#define AGS02MA_TIMEOUT  1000

// BH1750
// #define ENABLE_BH1750
#define BH1750_I2C_NUM    I2C_NUM_0
#define BH1750_I2C_ADDR   0x23
#define BH1750_RESOLUTION BH1750_CONTINUE_1LX_RES

#if defined(ENABLE_DHT11) + defined(ENABLE_DHT11_CUSTOM) + defined(ENABLE_DHT11_ULP) + defined(ENABLE_AHT20) + defined(ENABLE_LM35) > 1
#error Only one of DHT11, AHT20 and LM35 can be enabled at a time.
#elif defined(ENABLE_DHT11) + defined(ENABLE_DHT11_CUSTOM) + defined(ENABLE_DHT11_ULP) + defined(ENABLE_AHT20) + defined(ENABLE_LM35) == 1
#define TEMPERATURE_SENSOR_ENABLED
#endif

// Pins
#define PIN_LED          GPIO_NUM_15
#define PIN_WS2812       GPIO_NUM_34

#define PIN_PM1006_TX    GPIO_NUM_33
#define PIN_PM1006_RX    GPIO_NUM_35
#define PIN_PM1006_FAN   GPIO_NUM_18

#define PIN_DHT11        GPIO_NUM_6

#ifdef I2C_NUM0_CLOCK_SPEED
#define PIN_I2C_NUM0_SDA GPIO_NUM_17
#define PIN_I2C_NUM0_SCL GPIO_NUM_21
#endif /* I2C_NUM0_CLOCK_SPEED */
#ifdef I2C_NUM1_CLOCK_SPEED
#define PIN_I2C_NUM1_SDA GPIO_NUM_35
#define PIN_I2C_NUM1_SCL GPIO_NUM_36
#endif /* I2C_NUM1_CLOCK_SPEED */

#ifdef ENABLE_STATUS_LED
#include "pwm_led.h"
extern PWMLed *statusLED;
#endif
#ifdef ENABLE_WS2812
#include "ws2812.h"
extern WS2812Strip *ws2812;
#endif

#endif