#include <cstdint>
#include <cstring>
#include <climits>
#include <cfloat>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"

#include "config.h"
#include "io.h"

#ifdef ENABLE_STATUS_LED
#include "pwm_led.h"
#endif
#ifdef ENABLE_WS2812
#include "ws2812.h"
#endif

#ifdef ENABLE_PM1006
#include "pm1006.h"
#endif
#ifdef ENABLE_BH1750
#include "bh1750.h"
#endif
#ifdef ENABLE_AHT20
#include "aht20.h"
#endif
#ifdef ENABLE_BMP280
#include "bmp280.h"
#endif
#ifdef ENABLE_AGS02MA
#include "ags02ma.h"
#endif


#ifdef ENABLE_DHT11
#include "am2302_rmt.h"
#endif
#ifdef ENABLE_DHT11_CUSTOM
#include "dht_rmt.h"
#endif
#ifdef ENABLE_DHT11_ULP
#include "dht_ulp.h"
#endif
#ifdef ENABLE_LM35
#include "lm35.h"
#endif

#define BUF_SIZE 50


uint_fast8_t led_status = 0;

#ifdef ENABLE_STATUS_LED
PWMLed *statusLED;
#endif
#ifdef ENABLE_WS2812
WS2812Strip *ws2812;
#endif

#ifdef ENABLE_BH1750
bh1750_handle_t bh1750;
#endif
#ifdef ENABLE_PM1006
pm1006_handle_t pm1006;
#endif
#ifdef ENABLE_AHT20
aht20_dev_handle_t aht20;
#endif
#ifdef ENABLE_AGS02MA
AGS02MA *ags02ma;
#endif
#ifdef ENABLE_BMP280
BMP280 *bmp280;
#endif
#ifdef ENABLE_DHT11
am2302_handle_t dht11;
#endif
#ifdef ENABLE_DHT11_CUSTOM
dht_handle_t dht11;
#endif
#ifdef ENABLE_DHT11_ULP
DHT *dht11;
#endif
#ifdef ENABLE_LM35
lm35_handle_t lm35;
#endif

esp_err_t get_pm1006_value(int *fine_dust);
esp_err_t get_temphumi(float *temperature, int *humidity);
esp_err_t get_temppress(float *temperature, float *pressure);
esp_err_t get_tvoc(int *tvoc);
esp_err_t get_illuminance(int *illuminance);


void init_modules(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ll << PIN_PM1006_FAN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_PM1006_FAN, 1);

    // Onboard status LED
#ifdef ENABLE_STATUS_LED
    statusLED = new PWMLed(PIN_LED, LEDC_TIMER_0, LEDC_CHANNEL_0);
    ESP_ERROR_CHECK(statusLED->begin());
#endif

    // WS2812 Strip
#ifdef ENABLE_WS2812
    ws2812 = new WS2812Strip(PIN_WS2812, WS2812_PIXEL_CNT);
    ESP_ERROR_CHECK(ws2812->begin());
#endif

#ifdef I2C_NUM0_CLOCK_SPEED
    // I2C Num0
    ESP_LOGI("IO:init_modules", "Initializing I2C #0...");
    i2c_config_t i2c_num0_conf = {};
        i2c_num0_conf.mode             = I2C_MODE_MASTER;
        i2c_num0_conf.sda_io_num       = PIN_I2C_NUM0_SDA;
        i2c_num0_conf.scl_io_num       = PIN_I2C_NUM0_SCL;
        i2c_num0_conf.sda_pullup_en    = GPIO_PULLUP_DISABLE;
        i2c_num0_conf.scl_pullup_en    = GPIO_PULLUP_DISABLE;
        i2c_num0_conf.master.clk_speed = I2C_NUM0_CLOCK_SPEED;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_num0_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
#endif

#ifdef I2C_NUM1_CLOCK_SPEED
    // I2C Num1
    ESP_LOGI("IO:init_modules", "Initializing I2C #1...");
    i2c_config_t i2c_num1_conf = {};
        i2c_num1_conf.mode             = I2C_MODE_MASTER;
        i2c_num1_conf.sda_io_num       = PIN_I2C_NUM1_SDA;
        i2c_num1_conf.scl_io_num       = PIN_I2C_NUM1_SCL;
        i2c_num1_conf.sda_pullup_en    = GPIO_PULLUP_DISABLE;
        i2c_num1_conf.scl_pullup_en    = GPIO_PULLUP_DISABLE;
        i2c_num1_conf.master.clk_speed = I2C_NUM1_CLOCK_SPEED;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_num1_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_num1_conf.mode, 0, 0, 0));
#endif

#ifdef ENABLE_PM1006
    // PM1006 - UART
    ESP_LOGI("IO:init_modules", "Initializing PM1006...");
    pm1006_config_t pm1006_cfg = {};
        pm1006_cfg.uart_num    = UART_NUM_1;
        pm1006_cfg.tx_pin      = PIN_PM1006_TX;
        pm1006_cfg.rx_pin      = PIN_PM1006_RX;
        pm1006_cfg.tx_buf_size = PM1006_TX_BUF_SIZE;
        pm1006_cfg.rx_buf_size = PM1006_RX_BUF_SIZE;
        pm1006_cfg.timeout     = PM1006_TIMEOUT;
    ESP_ERROR_CHECK(pm1006_create(&pm1006_cfg, &pm1006));
#endif

// Temperature sensors
#ifdef ENABLE_AHT20
    // AHT20 - espressif/aht20 (I2C)
    ESP_LOGI("IO:init_modules", "Initializing AHT20...");
    aht20_i2c_config_t aht20_i2c_cfg = {
        .i2c_port = AHT20_I2C_NUM,
        .i2c_addr = (AHT20_I2C_ADDR << 1) // The library needs this
    };
    ESP_ERROR_CHECK(aht20_new_sensor(&aht20_i2c_cfg, &aht20));
#endif

#ifdef ENABLE_DHT11
    // DHT11 - suda-morris/am2302_rmt (RMT)
    am2302_config_t am2302_config = {
        .gpio_num = PIN_DHT11,
    };
    am2302_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(am2302_new_sensor_rmt(&am2302_config, &rmt_config, &dht11));
#endif
#ifdef ENABLE_DHT11_CUSTOM
    // DHT11 - RMT
    ESP_LOGI("IO:init_modules", "Initializing DHT11...");
    dht_config_t dht_cfg = {};
        dht_cfg.pin     = PIN_DHT11;
        dht_cfg.timeout = DHT11_TIMEOUT;
    
    ESP_ERROR_CHECK(dht_create(&dht_cfg, &dht11));
#endif
#ifdef ENABLE_DHT11_ULP
    // DHT11 - ULP
    ESP_LOGI("IO:init_modules", "Initializing DHT11...");
    dht11 = new DHT(PIN_DHT11);
#endif

#ifdef ENABLE_LM35
    // LM35 - ADC
    ESP_LOGI("IO:init_modules", "Initializing LM35...");
    lm35_config_t lm35_cfg = {
        .unit    = LM35_ADC_UNIT,
        .channel = LM35_ADC_CHANNEL,
        .sample_per_measurement = LM35_SAMPLE_PER_MEASUREMENT
    };
    ESP_ERROR_CHECK(lm35_create(&lm35_cfg, &lm35));
#endif
// End: Temperature sensors

#ifdef ENABLE_BMP280
    // BMP280 - I2C
    ESP_LOGI("IO:init_modules", "Initializing BMP280...");
    bmp280 = new BMP280(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_TIMEOUT);
    bmp280->begin(
        BMP280::PM_NORMAL, BMP280::P_UHIGH, BMP280::T_STANDARD, BMP280::FILTER_OFF, BMP280::STBY_1s
    );
#endif

#ifdef ENABLE_AGS02MA
    // AGS02MA - I2C
    ESP_LOGI("IO:init_modules", "Initializing AGS02MA...");
    ags02ma = new AGS02MA(AGS02MA_I2C_NUM, AGS02MA_I2C_ADDR, AGS02MA_TIMEOUT);
    ags02ma->begin();
#endif

#ifdef ENABLE_BH1750
    // BH1750 - espressif/bh1750 (I2C)
    ESP_LOGI("IO:init_modules", "Initializing BH1750...");
    bh1750 = bh1750_create(BH1750_I2C_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    if (bh1750_power_on(bh1750) != ESP_OK) {
        ESP_LOGW("IO:init_modules", "Failed to init bh1750.");
    }
    if (bh1750_set_measure_mode(bh1750, BH1750_RESOLUTION) != ESP_OK) {
        ESP_LOGW("IO:init_modules", "Failed to set measure mode of bh1750.");
    }
#endif
}

void init_gpio(void) {
    gpio_config_t cfg = {
        1 << PIN_LED,
        GPIO_MODE_OUTPUT,
        GPIO_PULLUP_DISABLE,
        GPIO_PULLDOWN_DISABLE,
        GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_LED, 0);
}

esp_err_t get_sensor_values(sensor_values *result) {
    esp_err_t err;
#ifdef ENABLE_PM1006
    if ((err = get_pm1006_value(&result->fine_dust)) != ESP_OK) {
        ESP_LOGW(
            "IO:get_sensor_values:get_pm1006_value",
            "Failed to get PM1006 value (Error: %s).",
            esp_err_to_name(err)
        );
        result->fine_dust = 0;
    }
#endif
#ifdef TEMPERATURE_SENSOR_ENABLED
    if ((err = get_temphumi(&result->temperature, &result->humidity)) != ESP_OK) {
        ESP_LOGW(
            "IO:get_sensor_values:get_temphumi",
            "Failed to get temperature or humidity (Error: %s).",
            esp_err_to_name(err)
        );
        result->temperature = FLT_MIN;
        result->humidity    = 0;
    }
#endif
#ifdef ENABLE_BMP280
    if ((err = get_temppress(&result->temperature2, &result->pressure)) != ESP_OK) {
        ESP_LOGW(
            "IO:get_sensor_values:get_temppress",
            "Failed to get temperature or pressure (Error: %s).",
            esp_err_to_name(err)
        );
        result->temperature2 = FLT_MIN;
        result->pressure     = FLT_MIN;
    }
#endif
#ifdef ENABLE_AGS02MA
    if ((err = get_tvoc(&result->tvoc)) != ESP_OK) {
        ESP_LOGW(
            "IO:get_sensor_values:get_tvoc",
            "Failed to get TVOC (Error: %s).",
            esp_err_to_name(err)
        );
        result->tvoc = 0;
    }
#endif
#ifdef ENABLE_BH1750
    if ((err = get_illuminance(&result->illuminance)) != ESP_OK) {
        ESP_LOGW(
            "IO:get_sensor_values:get_illuminance",
            "Failed to get illuminance (Error: %s).",
            esp_err_to_name(err)
        );
        result->illuminance = INT_MIN;
    }
#endif
    return ESP_OK;
}

esp_err_t get_pm1006_value(int *fine_dust) {
#ifdef ENABLE_PM1006
    ESP_RETURN_ON_ERROR(
        pm1006_get_value(pm1006, fine_dust),
        "IO:get_pm1006_value", "Failed to get PM2.5."
    );
    ESP_LOGI("IO:get_pm1006_value", "PM2.5: %dµg/m^3", *fine_dust);
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t get_temphumi(float *temperature, int *humidity) {
#ifdef ENABLE_DHT11
    float decimal_temperature = 0, decimal_humidity = 0;
    ESP_RETURN_ON_ERROR(
        am2302_read_temp_humi(dht11, &decimal_temperature, &decimal_humidity),
        "IO:get_temphumi", "Failed to get temperature/humidity."
    );
    *temperature = decimal_temperature;
    *humidity = (int)(decimal_humidity);
#elif defined(ENABLE_DHT11_CUSTOM)
    ESP_RETURN_ON_ERROR(
        dht_get_value(dht11, temperature, humidity),
        "IO:get_temphumi", "Failed to get temperature/humidity."
    );
#elif defined(ENABLE_DHT11_ULP)
    DHTResult result;
    ESP_RETURN_ON_ERROR(
        dht11->get(&result),
        "IO:get_temphumi", "Failed to get temperature/humidity."
    );
    *temperature = result.temperature;
    *humidity = (int)result.humidity;
#elif defined(ENABLE_AHT20)
    uint32_t raw_temperature, raw_humidity;
    float decimal_temperature, decimal_humidity;
    ESP_RETURN_ON_ERROR(
        aht20_read_temperature_humidity(aht20, &raw_temperature, &decimal_temperature, &raw_humidity, &decimal_humidity),
        "IO:get_temphumi", "Failed to get temperature/humidity."
    );
    *temperature = decimal_temperature;
    *humidity = (int)(decimal_humidity);
#elif defined(ENABLE_LM35)
    ESP_RETURN_ON_ERROR(
        lm35_get_value(lm35, temperature),
        "IO:get_temphumi", "Failed to get temperature."
    );
    *humidity = -1;
#endif
#ifdef TEMPERATURE_SENSOR_ENABLED
    ESP_LOGI("IO:get_temphumi", "Temperature: %.01f°C | Humidity: %d%%", *temperature, *humidity);
#endif
    return ESP_OK;
}

esp_err_t get_temppress(float *temperature, float *pressure) {
#ifdef ENABLE_BMP280
    BMP280Result result = {};
    ESP_RETURN_ON_ERROR(
        bmp280->get(&result),
        "IO:get_temppress", "Failed to get temperature/air pressure."
    );
    *temperature = result.temperature;
    *pressure = result.pressure;
    ESP_LOGI("IO:get_temppress", "Temperature: %.01f°C | Air Pressure: %.02fhPa", *temperature, *pressure);
    return ESP_OK;
#else
    ESP_LOGW("IO:get_temppress", "BMP280: Disabled.");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t get_tvoc(int *tvoc) {
#ifdef ENABLE_AGS02MA
    int result;
    ESP_RETURN_ON_ERROR(
        ags02ma->get(&result),
        "IO:get_tvoc", "Failed to get TVOC."
    );
    *tvoc = result;
    ESP_LOGI("IO:get_tvoc", "TVOC: %dppb", *tvoc);
    return ESP_OK;
#else
    ESP_LOGW("IO:get_tvoc", "AGS02MA: Disabled.");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t get_illuminance(int *illuminance) {
#ifdef ENABLE_BH1750
    float result_raw;
    ESP_RETURN_ON_ERROR(bh1750_get_data(bh1750, &result_raw), "IO:get_illuminance", "Failed to get illuminance");
    *illuminance = (int)result_raw;
    ESP_LOGI("IO:get_illuminance", "illuminance: %dlx", *illuminance);
#endif
    return ESP_OK;
}

void led_off(void) {
    gpio_set_level(PIN_LED, led_status = 0);
}

void led_on(void) {
    gpio_set_level(PIN_LED, led_status = 1);
}

void led_toggle(void) {
    gpio_set_level(PIN_LED, led_status ^= 1);
}
