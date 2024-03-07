idf_component_register(
    SRCS
        "src/i2c_common.cpp"

        "src/ags02ma.cpp"
        "src/bmp280.cpp"
        "src/dht_ulp.cpp"
        "src/pwm_led.cpp"
        "src/ws2812.cpp"

        "src/dht_rmt.c"
        "src/lm35.c"
        "src/pm1006.c"

        "src/utils/semaphore.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        driver
        ulp
)

set(ulp_app_name ulp_read_sensor_values)
set(ulp_riscv_sources
    "src/ulp/ulp_main.c"
    "src/ulp/sensors/dht.c"
)
set(ulp_exp_dep_srcs "src/modules/dht.cpp")
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")
