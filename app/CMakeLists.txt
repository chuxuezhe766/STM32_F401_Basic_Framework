cmake_minimum_required(VERSION 3.20.0)

# 让 Zephyr 能找到我们自己的 dts/bindings
list(APPEND DTS_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/ws2812
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32_f401_basic_framework)

target_sources(app PRIVATE
    src/main.c
    ../modules/ws2812/drivers/ws2812/ws2812.c
    ../modules/ws2812/drivers/ws2812/ws2812_pwm_stm32_dma_backend.c
)

target_include_directories(app PRIVATE
    ../modules/ws2812/drivers/ws2812
)