cmake_minimum_required(VERSION 3.20.0)

# prj.conf 等配置文件在 app/ 目录下,必须在 find_package 之前设置
set(APPLICATION_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/app)

# 自定义板子在项目根的 boards/ 下,BOARD_ROOT 指向包含 boards 子目录的目录
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# 让 Zephyr 能找到我们自己的 boards / dts/bindings
list(APPEND DTS_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ws2812
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/w25q128
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/mpu6050
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/qmc5883l
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/sht30
)

# 本项目不在 west 工作区里,Zephyr 无法通过 west 清单自动发现本地模块,
# 必须显式注册;注册后各模块 module.yaml 里的 kconfig/cmake/dts_root 才会生效
set(ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ws2812
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/w25q128
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/mpu6050
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/qmc5883l
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/sht30
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32_f401_basic_framework)

target_sources(app PRIVATE
    # src
    app/src/main.c
    # middleware
    middleware/flash_log/flash_log.c
    middleware/attitude/attitude.c
)

target_include_directories(app PRIVATE
    # middleware
    middleware/flash_log
    middleware/attitude
)
