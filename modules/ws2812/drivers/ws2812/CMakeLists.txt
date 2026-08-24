# Declares the current directory as a Zephyr library
# If no name is given, the name is derived from the directory name
zephyr_library()

# List the source code files for the library
zephyr_library_sources(ws2812.c)
zephyr_library_sources(ws2812_pwm_stm32_dma_backend.c)

# Add header files to the CMake search directories 
zephyr_include_directories(.)