# Custom Zephyr funtion that imports the button/ subdirectory if the Kconfig
# option CUSTOM_BUTTON is defined
add_subdirectory_ifdef(CONFIG_WINBOND_W25Q128 W25Q128)