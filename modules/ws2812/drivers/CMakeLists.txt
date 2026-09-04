# 目录名是小写 ws2812,大小写必须与实际目录一致,否则 add_subdirectory 找不到
add_subdirectory_ifdef(CONFIG_CUSTOM_WS2812 ws2812)