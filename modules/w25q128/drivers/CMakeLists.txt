# 目录名是小写 w25q128,大小写必须与实际目录一致,否则 add_subdirectory 找不到
add_subdirectory_ifdef(CONFIG_WINBOND_W25Q128 w25q128)