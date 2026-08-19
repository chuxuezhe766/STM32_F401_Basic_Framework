board_runner_args(jlink "--device=STM32F401RE" "--speed=4000")
board_runner_args(openocd --cmd-reset-halt "reset halt")

include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)