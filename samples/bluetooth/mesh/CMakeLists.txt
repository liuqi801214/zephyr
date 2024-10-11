# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(QEMU_EXTRA_FLAGS -s)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mesh)

target_sources(app PRIVATE src/main.c)

if (BOARD STREQUAL bbc_microbit)
  target_sources(app PRIVATE src/microbit.c)
else()
  target_sources(app PRIVATE src/board.c)
endif()

if (CONFIG_BUILD_WITH_TFM)
  target_include_directories(app PRIVATE
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
  )
endif()

zephyr_code_relocate(FILES ${ZEPHYR_BASE}/lib/os/cbprintf_complete.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/drivers/serial/uart_rtl87x2g.c LOCATION ITCM)
zephyr_code_relocate(FILES ${WEST_TOPDIR}/modules/hal/realtek/bee/drivers/uart/src/rtl_common/rtl_uart.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/lib/libc/minimal/source/stdout/stdout_console.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/drivers/console/uart_console.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/logging/log_output.c LOCATION ITCM)
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/logging/backends/log_backend_uart.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/logging/backends/*.c LOCATION ITCM)