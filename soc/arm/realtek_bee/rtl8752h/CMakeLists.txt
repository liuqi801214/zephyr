zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)
zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)
zephyr_linker_sources(SECTIONS trace_section.ld)
zephyr_library_sources(soc.c)

if (CONFIG_CODE_DATA_RELOCATION)
# save DataRAM for APP
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/logging/log_core.c LOCATION RAM2_BSS)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/shell/backends/shell_uart.c LOCATION RAM2_BSS)
endif()
