# bootloader
STM32F407 XMODEM bootloader with CDC USB intefrace

If button bootloader_pin is not pressed, bootloader checks if a new firmware present on flash address FLASH_USER_START_ADDR (0x08080000). Calculates CRC an copies pragramm data to APP_FLASH_FIRST_PAGE_ADDRESS (0x08010000).
If the button was pressed, bootloader goes into firmware update mode with XMODEM(CRC) protocol using CDC USB interface.
