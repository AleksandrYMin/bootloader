#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
/*Variable used for Erase procedure*/

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_8   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11  +  GetSectorSize(ADDR_FLASH_SECTOR_11) -1 /* End @ of user Flash area : sector start address + sector size -1 */
#define APP_FLASH_FIRST_PAGE_ADDRESS ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define CONFIGURATION_START_ADDR   ADDR_FLASH_SECTOR_3
//#define DATA_32                 ((uint32_t)0x12345678)


void WriteDeviceAddress(char* data, int size);

void CopyData(size_t destStartAddr, size_t srcStartAddr,const size_t srcEndAddr);

uint8_t clearFlash(uint32_t startAddr, uint32_t endAddr);

int WriteDeviceAddressOffset(char* data, int size, int offset);

void ReadDeviceAddressOffset(char* Dout, int size, int offset);

void ReadDeviceAddress(char* Dout, int size);

/**
 * @brief  Gets the sector of a given address
 * @param  None
 * @retval The sector of a given address
 */
uint32_t GetSector(uint32_t Address);

/**
 * @brief  Gets sector Size
 * @param  None
 * @retval The size of a given sector
 */
uint32_t GetSectorSize(uint32_t Sector);

#endif /* __FLASH_H */
