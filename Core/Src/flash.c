/*
 * flash.c
 *
 *  Created on: Jul 28, 2021
 *      Author: lab162
 */
#include "flash.h"

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t GetSector(uint32_t Address);

void WriteDeviceAddress(char* data, int size)
{
	HAL_FLASH_Unlock();

	/* Get the 1st sector to erase */
	uint32_t FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	uint32_t NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
	uint32_t SectorError = 0;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*
	      Error occurred while sector erase.
	      User can add here some code to deal with this error.
	      SectorError will contain the faulty sector and then to know the code error on this sector,
	      user can call function 'HAL_FLASH_GetError()'
		 */
		/*
	      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		return;
	}
	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	uint32_t Address = FLASH_USER_START_ADDR;

	for (int i = 0; i<size; i++){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address+i, data[i]) != HAL_OK){
		  /* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
		  /*
			FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		  */
		  return;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
}

void CopyData(size_t destStartAddr, size_t srcStartAddr,const size_t srcEndAddr)
{
	HAL_FLASH_Unlock();

	size_t dst = destStartAddr;
	size_t src = srcStartAddr;

	while ((size_t)src<srcEndAddr){
		uint32_t data = *(__IO uint32_t*)(src);

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dst, data) != HAL_OK){

		  /* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */

			//FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();

			return;
		}
		dst += sizeof(uint32_t);
		src += sizeof(uint32_t);
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
}

uint8_t clearFlash(uint32_t startAddr, uint32_t endAddr){
	HAL_FLASH_Unlock();

	/* Get the 1st sector to erase */
	uint32_t FirstSector = GetSector(startAddr);
	/* Get the number of sector to erase from 1st sector*/
	uint32_t NbOfSectors = GetSector(endAddr) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
	uint32_t SectorError = 0;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*
	      Error occurred while sector erase.
	      User can add here some code to deal with this error.
	      SectorError will contain the faulty sector and then to know the code error on this sector,
	      user can call function 'HAL_FLASH_GetError()'
		 */
		/*
	      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		return 0;
	}
	HAL_FLASH_Lock();
	return 1;
}

int WriteDeviceAddressOffset(char* data, int size, int offset)
{
	HAL_FLASH_Unlock();

	uint32_t Address = APP_FLASH_FIRST_PAGE_ADDRESS+offset;

	for (int i = 0; i<size; i++){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address+i, data[i]) != HAL_OK){
		  /* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
		  /*
			FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		  */
		  return 0;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return 1;
}

void ReadDeviceAddressOffset(char* Dout, int size, int offset)
{
	uint32_t Address = FLASH_USER_START_ADDR+offset;

	for (int i = 0; i<size; i++){
		Dout[i] = *(__IO char*)(Address+i);
	}
}

void ReadDeviceAddress(char* Dout, int size)
{
	uint32_t Address = FLASH_USER_START_ADDR;

	for (int i = 0; i<size; i++){
		Dout[i] = *(__IO char*)(Address+i);
	}
}

/**
 * @brief  Gets the sector of a given address
 * @param  None
 * @retval The sector of a given address
 */
uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_SECTOR_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_SECTOR_8;
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_SECTOR_9;
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_SECTOR_10;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
	{
		sector = FLASH_SECTOR_11;
	}

	return sector;
}

/**
 * @brief  Gets sector Size
 * @param  None
 * @retval The size of a given sector
 */
uint32_t GetSectorSize(uint32_t Sector)
{
	uint32_t sectorsize = 0x00;

	if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
	{
		sectorsize = 16 * 1024;
	}
	else if(Sector == FLASH_SECTOR_4)
	{
		sectorsize = 64 * 1024;
	}
	else
	{
		sectorsize = 128 * 1024;
	}
	return sectorsize;
}
