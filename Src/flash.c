/*
 * flash.c
 *
 *  Created on: Aug 5, 2017
 *      Author: libertas
 */

#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#include "flash.h"

/*
 * Flahs data structure
 *
 * [1B]            [1B]                     [2B]
 * version code    version code reversed    block choosing
 *
 * [12B]
 * reserved
 *
 * To avoid to erase too often
 * The other space will be divided into FLASH_BLOCKS_DIV blocks
 * And they make up only 1 virtual block for the user to use
 *
 */

uint8_t flash_version_code = 1;

uint32_t flash_sector_now;
uint32_t flash_addr_now;

uint32_t flash_sector_next;
uint32_t flash_addr_next;

void flash_verify_err_callback()
{
	sl_send(9, 1, "Flash Verify Error Occurred\n");
}

static int8_t flash_get_block()
{
	uint16_t *p = (uint16_t*)flash_addr_now;

	int i;

	for(i = 0; i < FLASH_BLOCKS_DIV ; i++) {
		if(*(p + 1) == 0xffff >> i) {
			return i;
		}
	}

	return i;
}

void flash_init()
{
	bool verify_data = flash_verify(FLASH_DATA_SECTOR);
	bool verify_back = flash_verify(FLASH_BACK_SECTOR);

	if(verify_data) {
		flash_sector_now = FLASH_DATA_SECTOR;
		flash_addr_now = FLASH_DATA_START_ADDR;

		flash_sector_next = FLASH_BACK_SECTOR;
		flash_addr_next = FLASH_BACK_START_ADDR;
	} else if(verify_back) {
		flash_sector_now = FLASH_BACK_SECTOR;
		flash_addr_now = FLASH_BACK_START_ADDR;

		flash_sector_next = FLASH_DATA_SECTOR;
		flash_addr_next = FLASH_DATA_START_ADDR;
	} else {
		flash_erase(FLASH_DATA_SECTOR);

		flash_sector_now = FLASH_DATA_SECTOR;
		flash_addr_now = FLASH_DATA_START_ADDR;

		flash_sector_next = FLASH_BACK_SECTOR;
		flash_addr_next = FLASH_BACK_START_ADDR;

		HAL_FLASH_Unlock();

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_sector_now, flash_version_code);
		FLASH_WaitForLastOperation(-1);

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_sector_now, ~flash_version_code);
		FLASH_WaitForLastOperation(-1);

		HAL_FLASH_Lock();
	}
}

bool flash_verify(uint32_t sector)
{
	uint16_t *p;
	switch(sector) {
	case FLASH_DATA_SECTOR:
		p = (uint16_t*)FLASH_DATA_START_ADDR;
		break;
	case FLASH_BACK_SECTOR:
		p = (uint16_t*)FLASH_BACK_START_ADDR;
		break;
	default:
		return false;
	}

	uint16_t fvc_verify = (((uint16_t)flash_version_code) << 8
			| flash_version_code) ^ 0xff00;

	bool block_choosing_err;

	if(flash_get_block() < FLASH_BLOCKS_DIV) {
		block_choosing_err = true;
	} else {
		block_choosing_err = false;
	}

	if(*p != fvc_verify || block_choosing_err) {
		return false;
	}

	return true;
}

void flash_erase(uint32_t sector)
{
	__disable_irq();

	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_SECTORS;
	f.Sector = sector;
	f.NbSectors = 1;
	f.Banks = FLASH_BANK_1;
	f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	uint32_t PageError = 0;

	HAL_FLASHEx_Erase(&f, &PageError);
	FLASH_WaitForLastOperation(-1);

	HAL_FLASH_Lock();

	__enable_irq();
}

static void flash_change_sector(uint16_t addr, uint8_t data[], uint16_t len)
{
	sl_send(9, 1, "change sector...", 17);
	osDelay(1);

	for(uint32_t i = 0; i < FLASH_DATA_SIZE; i++) {
		if(((uint8_t*)flash_addr_next)[i] != 0xff) {
			flash_erase(flash_sector_next);
			break;
		}
	}

	uint16_t fvc_verify = (((uint16_t)flash_version_code) << 8
				| flash_version_code) ^ 0xff00;

	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr_next, fvc_verify);
	FLASH_WaitForLastOperation(-1);
	HAL_FLASH_Lock();

	uint8_t c;

	for(uint32_t i = 0; i < FLASH_VIRTUAL_SIZE; i++) {

		if(i >= addr && i < addr + len) {
			c = data[i - addr];
		} else {
			c = ((uint8_t*)flash_addr_now)[FLASH_SYSTEM_INFO_LEN +\
										   (FLASH_BLOCKS_DIV - 1) * FLASH_VIRTUAL_SIZE\
										   + i];
		}

		if(*(uint8_t*)(flash_addr_next + FLASH_SYSTEM_INFO_LEN + i) != c) {
			HAL_FLASH_Unlock();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_addr_next + FLASH_SYSTEM_INFO_LEN + i, c);
			FLASH_WaitForLastOperation(-1);
			HAL_FLASH_Lock();
		}
	}

	flash_sector_now = flash_sector_now ^ flash_sector_next;
	flash_sector_next = flash_sector_now ^ flash_sector_next;
	flash_sector_now = flash_sector_now ^ flash_sector_next;

	flash_addr_now = flash_addr_now ^ flash_addr_next;
	flash_addr_next = flash_addr_now ^ flash_addr_next;
	flash_addr_now = flash_addr_now ^ flash_addr_next;

	flash_erase(flash_sector_next);

	sl_send(9, 1, "sector changed!", 16);
	osDelay(1);
}

bool flash_write(uint16_t addr, uint8_t data[], uint16_t len)
{
	if(addr + len > FLASH_VIRTUAL_SIZE) {
		return false;
	}

	uint8_t *p = flash_get_addr(addr);
	if(!p) {
		flash_init();
	}

	bool skip_flag = false;

	for(uint16_t i = 0; i < len; i++) {
		if((p[i] & data[i]) == data[i]) {
			continue;
		} else {
			skip_flag = true;
			break;
		}
	}

	if(skip_flag) {
		int16_t block = flash_get_block();

		if(block == FLASH_BLOCKS_DIV - 1) {
			flash_change_sector(addr, data, len);
		} else {
			uint16_t c;

			c = 0xffff >> block >> 1;

			HAL_FLASH_Unlock();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr_now + 2, c);
			FLASH_WaitForLastOperation(-1);
			HAL_FLASH_Lock();

			uint32_t src = flash_addr_now + FLASH_BLOCKS_DIV + block * FLASH_VIRTUAL_SIZE;
			uint32_t dst = src + FLASH_BLOCKS_DIV;

			for(uint32_t i = 0; i < FLASH_VIRTUAL_SIZE; i++) {
				HAL_FLASH_Unlock();
				if(i >= addr && i < addr + len) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dst + i, data[i - addr]);
				} else {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dst + i, ((uint8_t*)(&src))[i]);
				}
				FLASH_WaitForLastOperation(-1);
				HAL_FLASH_Lock();
			}
		}

	} else {
		for(uint16_t i = 0; i < len; i++) {
			HAL_FLASH_Unlock();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)p + i, data[i]);
			FLASH_WaitForLastOperation(-1);
			HAL_FLASH_Lock();
		}
	}

	return true;
}


uint8_t* flash_get_addr(uint16_t addr)
{
	int8_t block = flash_get_block();
	if(block < FLASH_BLOCKS_DIV) {
		return (uint8_t*)(flash_addr_now + FLASH_SYSTEM_INFO_LEN + block * FLASH_VIRTUAL_SIZE + addr);
	} else {
		return NULL;
	}
}