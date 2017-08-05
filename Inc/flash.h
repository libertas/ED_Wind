/*
 * flash.h
 *
 *  Created on: Aug 5, 2017
 *      Author: libertas
 */

#ifndef FLASH_H_
#define FLASH_H_

#define FLASH_DATA_SECTOR FLASH_SECTOR_6
#define FLASH_BACK_SECTOR FLASH_SECTOR_7

#define FLASH_DATA_START_ADDR 0x08040000
#define FLASH_BACK_START_ADDR 0x08060000

#define FLASH_DATA_SIZE 128000

#define FLASH_SYSTEM_INFO_LEN 16

#define FLASH_BLOCKS_DIV 16

#define FLASH_VIRTUAL_SIZE ((FLASH_DATA_SIZE - FLASH_SYSTEM_INFO_LEN) / FLASH_BLOCKS_DIV)

#define FLASH_VOLTAGE FLASH_VOLTAGE_RANGE_3


void flash_init();
bool flash_verify(uint32_t sector);
void flash_erase(uint32_t sector);
bool flash_write(uint16_t addr, uint8_t data[], uint16_t len);

/* this address is only usable before other write or erase operations */
uint8_t* flash_get_addr(uint16_t addr);

#endif /* FLASH_H_ */
