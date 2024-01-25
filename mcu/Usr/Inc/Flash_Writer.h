#ifndef FLASH_WRITER_H_
#define FLASH_WRITER_H_

#include "my_types.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0   ((uint32_t)0x08000000) /* Base address of Sector 0, 16  Kbytes */
#define ADDR_FLASH_SECTOR_1   ((uint32_t)0x08004000) /* Base address of Sector 1, 16  Kbytes */
#define ADDR_FLASH_SECTOR_2   ((uint32_t)0x08008000) /* Base address of Sector 2, 16  Kbytes */
#define ADDR_FLASH_SECTOR_3   ((uint32_t)0x0800C000) /* Base address of Sector 3, 16  Kbytes */
#define ADDR_FLASH_SECTOR_4   ((uint32_t)0x08010000) /* Base address of Sector 4, 64  Kbytes */
#define ADDR_FLASH_SECTOR_5   ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6   ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7   ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_MAX ((uint32_t)0x0807FFFF)

void flash_erase_address(uint32_t address, uint16_t len);
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
void flash_read(uint32_t address, uint32_t *buf, uint32_t len);
uint32_t get_sector(uint32_t address);
uint32_t get_next_flash_address(uint32_t address);
#endif
