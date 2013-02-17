#ifndef __FLASH_h
#define __FLASH_h

#include "stm32f10x.h"

#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)

#define LAST_PAGE (uint32_t) 0x800FC00
#define PAGE_SIZE 1024

void flash_unlock(void);
void flash_erase_page(uint32_t address);
void WriteFlash(void* Src, void* Dst, int Len);
void * FindNextAddr (int len);
void * findLastBlock (int len);


#endif