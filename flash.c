
#include "flash.h"


void flash_unlock(void) 
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}
//-----------------------------------------------------------
//������� ������� ���� ��������. � �������� ������ ����� ������������ �����
//������������� ��������� ������� ��� �������� ������� ����� ��������.
void flash_erase_page(uint32_t address) 
{
	FLASH->CR|= FLASH_CR_PER; 	//������������� ��� �������� ����� ��������
	FLASH->AR = address; 		// ������ � �����
	FLASH->CR|= FLASH_CR_STRT; 	// ��������� ��������
	while ((FLASH->SR & FLASH_SR_BSY) != 0 );  //���� ���� �������� ��������.
	FLASH->CR&= ~FLASH_CR_PER; 	//���������� ��� �������
}

//-----------------------------------------------------------
void WriteFlash(void* Src, void* Dst, int Len)
{
  uint16_t* SrcW = (uint16_t*)Src;
  volatile uint16_t* DstW = (uint16_t*)Dst;

  FLASH->CR |= FLASH_CR_PG; /* Programm the flash */
  while (Len)
  {
    *DstW++ = *SrcW++;
	Len -= sizeof(uint16_t);
    while ((FLASH->SR & FLASH_SR_BSY) != 0 );
  }
  FLASH->CR &= ~FLASH_CR_PG; /* Reset the flag back !!!! */
}
void * FindNextAddr (int len)
{
	uint8_t * addr = (uint8_t *)LAST_PAGE;
	while (addr <= (uint8_t *)( LAST_PAGE+PAGE_SIZE-len))
	{
		if (*(uint32_t*)addr == 0xffffffff) return addr;
		addr += len;
	}
	flash_erase_page (LAST_PAGE);
	return (void *)LAST_PAGE;

}
void * findLastBlock (int len)
{
	uint8_t * addr = (uint8_t *)LAST_PAGE;
	while (addr <= (uint8_t *)( LAST_PAGE+PAGE_SIZE-len))
	{
		if (*(uint32_t*)addr == 0xffffffff)
		{
			return addr-len;
		}
		addr += len;
	}
	return  addr-len;;
}
