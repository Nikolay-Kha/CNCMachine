#include "veeprom.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

// http://easystm32.ru/for-beginners/38-flash-stm32
#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)

// http://chipspace.ru/stm32-flash-structure/
#define VE_PAGE_SIZE 2048
#define VE_FLASH_ADDRESS 0x08000000
#define VE_PAGES_COUNT 255
#define VE_CRC_SIZE sizeof (uint32_t)

unsigned char *pageToAddress(unsigned int page) {
	if(page>VE_PAGES_COUNT)
		return 0;
	return (unsigned char *)(VE_FLASH_ADDRESS+page*VE_PAGE_SIZE);
}

uint32_t calcPageCRC(unsigned int page)
{
	volatile uint32_t *crc_p  = (volatile uint32_t *)pageToAddress(page);
	unsigned int i;
	if(crc_p==0)
		return 0;
	// http://easystm32.ru/useful-things/17-crc-calculation
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE); //Включим модуль подсчета CRC
	CRC->CR |= CRC_CR_RESET; //Делаем сброс...
	for(i=0; i<(VE_PAGE_SIZE-VE_CRC_SIZE)/VE_CRC_SIZE; i++) {
		CRC->DR = crc_p[i];
	}
	uint32_t crc_res = CRC->DR;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);
	return crc_res;
}

uint32_t getPageCRC(unsigned int page)
{
	volatile uint32_t *crc_p  = (volatile uint32_t *)pageToAddress(page);
	if(crc_p==0)
		return 0;
	return crc_p[(VE_PAGE_SIZE-VE_CRC_SIZE)/VE_CRC_SIZE];
}

int VEEPROM_GetData(unsigned int page, void *out, unsigned int size)
{
	if(size>VE_PAGE_SIZE-VE_CRC_SIZE)
		return 0;
	volatile unsigned char *a = pageToAddress(page);
	if(a==0)
		return 0;

	if(calcPageCRC(page)!=getPageCRC(page))
		return 0;
	unsigned int i;
	for(i=0; i<size; i++)
		((unsigned char *)out)[i] = a[i];
	return 1;
}

//Функция возврщает true когда можно стирать или писать память.
uint8_t flash_ready(void) {
	return !(FLASH->SR & FLASH_SR_BSY);
}

void flash_unlock(void) {
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}

void flash_lock() {
	FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase_page(uint32_t address) {
	FLASH->CR|= FLASH_CR_PER; //Устанавливаем бит стирания одной страницы
	FLASH->AR = address; // Задаем её адрес
	FLASH->CR|= FLASH_CR_STRT; // Запускаем стирание
	while(!flash_ready());  //Ждем пока страница сотрется.
	FLASH->CR&= ~FLASH_CR_PER; //Сбрасываем бит обратно
}

int VEEPROM_SetData(unsigned int page, void *data, unsigned int size)
{
	if(size>VE_PAGE_SIZE-VE_CRC_SIZE)
		return 0;
	volatile uint16_t* a = (uint16_t*)pageToAddress(page);
	if(a==0)
		return 0;

	flash_unlock();
	flash_erase_page((uint32_t)a);

	FLASH->CR |= FLASH_CR_PG; //Разрешаем программирование флеша
	while(!flash_ready()); //Ожидаем готовности флеша к записи

	// запись
	int i;
	uint32_t to = size/sizeof (uint16_t);
	for(i=0; i<to; i++) {
		a[i] =((uint16_t *)data)[i];
	}
	if(to*2<size)
		a[to] = (uint16_t)(((uint8_t *)data)[size-1]);
	while(!flash_ready());

	// проверка
	int ok = 1;
	for(i=0; i<size; i++) {
		if( ((uint8_t *)a)[i]!= ((uint8_t *)data)[i] ) {
			ok = 0;
			break;
		}
	}
	if(ok) { // метим CRC
		uint32_t crc = calcPageCRC(page);
		a[VE_PAGE_SIZE/sizeof(uint16_t) - 2] = (uint16_t)crc;
		a[VE_PAGE_SIZE/sizeof(uint16_t) - 1] = (uint16_t)(crc>>16);
		while(!flash_ready());
		if(getPageCRC(page)!=crc)
			ok = 0;
	}

	FLASH->CR &= ~(FLASH_CR_PG); //Запрещаем программирование флеша
	flash_lock();
	return ok;
}
