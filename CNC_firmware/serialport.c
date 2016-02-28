#include "serialport.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "hardware_config.h"

// пиздим отсюда http://microtechnics.ru/stm32-uchebnyj-kurs-usart/
// и отсюда http://easystm32.ru/interfaces/16-uart-in-stm32-part-2

#define SERIALPORT_BAUDRATE 115200
#define USBHOSTSERIALPORT_BAUDRATE 57600

char serialport_send_zero[] = "\0";
char *serialport_stringto_send_async = serialport_send_zero;
#define SERIALPORT_READ_BUF_SIZE 640
char serialport_readbuffer[SERIALPORT_READ_BUF_SIZE];
int serialport_readbuffer_pointer = 0;
char hostserialport_readbuffer[SERIALPORT_READ_BUF_SIZE];
int hostserialport_readbuffer_pointer = 0;

void serialport_init()
{
	GPIO_InitTypeDef port;
	USART_InitTypeDef usart;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(getRcc(USBUARTPORT), ENABLE);
    //Пины PA9 и PA10 в режиме альтернативных функций –
    //Rx и Tx USART’а
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = USBUARTTXPIN;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(USBUARTPORT, &port);

    port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    port.GPIO_Pin = USBUARTRXPIN;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(USBUARTPORT, &port);

    //Настройка USART, все поля оставляем дефолтными, кроме скорости обмена
    USART_StructInit(&usart);
    usart.USART_BaudRate = SERIALPORT_BAUDRATE;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    //Запускаем сам USART
    USART_Cmd(USART1, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 10);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(getRcc(USBHOSTUARTPORT), ENABLE);
	//Пины PB10 и PB11 в режиме альтернативных функций –
	//Rx и Tx USART’а
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = USBHOSTUARTTXPIN;
	port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(USBHOSTUARTPORT, &port);

	port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	port.GPIO_Pin = USBHOSTUARTRXPIN;
	port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(USBHOSTUARTPORT, &port);

	//Настройка USART, все поля оставляем дефолтными, кроме скорости обмена
	USART_StructInit(&usart);
	usart.USART_BaudRate = USBHOSTSERIALPORT_BAUDRATE;
	USART_Init(USART3, &usart);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_TC, DISABLE);
	//Запускаем сам USART
	USART_Cmd(USART3, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_SetPriority(USART3_IRQn, 10);

}

void sendchar_asyncprivate()
{
	if(*serialport_stringto_send_async){
		USART1->DR=(u16)*serialport_stringto_send_async;
		serialport_stringto_send_async++;
	} else {
		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
		serialport_stringto_send_async = serialport_send_zero;
	}
}

char serialposr_async_done() // использовать как бул
{
	return *serialport_stringto_send_async == 0;
}

void serialposr_sendstring_async(char *string) // строчка должна быть не измена пока не отправит её
{
	serialport_stringto_send_async = string;
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	if(USART1->SR & USART_SR_TC)
		sendchar_asyncprivate();
}

void serialposr_sendstring_sync(char *string)
{
	while(*string) {
		while(!(USART1->SR & USART_SR_TC));
		USART1->DR=(u16)*string;
		string++;
	}
}

void hostserialposr_sendstring_sync(const char *string)
{
	while(*string) {
		while(!(USART3->SR & USART_SR_TC));
		USART3->DR=(u16)*string;
		string++;
	}
}

void USART1_IRQHandler()
{
	if(USART1->SR & USART_SR_RXNE){ // читаем
			char data = USART1->DR;
			USART1->SR &= (uint16_t)~USART_SR_RXNE;
			int overfill = serialport_readbuffer_pointer > (SERIALPORT_READ_BUF_SIZE-2);
			if(overfill || data=='\0' || data=='\r' || data=='\n') {
				if(serialport_readbuffer_pointer>0) {
					serialport_readbuffer[serialport_readbuffer_pointer] = 0;
					serial_port_recived(serialport_readbuffer);
					serialport_readbuffer_pointer = 0;
				}
				if(!overfill)
					return;
			}
			if(((u8)data)>=0x20) { //читаемые символы
				serialport_readbuffer[serialport_readbuffer_pointer] = data;
				serialport_readbuffer_pointer++;
			}
	} else if(USART1->SR & USART_SR_TC){ // пишем
		USART1->SR &= (uint16_t)~USART_SR_TC;
		sendchar_asyncprivate();
	}
	if(USART1->SR & USART_SR_ORE)
		USART1->SR &= (uint16_t)~USART_SR_ORE;
}

void USART3_IRQHandler()
{
	if(USART3->SR & USART_SR_RXNE){ // читаем
			char data = USART3->DR;
			USART3->SR &= (uint16_t)~USART_SR_RXNE;
			int overfill = hostserialport_readbuffer_pointer > (SERIALPORT_READ_BUF_SIZE-2);
			if(overfill || data=='\0' || data=='\r' || data=='\n') {
				if(hostserialport_readbuffer_pointer>0) {
					hostserialport_readbuffer[hostserialport_readbuffer_pointer] = 0;
					hostserial_port_recived(hostserialport_readbuffer);
					hostserialport_readbuffer_pointer = 0;
				}
				if(!overfill)
					return;
			}
			if(((u8)data)>=0x20) { //читаемые символы
				hostserialport_readbuffer[hostserialport_readbuffer_pointer] = data;
				hostserialport_readbuffer_pointer++;
			}
	} else if(USART3->SR & USART_SR_TC){ // пишем
		USART3->SR &= (uint16_t)~USART_SR_TC;
	}
	if(USART3->SR & USART_SR_ORE)
		USART3->SR &= (uint16_t)~USART_SR_ORE;
}
