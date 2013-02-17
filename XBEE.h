#ifndef __XBEE_h
#define __XBEE_h
#include "stm32f10x.h"
#include "queue.h"

/*
#ifdef XBEE_USART2
	#define XB_UART 				USART2
	#define XB_BAUD_RATE				57600
	#define XB_UART_GPIO_TX         GPIO_Pin_2
	#define XB_UART_GPIO_RX         GPIO_Pin_3
	#define XB_UART_GPIO            GPIOA
	#define XB_RCC_APBPeriph_UART   RCC_APB1Periph_USART2
	#define XB_UART_TX_DMA          DMA1_Channel7
	#define XB_UART_RX_DMA          DMA1_Channel6
	
#endif
*/
#define XB_BAUD_RATE 38400
#define RX_QUEUE_LEN 5
#define TX_QUEUE_LEN 5
typedef struct 
{
	uint16_t lenght;
	uint8_t * frame_array;
} xb_frame;
typedef struct 
{
	uint8_t * msg;
	uint8_t lenght;
	uint8_t type;
	//uint32_t addr_MSB_32;
	//uint32_t addr_LSB_32;
	uint16_t address;
	uint8_t rssi;
	uint8_t options;
	uint8_t retries;
} xb_message;

extern xQueueHandle xb_rx_msg_queue;
//extern xQueueHandle xb_tx_msg_queue;
extern  uint16_t dropped_frames;

void XB_init (void);
void XB_send_data (char *ar, int len, uint16_t addr, uint8_t retries);
#endif