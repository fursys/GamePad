#ifndef __ADC_h
#define __ADC_h
#include "stm32f10x.h"
#include "queue.h"

#define ADC_QUEUE_LEN 2

extern xQueueHandle adc_queue;
void ADC_InitProc (void);
void ADC_read (void);


#endif