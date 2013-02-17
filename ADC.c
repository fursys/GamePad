#include<stdlib.h>
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "ADC.h"

xQueueHandle adc_queue;
uint32_t adc_buffer[7];

//============================================================================================
void DMA1_Channel1_IRQHandler (void)//ADC DMA IRQ handler
{
  //���� ����� ��������
  if(DMA1->ISR & DMA_ISR_TCIF1) 
  { 
	//���-�� ������
	portBASE_TYPE q_res = pdFALSE; //���������� ��� �������� ���������� ���������� � �������
	
    DMA1->IFCR = DMA_IFCR_CTCIF1; //������� ��� ����������
	
	
	//������ ��������� � ������� ����������� ������
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	q_res = xQueueSendToBackFromISR(adc_queue, &adc_buffer, &xHigherPriorityTaskWoken);
	/* ��� ������������ ������-����������. ��� ���� ��������� ������-����������� ���� ���������� ������������� � ������ ������ ������������� ������. ������� ����������� �������� ������������� - ��� �� �������� ����, ��� ����� ���������� ����������� ���������� ���������� ������� ������ ����������.*/
	/* ������, ����������� ������������ ���������. �� ������ ���������� ��� ������� ����� ���� ������! */
	
	if (q_res != pdPASS) //���� ������� ������, ����������� ����� ������ �������� � ������ �����
	{
		xQueueReceiveFromISR (adc_queue, NULL, 0);
		xQueueSendToBackFromISR(adc_queue, &adc_buffer, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);

  }      
 
  //���� �������� �������� ������
  if(DMA1->ISR & DMA_ISR_HTIF1) 
  { 
	DMA1->IFCR |= DMA_IFCR_CHTIF1; //������� ��� ����������
  }      //���-�� ������
 
  //���� ��������� ������ ��� ������
  if(DMA1->ISR & DMA_ISR_TEIF1) 
  { 
	DMA1->IFCR |= DMA_IFCR_CTEIF1; //������� ��� ����������
  }      //���-�� ������
  
  DMA1->IFCR |= DMA_IFCR_CGIF1; //������� ��� ����������� ����������
}

//********************************************************************************
//Function: ������������� DMA ��� ������ � ADC 						          //
//********************************************************************************
void ADC_DMA_Init(void)
{
 //�������� ������������ DMA1
 if ((RCC->AHBENR & RCC_AHBENR_DMA1EN) != RCC_AHBENR_DMA1EN)
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
 //������ ����� ��������� � ��������� � ���������� ������ ��� ������
 DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;   //����� �������� ���������
 DMA1_Channel1->CMAR  = (uint32_t)adc_buffer;   	//����� ������ � ������
 DMA1_Channel1->CNDTR =  7;                      //���������� ������ ��� ������
 //----------------- ����������� � ��������� ������������  ----------------
 //��������� �������� ����� ���������� � ���� ������� (������� ��� �����������)
 DMA1_Channel1->CCR   =  0;									//����������� �������� ������������
 //DMA1_Channel1->CCR  &= (uint16_t) (~DMA_CCR1_CIRC);		//��������� ����������� �����
 DMA1_Channel1->CCR  |= DMA_CCR1_CIRC;						//�������� ����������� �����
 DMA1_Channel1->CCR  &= (uint16_t) (~DMA_CCR1_DIR);         //�����������: ������ � ������
 //��������� ������ � ������������ �����������
 //DMA1_Channel2->CCR  &= (uint16_t)(~DMA_CCR7_PSIZE); 		//����������� ������ 8 ���
 DMA1_Channel1->CCR   |= DMA_CCR1_PSIZE_1;          		//����������� ������ 16 ���
 DMA1_Channel1->CCR   &= (uint16_t)(~DMA_CCR1_PINC);		//�� ������������ ��������� ���������
 //��������� ������ � �������
 DMA1_Channel1->CCR   |= DMA_CCR1_MSIZE_1;					//����������� ������ 16 ���
 DMA1_Channel1->CCR  |=  DMA_CCR1_MINC;						//������������ ��������� ���������

 //��������� ���������� �� ���������� ������:
 DMA1_Channel1->CCR |= DMA_CCR1_TCIE;						//����� 1
 NVIC_EnableIRQ (DMA1_Channel1_IRQn);						//��������� ���������� �� DMA
 NVIC_SetPriority (DMA1_Channel1_IRQn, 15);
 ADC1->CR2         |=  ADC_CR2_DMA;							//��������� �������� ADC ����� DMA
 DMA1_Channel1->CCR  |=  DMA_CCR1_EN;      //��������� ������ ������
}
//============================================================================================

void ADC_read (void)
{
	// Start the conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC_InitProc (void)
{

	adc_queue = xQueueCreate (ADC_QUEUE_LEN,sizeof(adc_buffer));

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1, ENABLE );
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	
		/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitTypeDef  ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 7;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_55Cycles5);
  
	/* Enable ADC1  */
	ADC_Cmd(ADC1, ENABLE);
	
		/* Enable ADC1 reset calibration register */   
	ADC_ResetCalibration(ADC1);

	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);

	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_DMA_Init();
}