#include "stm32f10x_conf.h"
#include "stm32f10x.h"
//---------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//---------------------------------------------------------------------------
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//---------------------------------------------------------------------------
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
//---------------------------------------------------------------------------
#include "XBEE.h"
#include "ADC.h"
#include "flash.h"
//===========================================================================

//---------------------------------------------------------------------------
#define MAIN_LOOP_TIME 5
#define A_EMA  0.3333 	// 2/(N+1) N = 5 периодов
#define CHANNELS 6 		// 0-5 -каналы управления
#define BATT_CHANNEL 6	// 6 - канал статуса батареи
#define PARAMETERS_HEADER 0xA0B0A1B1 //Маркер начала блока параметров
//---------------------------------------------------------------------------
typedef struct 
{
	uint32_t header; //= PARAMETERS_HEADER; Заголовок нужен чтобы начало блока никода небыло равно 0xFFFFFFFF
	uint16_t ScaleFactor [CHANNELS];
	int8_t Transform [CHANNELS]; //= {1,-1,1,1,1,1};
	int16_t Calibration [3][CHANNELS];// Нижнее значение|Ноль|Верхнее значение
	uint16_t AircraftAddr;
	uint16_t GroundStationAddr;
}SaveDomain;


SaveDomain Parameters; //переменная типа SaveDomain для хранения параметров системы
gamepad_report_t gamePadReport;
char strResult [120];
uint8_t buttons = 0;
int16_t EMA [7];
uint8_t CalibrateMode = 0; // 0-NormalMode| 1-CalibrateMode
uint16_t BatteryLevel = 0;// ADC канал батареи считываться сразу после каналов управления
void * SaveParametersAddr;
uint8_t pr = 4;
uint8_t USB_connected = 0;
uint8_t PrintRawData = 0; 

__IO uint8_t PrevXferComplete = 1;

volatile xSemaphoreHandle XBMutex;

//+++++++++++++++++++++++++++++++Service functions+++++++++++++++++++
//---------------------------------------------------------------------------
int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//--------------------------------------------------------------------
uint32_t GetNextIntFromString (char ** cursor, char divider)
{
	char val [20];
	uint8_t cmdPointer = 0;
	val[0] = 0;
	//char * cursor = *cursor_ptr;
	
	while (**cursor)
	{
		if (**cursor != divider)
		{
			val[cmdPointer++] = **cursor;
			(*cursor)++;
		}
		else 
		{
			(*cursor)++;
			val[cmdPointer] = 0;
			
			return atoi(val);
		}
	}
	val[cmdPointer] = 0;
	return atoi(val);
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------

//--------------------------------------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int16_t Calibrate (int16_t val, uint8_t index)
{
	int16_t scaledVal = 0;
	scaledVal = val + Parameters.Calibration [1][index];
	if (scaledVal > 0) 
	{
		scaledVal = map (scaledVal,0,Parameters.Calibration [2][index],0,Parameters.ScaleFactor[index]);
		if (scaledVal > Parameters.ScaleFactor[index]) scaledVal = Parameters.ScaleFactor[index];
	}
	else 
	{
		scaledVal = map (scaledVal, Parameters.Calibration [0][index],0,-Parameters.ScaleFactor[index],0);
		if (scaledVal < -Parameters.ScaleFactor[index]) scaledVal = -Parameters.ScaleFactor[index];
	}
	scaledVal *= Parameters.Transform[index];
	return scaledVal;
}
//---------------------------------------------------------------------------
void PrintDataXBEE (int16_t * ar)
{
	char * strPointer;
	strResult [0] = 0;
	//h_size = xPortGetFreeHeapSize();

	strPointer = strResult;
	strPointer += sprintf (strPointer, "STE%c", ';');
	if (PrintRawData)
	{
		
		for (int i = 0;i<CHANNELS;i++)
		{
			strPointer += sprintf (strPointer, "%d;",ar[i]);
		}
	}
	else 
	{
		for (int i = 0;i<CHANNELS;i++)
		{

			strPointer += sprintf (strPointer, "%d;",Calibrate(ar[i],i));
		}
	}
	strPointer += sprintf (strPointer, "%d;",buttons);
	//strPointer += sprintf (strPointer, "%d;",BatteryLevel);
	//strPointer += sprintf (strPointer, "%d;",h_size);//debug - heap size

	*(strPointer++) = 10;
	*(strPointer++) = 13;
	*(strPointer++) = 0;
	xSemaphoreTake( XBMutex, portMAX_DELAY );
	XB_send_data (strResult, strlen (strResult), Parameters.AircraftAddr, 0);
	xSemaphoreGive( XBMutex );
}

//---------------------------------------------------------------------------
void PrintDataUSB (int16_t  * ar)
{
	gamePadReport.buttons = buttons;
	gamePadReport.Y = Calibrate (ar[0],0);
	gamePadReport.X = Calibrate (ar[1],1);
	gamePadReport.RX = Calibrate (ar[2],2);
	gamePadReport.Z = Calibrate (ar[3],3);
	gamePadReport.RY = Calibrate (ar[4],4);
	gamePadReport.RZ = Calibrate (ar[5],5);
	/*
	int16_t calib [CHANNELS];
	for (int i = 0;i<CHANNELS;i++)
	{
		calib[i] = Calibrate (ar[i],i);
	}
	*/
	Joystick_Send (&gamePadReport);
}

//---------------------------------------------------------------------------
void GetZeroLevel(void)
{
	for (int i=0;i<CHANNELS;i++)
	{
		Parameters.Calibration [1][i] = -EMA[i];
	}
}
//---------------------------------------------------------------------------
void ADC_receiver( void *pvParameters )
{
	uint32_t cur_adc [7];
	portTickType timer = 0;
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		ADC_read ();
		xQueueReceive (adc_queue, cur_adc, portMAX_DELAY); //Get data from Queue
		for (int i = 0;i<CHANNELS;i++)
		{
			EMA[i] = (int) (A_EMA*cur_adc[i] + (1-A_EMA)* EMA[i]);
			//EMA[i] = cur_adc[i];
			if (CalibrateMode)
			{
				if (EMA[i]+Parameters.Calibration[1][i] < Parameters.Calibration[0][i]) Parameters.Calibration[0][i] = EMA[i] + Parameters.Calibration[1][i];
				else if (EMA[i]+Parameters.Calibration[1][i] > Parameters.Calibration[2][i]) Parameters.Calibration[2][i] = EMA[i] + Parameters.Calibration[1][i];
			}
		}

		BatteryLevel = cur_adc[BATT_CHANNEL];
		if (!--pr)
		{
			if (USB_connected)  PrintDataUSB (EMA);
			PrintDataXBEE (EMA);
			pr = 4;
		}
		
		timer = xLastWakeTime;
		vTaskDelayUntil( &xLastWakeTime, ( MAIN_LOOP_TIME / portTICK_RATE_MS ) );
	}
}



void Blink1( void *pvParameters )
{
  	volatile int State=0;
	while( 1 ){

		 switch( State ){
			case 0: 
				GPIO_SetBits( GPIOB, GPIO_Pin_0 );
				State = 1;
				break;

			case 1:	
				GPIO_ResetBits( GPIOB, GPIO_Pin_0 );
				State = 0;
				break;
            default:
				State = 0;
				break;
		}
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}
//---------------------------------------------------------------------------
void ButtonsTask ( void *pvParameters )
{
	uint32_t pData = 0;
	while (1)
	{
		pData = (GPIOB->IDR >> 5)&0x1F;
		pData |= ((GPIOC->IDR >> 7)&0x10);
		buttons = pData;
		
		switch (buttons)
		{
			case 0x05://две левые - получить нуль
				GetZeroLevel();
				break;
			case 0x0A://две правые - калибровка
				if (CalibrateMode)
				{
					CalibrateMode = 0;
				}
				else
				{
					for (int i=0;i<CHANNELS;i++)
					{
						Parameters.Calibration [0][i] = 0x7FFF;
						Parameters.Calibration [2][i] = -0x7FFF;
					}
					CalibrateMode = 1;
				}
				break;
			case 0x0F: //четыре кнопки - сохранение
				SaveParametersAddr = FindNextAddr(sizeof (Parameters));
				WriteFlash(&Parameters, SaveParametersAddr, sizeof (Parameters));
				break;
			case 0x03: //Две верхние - подключить USB
				if (!USB_connected)
				{
					GPIO_ResetBits( GPIOA, GPIO_Pin_10);
					USB_connected = 1;
				}
				break;
			default:
				//Joystick_Send (buttons);
				break;
		
		}
		vTaskDelay( 5 / portTICK_RATE_MS );
	}
}
//---------------------------------------------------------------------------
//Command parcer task
void CommandParcer ( void *pvParameters )
{
	xb_message current_msg;
	char cmd [10];
	uint8_t cmdPointer;
	//char val [10];
	uint8_t valNum;
	char * globalPointer;
	//----Print values----
	char * strPointer;
	char strResCMD [120];
	//--------------------
	
	while(1)
	{
		xQueueReceive (xb_rx_msg_queue, &current_msg,portMAX_DELAY); //Get message from Queue
		
		//Values init
		cmd [0] = 0;
		valNum = 1;
		globalPointer = (char*) current_msg.msg;
		cmdPointer = 0;
		//Parse command string
		while (valNum)
		{
			if (*globalPointer == ';')
			{
				//если нашли разделитель, закрываем строку команды
				cmd [cmdPointer] = 0;
				cmdPointer = 0;
				valNum = 0;
			}
			else 
			{
				cmd [cmdPointer++] = *globalPointer;
			}
			globalPointer++;
			
		}
		
		//Parce command values
		if (!strcmp (cmd, "ZER")) //Calibration set zero level
		{
			GetZeroLevel();
		}
		else if (!strcmp (cmd,"SCA")) //start Calibration
		{
			CalibrateMode = 1;
			for (int i=0;i<CHANNELS;i++)
			{
				Parameters.Calibration [0][i] = 0x7FFF;
				Parameters.Calibration [2][i] = -0x7FFF;
			}
		}
		else if (!strcmp (cmd,"BCA")) //Break calibration
		{
			CalibrateMode = 0;
		}
		else if (!strcmp (cmd,"SPF")) //save parameters to flash
		{
			SaveParametersAddr = FindNextAddr(sizeof (Parameters));
			WriteFlash(&Parameters, SaveParametersAddr, sizeof (Parameters));
		}
		else if (!strcmp (cmd,"SSC")) //set scale factor
		{
			while ((*globalPointer) && (valNum < CHANNELS))
			{
				Parameters.ScaleFactor [valNum++] = GetNextIntFromString (&globalPointer, ';');
			}
		}
		else if (!strcmp (cmd,"GSC")) //get scale factor
		{
			strPointer = strResCMD;
			strPointer += sprintf (strPointer, "GSC%c", ';');
			while (valNum < CHANNELS)
			{
				strPointer += sprintf (strPointer, "%d;",Parameters.ScaleFactor[valNum++]);
			}
			*(strPointer++) = 10;
			*(strPointer++) = 13;
			*(strPointer++) = 0;
			xSemaphoreTake( XBMutex, portMAX_DELAY );
			XB_send_data (strResCMD, strlen (strResCMD), Parameters.GroundStationAddr, 5);
			xSemaphoreGive( XBMutex );
		}		
		else if (!strcmp (cmd,"SAA")) //set aircraft address
		{
			Parameters.AircraftAddr = GetNextIntFromString (&globalPointer, ';');
		}
		else if (!strcmp (cmd,"GAA")) //get aircraft address
		{
			strPointer = strResCMD;
			strPointer += sprintf (strPointer, "GAA%c", ';');
			strPointer += sprintf (strPointer, "%d;",Parameters.AircraftAddr);
			*(strPointer++) = 10;
			*(strPointer++) = 13;
			*(strPointer++) = 0;
			xSemaphoreTake( XBMutex, portMAX_DELAY );
			XB_send_data (strResCMD, strlen (strResCMD), Parameters.GroundStationAddr, 5);
			xSemaphoreGive( XBMutex );
		}
		else if (!strcmp (cmd,"SGA")) //set ground station address
		{
			Parameters.GroundStationAddr = GetNextIntFromString (&globalPointer, ';');
		}
		else if (!strcmp (cmd,"GGA")) //get ground station address
		{
			strPointer = strResCMD;
			strPointer += sprintf (strPointer, "GGA%c", ';');
			strPointer += sprintf (strPointer, "%d;",Parameters.GroundStationAddr);
			*(strPointer++) = 10;
			*(strPointer++) = 13;
			*(strPointer++) = 0;
			xSemaphoreTake( XBMutex, portMAX_DELAY );
			XB_send_data (strResCMD, strlen (strResCMD), Parameters.GroundStationAddr, 5);
			xSemaphoreGive( XBMutex );
		}

		vPortFree (current_msg.msg);
		current_msg.msg = NULL;
	}
}

//---------------------------------------------------------------------------
int main()
{
	XBMutex = xSemaphoreCreateMutex();

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE );

 	GPIO_InitTypeDef  GPIO_InitStructure;
	
	// Configure USB_CON Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	
	// Configure LED0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	

	//Xbee SLEEP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_SetBits( GPIOB, GPIO_Pin_12); // Sleep ON
	
	//+++++++++++++Button ports config+++++++++++++++++++++++
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	XB_init();
	ADC_InitProc();
	
	// init values============================================================
	Parameters.header = PARAMETERS_HEADER;
	
	SaveParametersAddr = findLastBlock (sizeof (Parameters));
	if (SaveParametersAddr)
	{
		memcpy (&Parameters,SaveParametersAddr,sizeof (Parameters));
	}
	else 
	{
		for (int i=0;i<CHANNELS;i++)
		{
			Parameters.Calibration [0][i] = 0x7FFF;
			Parameters.Calibration [1][i] = 0;
			Parameters.Calibration [2][i] = -0x7FFF;
			
			Parameters.ScaleFactor [i] = 1000;
			Parameters.Transform [i] = 1;
		}
		SaveParametersAddr = (void*) LAST_PAGE;
	}
	flash_unlock();
	
	//++++++++++ USB init++++++++++++
	USB_Interrupts_Config();
  
	Set_USBClock();
  
	USB_Init();

	//++++++++++++++++++++++++++++++++
	
	xTaskCreate( CommandParcer,(signed char *)"CommandParcer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( ADC_receiver,(signed char *)"ADC_receiver", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( ButtonsTask,(signed char *)"ButtonsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( Blink1,(signed char *)"Blink1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	
		//===========================================================================
	/* Start the scheduler. */
	vTaskStartScheduler();
  return 0;
}
