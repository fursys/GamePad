/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
//#include "platform_config.h"
#include "stm32f10x.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
typedef struct 
{
	uint16_t buttons;
	int8_t X;
	int8_t Y;
	int8_t Z;
	int8_t RX;
	int8_t RY;
	int8_t RZ;
}gamepad_report_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
  #define USB_DISCONNECT                      GPIOA  
  #define USB_DISCONNECT_PIN                  GPIO_Pin_9



#define CURSOR_STEP     20

#define DOWN            8
#define LEFT            1
#define RIGHT           2
#define UP              4

/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void GPIO_AINConfig(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void Joystick_Send(gamepad_report_t * gpRep);
uint8_t JoyState(void);
void Get_SerialNum(void);

#endif  /*__HW_CONFIG_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
