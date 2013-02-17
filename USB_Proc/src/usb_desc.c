/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Descriptors for Joystick Mouse Demo
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


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t Joystick_DeviceDescriptor[JOYSTICK_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize 64*/
    0x83,                       /*idVendor (0x0483)*/
    0x04,
    0x10,                       /*idProduct = 0x5710*/
    0x57,
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    1,                          /*Index of string descriptor describing
                                                  manufacturer */
    2,                          /*Index of string descriptor describing
                                                 product*/
    3,                          /*Index of string descriptor describing the
                                                 device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* Joystick_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t Joystick_ConfigDescriptor[JOYSTICK_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    JOYSTICK_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x03,         /*bNumInterfaces: 3 interfaces*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
                                     the configuration*/
    0xE0,         /*bmAttributes: bus powered */
    0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

    /************** Descriptor of Joystick Mouse interface ****************/
    /* 09 */
    0x09,         /*bLength: Interface Descriptor size*/
    0x04,         /*bDescriptorType: Interface descriptor type*/
    0x00,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x01,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of Joystick Mouse HID ********************/
    /* 18 */
    0x09,         /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x00,         /*bcdHID: HID Class Spec release number*/
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    JOYSTICK_SIZ_REPORT_DESC,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of Joystick Mouse endpoint ********************/
    /* 27 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x81,          /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    0x40,          /*wMaxPacketSize: 64 Byte max */
    0x00,
    0x20,          /*bInterval: Polling Interval (32 ms)*/
	
	/******************** Descriptor of COM port Communication interface ********************/
    /* 34 */
    0x09,         /*bLength: Interface Descriptor size*/
    0x04,         /*bDescriptorType: Interface descriptor type*/
    0x01,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x01,         /*bNumEndpoints*/
    0x02,         /*bInterfaceClass: CDC*/
    0x02,         /*bInterfaceSubClass : abstract control model*/
    0x02,         /*nInterfaceProtocol : Protocol: V.25ter (AT commands)*/
    0,            /*iInterface: Index of string descriptor*/
	/* 43 */
    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,
	/* 48 */
    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */
	/* 53 */
    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */
	/* 57 */
    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */
	/* 62 */
    /*Endpoint 2 Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    0x05,   /* bDescriptorType: Endpoint */
    0x82,   /* bEndpointAddress: (IN2) */
    0x03,   /* bmAttributes: Interrupt */
    0x08,      /* wMaxPacketSize: */
    0x00,
    0xFF,   /* bInterval: */
	/* 69 */
    /******************** Data class interface descriptor ********************/
    0x09,   /* bLength: Endpoint Descriptor size */
    0x04,  /* bDescriptorType: */
    0x02,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */
	/* 78 */
    /*Endpoint 3 Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    0x05,   /* bDescriptorType: Endpoint */
    0x03,   /* bEndpointAddress: (OUT3) */
    0x02,   /* bmAttributes: Bulk */
    0x40,             /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: ignore for Bulk transfer */
	/* 85 */
    /*Endpoint 3 Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    0x05,   /* bDescriptorType: Endpoint */
    0x84,   /* bEndpointAddress: (IN4) */
    0x02,   /* bmAttributes: Bulk */
    0x40,             /* wMaxPacketSize: */
    0x00,
    0x00    /* bInterval */
	/* 92 */
  }
  ; 
  
const uint8_t  Joystick_ReportDescriptor[JOYSTICK_SIZ_REPORT_DESC] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x10,                    //     USAGE_MAXIMUM (Button 16)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x10,                    //     REPORT_COUNT (16)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x32,                    //     USAGE (Z)
    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,                    //     USAGE (Ry)
    0x09, 0x35,                    //     USAGE (Rz)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x06,                    //     REPORT_COUNT (6)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};//GamePad descriptor



/* USB String Descriptors (optional) */
const uint8_t Joystick_StringLangID[JOYSTICK_SIZ_STRING_LANGID] =
  {
    JOYSTICK_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t Joystick_StringVendor[JOYSTICK_SIZ_STRING_VENDOR] =
  {
    JOYSTICK_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };

const uint8_t Joystick_StringProduct[JOYSTICK_SIZ_STRING_PRODUCT] =
  {
    JOYSTICK_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'J', 0,
    'o', 0, 'y', 0, 's', 0, 't', 0, 'i', 0, 'c', 0, 'k', 0
  };
uint8_t Joystick_StringSerial[JOYSTICK_SIZ_STRING_SERIAL] =
  {
    JOYSTICK_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

