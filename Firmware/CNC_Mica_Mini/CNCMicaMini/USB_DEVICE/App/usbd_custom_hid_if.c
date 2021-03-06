/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */

char ReceiveBuff[27];
uint8_t Resume[] = "R";
uint8_t Stop[] = "STP";

bool debug_flag = false;
uint8_t process_mode = 0;

uint8_t _cncState = 4;
uint8_t _poscontrol = 0;

int temp;

float X_next = 0;
float Y_next = 0;
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
		0x06, 0x00, 0xff,              // 	USAGE_PAGE (Vendor defined page 1)
		0x09, 0x01,                    // 	USAGE (Vendor Usage 0x01)
    // System Parameters
    0xa1, 0x01,                    // 	COLLECTION (Application)
    0x85, 0x02,                    //   REPORT_ID (2)
    0x75, 0x08,                    //   REPORT_SIZE (8 bits = 1 byte)
    0x95, 0x40, 	                     //   REPORT_COUNT (64)
    0x09, 0x01,                    //   USAGE (Vendor Usage 0x01)
    0x91, 0x82,                    //   OUTPUT ((Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)
 
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x04,                    //   USAGE (Vendor Usage 0x04)
    0x75, 0x08,                    //   REPORT_SIZE (8 bits = 1 byte)
    0x95, 0x40, 	                     //   REPORT_COUNT (64)
    0x81, 0x82,											// (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
	USBD_CUSTOM_HID_HandleTypeDef* hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
	for(uint8_t i = 0; i < 27; i++)
	{
		ReceiveBuff[i] =  hhid -> Report_buf[i];
	}  
	memset(hhid ->Report_buf , 0, 65);
	switch( ReceiveBuff[0])
	{
		// set home position
		case 'H': 
			
			break;
		// control x y z motor
		case 'T':
			switch(ReceiveBuff[1])
			{
				case '1': // x up
					_poscontrol = 1;
					break;					
				case '2': // x down
					_poscontrol = 2;
					break;
				case '3': // y up
					_poscontrol = 3;
					break;
				case '4': // y down
					_poscontrol = 4;
					break;
				case '5': // z up
					_poscontrol = 5;
					break;
				case '6': // z down
					_poscontrol = 6;
					break;
			}
			process_mode = 1; // mode control position motor
			break;
		// Gcode control motor
		case 'G':
			if(( ReceiveBuff[1] == '0') && (ReceiveBuff[4] == 'X'))
			{
				sscanf(ReceiveBuff, "G0%u X%f Y%f", &temp, &X_next, &Y_next);
				switch( ReceiveBuff[2])
					{
						case '0': //
							_cncState = 0; 
							break;
						case '1': //
							_cncState = 1;
							break;
						case '2': //
							_cncState = 2;
							break;
						case '3': //
							_cncState = 3;
							break;
					}	
			 process_mode = 2; // mode gcode control motor
			}
			else
				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Resume, 1);
			break;			
		// command control Start and Stop CNC
		case 'S':
			switch(ReceiveBuff[2])
			{
				case 'R':
					USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Resume, 1);
					break;
				case 'P':
					USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Stop, 3);
					break;
			}
			break;
		// command control Resume CNC
		case 'R':
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Resume, 1);
			break;
		// command on/off debug
		case 'D':
			switch( ReceiveBuff[2])
			{
				case '0':
					debug_flag = false;
					break;
				case '1':
					debug_flag = true;
					break;				
			}
			break;
		// skip other Gcode		
		case 0xD:
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Resume, 1);	
			break;
	}
	memset(ReceiveBuff, 0, 27);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

