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
#include "main.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum
{
	Idle = 0,
	Gcode,
	Home,
	Calib
}CNC_Mode_usb;


extern AXIS x_axis;
extern AXIS y_axis;
extern AXIS z_axis;

extern DATA data;

extern uint8_t process_mode;
extern int thickness;
extern float I;
extern float J;

uint8_t _cncState = 4;

int temp;


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
	
	// memem set buff
	memset(data.ReceiveBuff, 0, 127);
	memset(data.Command, 0, 55);
	memset(data.TransBuff, 0, 45);
	
	
	USBD_CUSTOM_HID_HandleTypeDef* hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
	for(uint8_t i = 0; i < 127; i++)
	{
		data.ReceiveBuff[i] =  hhid -> Report_buf[i];
	}  
	memset(hhid ->Report_buf , 0, 127);
	
	// spilit command and data form GUI
	sscanf(data.ReceiveBuff, "%d %s ", &data.receive, data.Command);
	
	// check data from GUI, If false send NAK
	if( data.receive != data.need)
	{
		sprintf(data.TransBuff, "NAK %d_%d", data.need, data.receive);
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)data.TransBuff, 45);
		return(USBD_OK);
	}	
	
	// check command from data
	switch( data.Command[0])
	{
		// Gcode control motor
		case 'G':
			if(data.Command[1] == '0')
			{
				//sscanf(data.ReceiveBuff, "%d G0%u X%f Y%f",&temp, &temp, &x_axis.next, &y_axis.next);
				switch( data.Command[2])
					{
						case '0': //
							sscanf(data.ReceiveBuff, "%d G0%u X%f Y%f",&temp, &temp, &x_axis.next, &y_axis.next);
							_cncState = 0; 
							break;
						case '1': //
							sscanf(data.ReceiveBuff, "%d G0%u X%f Y%f",&temp, &temp, &x_axis.next, &y_axis.next);
							_cncState = 1;
							break;
						case '2': //
							sscanf(data.ReceiveBuff, "%d G0%u X%f Y%f Z%d I%f J%f" ,&temp, &temp, &x_axis.next, &y_axis.next, &temp, &I, &J);
							_cncState = 2;
							break;
						case '3': //
							sscanf(data.ReceiveBuff, "%d G0%u X%f Y%f Z%d I%f J%f" ,&temp, &temp, &x_axis.next, &y_axis.next, &temp, &I, &J);
							_cncState = 3;
							break;
					}	
			 process_mode = Gcode; // mode gcode control motor
			}
			else
				sprintf(data.TransBuff, "ACK R %d_RESUME", data.receive);
			break;			
		// command control Start and Stop CNC
		case 'S':
			switch(data.Command[2])
			{
				case 'R':
					sprintf(data.TransBuff, "ACK R %d_START", data.receive);
					break;
				case 'P':
					sprintf(data.TransBuff, "ACK STP %d_STOP", data.receive);
					data.need = -1;
					break;
			}
			break;
		// command goto HOME
		case 'H':
				x_axis.home = false;
				y_axis.home = false;
				z_axis.home = false;
				
				sscanf(data.ReceiveBuff, "%d H %d", &temp, &thickness);
				process_mode = Home; // mode goto home
				break;
			
		// command set PID 
		case 'I':
			switch( data.Command[1])
			{
				case 'X':
					sscanf(data.ReceiveBuff, "%d IX %f %f %f",&temp, &x_axis.Kp, &x_axis.Ki, &x_axis.Kd);
					sprintf(data.TransBuff, "ACK F %d_PID X %.2f %.4f %.2f",data.receive, x_axis.Kp, x_axis.Ki, x_axis.Kd);
					break;
				case 'Y':
					sscanf(data.ReceiveBuff, "%d IY %f %f %f",&temp, &y_axis.Kp, &y_axis.Ki, &y_axis.Kd);
					sprintf(data.TransBuff, "ACK F %d_PID Y %.2f %.4f %.2f",data.receive, y_axis.Kp, y_axis.Ki, y_axis.Kd);
					break;
				case 'Z':
					sscanf(data.ReceiveBuff, "%d IZ %f %f %f",&temp, &z_axis.Kp, &z_axis.Ki, &z_axis.Kd);
					sprintf(data.TransBuff, "ACK F %d_PID Z %.2f %.4f %.2f",data.receive, z_axis.Kp, z_axis.Ki, z_axis.Kd);
					break;
			}
			break;
		// command pause CNC
		case 'P': 
			sprintf(data.TransBuff, "ACK P %d_PAUSE", data.receive);
			break;
		// command control Resume CNC
		case 'R':		
			sprintf(data.TransBuff, "ACK R %d_RESUME", data.receive);
			break;
		// command calib 
		case 'C':
			sscanf(data.ReceiveBuff, "%d C %f", &temp, &z_axis.next);
			process_mode = Calib; // mode calib
			break;
		// skip other Gcode		
		default:
			sprintf(data.TransBuff, "ACK R %d_SKIP", data.receive);
			break;
	}

	// check process for skip send ack
	if( process_mode != Idle)
		return(USBD_OK);
	// increase recieve count
	data.need++;
	// if true send ACK
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)data.TransBuff, 45);
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

