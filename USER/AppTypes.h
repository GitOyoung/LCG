/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : APP_Types.h
* Author             : IMS Systems Lab
* Version            : V1.1.0
* Date               : 08/25/2009
* Description        : This file contains all the specific Application types
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED 
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __APP_TYPES_H
#define __APP_TYPES_H

#include "stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
// NULL pointer
#define APP_NULL ((void *)0)
// Application types
typedef enum { APP_FALSE   = 0, APP_TRUE  = !APP_FALSE  } APP_Boolean;
typedef enum { APP_SUCCESS = 0, APP_ERROR = !APP_SUCCESS} APP_ErrStatus;

/* Application Event type */
typedef enum
{   EVENT_NONE =0,
    EVENT_JOY_SEL,   /* Joystick selection */
    EVENT_JOY_UP,    /* Joystick move up */
    EVENT_JOY_DOWN,  /* Joystick move down */
    EVENT_JOY_LEFT,  /* Joystick move left */
    EVENT_JOY_RIGHT, /* Joystick move rightp */
    EVENT_BTN_PRESS  /* Button presses */
} AppEventType;

/* Application Status type */
typedef enum
{   STATUS_CONFIG =0,  /* Configuration/Calibration mode */
    STATUS_RUNNING     /* Normal Running mode */    
} AppStatusType;


#endif /*__APP_TYPES_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
