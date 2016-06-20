/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : AppLayer.h
* Author             : IMS Systems Lab
* Version            : V1.0.0
* Date               : 08/25/2009
* Description        : Entry point for APP layer and relative API;
*                      This file contains App defines and app global variables also
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
#ifndef __APP_LAYER_H
#define __APP_LAYER_H

/* Includes ------------------------------------------------------------------*/
#include "AppTypes.h"

// TRACE used to send debug message on Terminal I/O
#ifdef __IAR_SYSTEMS_ICC__    
    #ifdef APP_TRACE_ON
        #include <stdio.h>
        #define APP_TRACE(...) printf(__VA_ARGS__)        
    #else
        #define APP_TRACE(...) 
    #endif
#else
    #define APP_TRACE(...) 
#endif

//Application Frame Type
enum
{
  APP_DATA_REQ = 0,
  APP_DATA_RESP,
  APP_CHANGE_CONF,
  APP_RESET_SYS,
  APP_SWITCHOFF = 0xAA,
  APP_SWITCHON = 0xBB  
};


//E-Meter working Mode
typedef enum
{
    WM_Network  = 0,
    WM_Standalone,
    WM_Calibration
} WorkingModeType;

//E-Meter Working Status
typedef enum
{
    WS_Startup = 0,
    WS_DataReportingOn,
    WS_DataReportingOff,
    WS_Error      
} WorkingStatusType;

// Application Status
typedef struct {
    WorkingModeType   WorkingMode;
    WorkingStatusType WorkingStatus;    
    APP_Boolean       IsChanged;
} AppliStatusType;


/*Application Vm */
typedef struct{
u8 Nodo_I[2];
char* IDN;
}DATASTAMP_T;    

enum{
  UNJOINED = 0,
  JOINED,
  UPDATED,
  UPDATERROR  
};

typedef enum{
  IDLE = 0,
  SYS_RESET, 
  READ_DATA, 
  CHANGE_CHANNEL,
  LOAD_NUM,
  LOAD_LIST,
  LOAD_NUM_FROM_ST,
  LOAD_LIST_FROM_ST,
  STARTUP,
  SWITCH_OFF,
  SWITCH_ON,
  SWITCH_OFF_ONE,
  SWITCH_ON_ONE
}SERIAL_ST;

typedef enum{
  DATA_R = 0,
  DATA_I, 
  LOOP_AD
}UPDATE_ST;

typedef struct{ 
DATASTAMP_T Element[24];
}NT_DATSA_T;

typedef struct{
u8 Address[6];
u16 Id;
u8 Status;
}STRING_NODE_T;   


typedef struct{
u16 Max_JB_Num;  
u8 BlockUpdate;
STRING_NODE_T StrindNode[450];
}NODE_TABLE_T;   


//Application Extended Protocol Command
enum
{
  RESET_R = 0xA0,
  MIB_W_R = 0xB0,
  MIB_R_R = 0xB4,
  MIB_E_R = 0xB8,
  DL_DATA_R = 0xC0,
  DL_DATA_I = 0xC2,
  DL_SNIFF_I = 0xCA,
  SS_DATA_R = 0xC4,
  SS_DATA_I = 0xC6,
  SS_SNIFF_I = 0xCE,
};
typedef struct {
   u8 Address[6];
   u8 Join_State;
}ADDRESS_T;

typedef struct {
   u8 AccessLayer;
   u8 RepSet;
   u8 ModemConf[13];
   ADDRESS_T SlaveADDConn[5];
   u8 SlaveData[12];
}APPLICATION_MIB;
enum
{
  NO_ACTION = 0,
  MIB_READ_ON = 1,
  MIB_WRITE_ON = 2,
};
/* Private define ------------------------------------------------------------*/


void    AppTasksInit          (void);
void    AppHardwareInit       (void);
extern void f_ApplicationLayer(void);
#endif /*__APP_LAYER_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
