/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test.c
  摘 要:     API使用例程

  其 他:
  函数列表:

  当前版本: 1.0
  作 者:
  修改内容:
  完成日期: 2018年06月27日
*****************************************************************************/
#include "../include/byd/api_test.h"
#include <string.h>
#include <stdio.h>


/** Global variables **/
extern uint8_t  driving_mode;

/*****************************************************************************
  函数名称:       void BydAuto_DeveloperAPI_Start( void )
  说明:           开发者平台API初始化
  调用:
  被调用:
  输入参数:       void		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           启动接收任务和发送任务之前调用   
*****************************************************************************/
void BydAuto_DeveloperAPI_Start( void )
{
    BydAutoApiDataInit();
}

/*****************************************************************************
  函数名称:       void BydAuto_DeveloperAPI_ReceiveTest( PVCI_CAN_OBJ pCAN_Obj,
                                                         uint32_t objCount )
  说明:           开发者平台API的接收任务
  调用:
  被调用:
  输入参数:       pCAN_Obj  -- 接收报文对象	
                  objCount  -- 报文对象的个数		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           接收到CAN报文以后调用该函数   
*****************************************************************************/
void BydAuto_DeveloperAPI_ReceiveTest( PVCI_CAN_OBJ pCAN_Obj, uint32_t objCount )
{
    uint32_t loop;

    for (loop = 0; loop < objCount; loop++) {
        // CAN Receive Message Analysis
        BydAutoCANRuntimeRcv(pCAN_Obj[loop].ID,
                             pCAN_Obj[loop].DataLen,
                             pCAN_Obj[loop].Data);
    }
}


/*****************************************************************************
  函数名称:       int32_t BydAuto_DeveloperAPI_TransmitTest(
                                                      uint32_t msgId,
                                                      uint32_t Count,
                                                      PVCI_CAN_OBJ pCAN_Obj )
  说明:           开发者平台API的发送任务
  调用:
  被调用:
  输入参数:       pCAN_Obj  -- 发送报文对象		
  输出参数:       
  返回值:         bool值
  被访问变量:
  被修改变量:
  其他:           发送报文之前调用该函数   
*****************************************************************************/
int32_t BydAuto_DeveloperAPI_TransmitTest( uint32_t msgId,
                                           uint32_t Count,
                                           PVCI_CAN_OBJ pCAN_Obj )
{
    int32_t retVal = 0;
    
    memset(pCAN_Obj->Data, 0x0, 8);
    pCAN_Obj->DataLen = 0;
    pCAN_Obj->ExternFlag = 1;
    pCAN_Obj->RemoteFlag = 0;
    pCAN_Obj->ID = msgId;

    switch (msgId) {
    case BYD_AUTO_IDRIVING_CMD_0:
        {
            /* 10ms */
            BydAutoCANRuntimeXmt(pCAN_Obj->ID,
                                 (uint32_t *)&pCAN_Obj->DataLen,
                                 pCAN_Obj->Data);
            retVal = 1;
        }
        break;
    case BYD_AUTO_IDRIVING_CMD_1:
        {
           
            /* 20ms */
            if (0 == Count % 2) {
                BydAutoCANRuntimeXmt(pCAN_Obj->ID,
                                     (uint32_t *)&pCAN_Obj->DataLen,
                                     pCAN_Obj->Data);
                retVal = 1;
            }			

        }
        break;
    case BYD_AUTO_IDRIVING_CMD_2:
        {
            /* 20ms */
            if (0 == Count % 2) {
                BydAutoCANRuntimeXmt(pCAN_Obj->ID,
                                     (uint32_t *)&pCAN_Obj->DataLen,
                                     pCAN_Obj->Data);
                retVal = 1;
            }
        }
        break;
    case BYD_AUTO_IDRIVING_CMD_3:
        {
            /* 20ms */
            if (0 == Count % 2) {
                BydAutoCANRuntimeXmt(pCAN_Obj->ID,
                                     (uint32_t *)&pCAN_Obj->DataLen,
                                     pCAN_Obj->Data);
                retVal = 1;
            }
        }
        break;
    case BYD_AUTO_IDRIVING_CMD_4:
        {
            /* 20ms */
            if (0 == Count % 2) {
                BydAutoCANRuntimeXmt(pCAN_Obj->ID,
                                     (uint32_t *)&pCAN_Obj->DataLen,
                                     pCAN_Obj->Data);
                retVal = 1;
            }
        }
        break;
    case BYD_AUTO_IDRIVING_CMD_5:
        {
			/* 100ms */
            if (0 == Count % 10) {
                BydAutoCANRuntimeXmt(pCAN_Obj->ID,
                                     (uint32_t *)&pCAN_Obj->DataLen,
                                     pCAN_Obj->Data);
				retVal = 1;
			}
        }
        break;
    default:
        break;
    }

    return retVal;
}


/*****************************************************************************
  函数名称:       void BydAuto_DeveloperAPI_ControlTest( int8_t *pTestRq )
  说明:           开发者平台API的控制操作
  调用:
  被调用:
  输入参数:       testRq -- 测试请求		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口对整车进行控制，同时通过API接口获取整车状态值   
*****************************************************************************/
void BydAuto_DeveloperAPI_ControlTest( int8_t *pTestRq )
{
	uint8_t m_automode_eps_permission = 0;
	uint8_t m_automode_esp_permission = 0;  
	uint64_t nTimeStamp;
	if (0x1 != pTestRq[0]) {
        return;
    }
    
    switch (pTestRq[1]) {
    case 'g':
    case 'G':
        {
            driving_mode = NORMAL_MODE;
			printf("Developer API Get-func testing Go...\n");
            BydAuto_DeveloperAPI_GetTest();
        }
        break;

    case 's':
    case 'S':
        {
            driving_mode = NORMAL_MODE;
			printf("Developer API Set-func testing Go...\n");
            BydAuto_DeveloperAPI_SetTest();
        }
        break;
    case 'a':
    case 'A':
    	  {
 			if(ERR_BYD_AUTO_OK != GetESPAutoDrivePermissionInfo(&m_automode_esp_permission,&nTimeStamp))   //获取ESP激活状态
			{
				printf("State Acquisition：GetESPAutoDrivePermissionInfo error!\n");
			}           
			if(ERR_BYD_AUTO_OK != GetDriveAutoPermissionInfo(&m_automode_eps_permission,&nTimeStamp))      //获取EPS是否就绪
			{
				printf("State Acquisition：GetDriveAutoPermissionInfo error!\n");
			}			
			if(BYD_AUTO_DRIVE_ESP_PERMISSION_OK == m_automode_esp_permission)
			   //BYD_AUTO_EPS_PERMISSION_OK == m_automode_eps_permission                  //暂时屏蔽EPS 			                                                                    
			{
				if(BYD_AUTODRIVE_REQUEST_OK == DeveloperAPI_AutoDrive_ModeRequest())   
				{
             		driving_mode = AUTO_MODE;	
					printf("Developer API AutoDriving testing Go...\n");
					DeveloperAPI_AutoDrive_Control();
				}
				else
				{
             		printf("Automatic driving mode request failed！\n");		
				}
			}
			else
			{
				switch(m_automode_esp_permission)
				{
				case 0:
					printf("The ESP is not permission！\n");
					break;
				case 1:
					printf("The ESP is permission！\n");
					break;
				default:
					break;
				}

				switch(m_automode_eps_permission)
				{
				case 0:
					printf("The EPS is invalid!\n");
					break;
				case 1:
					printf("The EPS is ready!\n");
					break;
				case 2:
					printf("The EPS is not ready!\n");
					break;
				default:
					break;
				}
			}
    	  	break;
    	  }

    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void DeveloperAPI_TestItemDisplay( void )
  说明:           开发者平台API测试项显示
  调用:
  被调用:
  输入参数:       void		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           提示用户操作的打印语句  
*****************************************************************************/
void DeveloperAPI_TestItemDisplay( void )
{
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ESC to terminate testing.\n");
    printf("please enter 'g' or 'G' to start Get-func testing.\n");
    printf("please enter 's' or 'S' to start Set-func testing.\n");
    printf("please enter 'a' or 'A' to start AutoDrive testing.\n");    
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
}
