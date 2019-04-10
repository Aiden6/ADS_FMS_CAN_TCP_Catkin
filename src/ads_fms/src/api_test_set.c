/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test_set.c
  摘 要:     API使用例程

  其 他:
  函数列表:

  当前版本: 1.0
  作 者:
  修改内容:
  完成日期: 2018年06月27日
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include "../include/byd/api_test.h"


/** Global variables **/
extern int32_t  gTestFlag; 
extern int8_t   gCharInput[STRING_LEN];
extern sem_t    sem_event;



/*****************************************************************************
  函数名称:       void DeveloperAPI_SetTestItemDisplay( void )
  说明:           开发者平台API的Set相关条目测试打印信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           用于提示用户进行相关操作  
*****************************************************************************/
void DeveloperAPI_SetTestItemDisplay( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Otherwise, enter:\n");
    printf("s1  or S1:  SendAutoDriveModeRequest(uint8_t nModeRequest)\n");
    printf("s2  or S2:  SetLampAutoSwitch(uint8_t nAutoSwitch)\n");
    printf("s3  or S3:  SetLampTurnSwitch(uint8_t nLeft, uint8_t nRight)\n");
    printf("s4  or S4:  SetLampBrakeSwitch(uint8_t nBrakeSwitch)\n");
    printf("s5  or S5:  SetAngular(uint16_t nAngular)\n");
    printf("s6  or S6:  SetEpbState(uint8_t nEpbState)\n");
    printf("s7  or S7:  SetRainWiper(uint8_t nRainWiper)\n");
    printf("s8  or S8:  SetCarGear(uint8_t nCarGear)\n");
    printf("s9  or S9:  SetDriveModeFeedback(uint8_t nDriveModeFeedback)\n");
    printf("s10 or S10: SetAcceleratedVelocity(uint8_t nAcceleratedVelocity)\n");
    printf("s11 or S11: SetDriveAutoState(uint8_t nDriveAutoState)\n");
    printf("s12 or S12: SendAliveCount(uint8_t nCount)\n");
    printf("s13 or S13: SetLampEmergencyWarningSwitch(uint8_t nSwitch)\n");
    printf("s14 or S14: SetLampOutSideSwitch(uint8_t nArea, uint8_t nSwitch)\n");
    printf("s15 or S15: SetHornSwitch(uint8_t nSwitch)\n");
    printf("s16 or S16: SetMultimediaDisplaySwitch(uint8_t nSwitch)\n");
	printf("s17 or S17: SetAutoDriveLateralState(uint8_t nAutoDriveLateralState)\n");
}

/*****************************************************************************
  函数名称:       void DeveloperAPI_TestSetFunc1_9( void )
  说明:           开发者平台API的Set接口1～9
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口对整车上各执行机构进行控制   
*****************************************************************************/
void DeveloperAPI_TestSetFunc1_9(int8_t Index)
{
    switch (Index) {
    case '1': 
        {
            /* send auto drive mode request */
            printf("call SendAutoDriveModeRequest(uint8_t nModeRequest);\n");
            SendAutoDriveModeRequest_TestDemo();
        }
        break;
    case '2':
        {
            /* set lamp-AUTO request */
            printf("call SetLampAutoSwitch(uint8_t nAutoSwitch);\n");
            SetLampAutoSwitch_TestDemo();
        }
        break;
    case '3':
        {
            /* set lamp-Turn request */
            printf("call SetLampTurnSwitch(uint8_t nLeft, uint8_t nRight);\n");
            SetLampTurnSwitch_TestDemo();
        }
        break;
    case '4':
        {
            /* set lamp-Brake information request */
            printf("call SetLampBrakeSwitch(uint8_t nBrakeSwitch);\n");
            SetLampBrakeSwitch_TestDemo();
        }
        break;
    case '5':
        {
            /* set steering wheel angle request */
            printf("call SetAngular(uint16_t nAngular);\n");
            SetAngular_TestDemo();
        }
        break;
    case '6':
        {
            /* set epb-status request */
            printf("call SetEpbState(uint8_t nEpbState);\n");
            SetEpbState_TestDemo();
        }
        break;
    case '7':
        {
            /* set rain-wiper request */
            printf("call SetRainWiper(uint8_t nRainWiper);\n");
            SetRainWiper_TestDemo();
        }
        break;
    case '8':
        {
            /* set car-gear request */
            printf("call SetCarGear(uint8_t nCarGear);\n");
            SetCarGear_TestDemo();
        }
        break;
    case '9':
        {
            /* set ACU's drive mode feedback */
            printf("call SetDriveModeFeedback(uint8_t nDriveModeFeedback);\n");
            SetDriveModeFeedback_TestDemo();
        }
        break;
    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void DeveloperAPI_TestSetFunc10_17( void )
  说明:           开发者平台API的Set接口10～16
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口对整车上各执行机构进行控制   
*****************************************************************************/
void DeveloperAPI_TestSetFunc10_17(int8_t Index)
{
    switch (Index) {
    case '0':
        {
            /* set acceleration velocity */
            printf("call SetAcceleratedVelocity(uint8_t nAcceleratedVelocity);\n");
            SetAcceleratedVelocity_TestDemo();
        }
        break;
    case '1':
        {
            /* set VLC auto-drive state */
            printf("call SetDriveAutoState(uint8_t nDriveAutoState);\n");
            SetDriveAutoState_TestDemo();
        }
        break;
    case '2':
        {
            /* set alive counter of VLC */
            printf("call SendAliveCount(uint8_t nCount);\n");
            SendAliveCount_TestDemo();
        }
        break;
    case '3':
        {
            /* set emergency lamp switch */
            printf("call SetLampEmergencyWarningSwitch(uint8_t nSwitch);\n");
            SetLampEmergencyWarningSwitch_TestDemo();
        }
        break;
    case '4':
        {
            /* set outside lamp switch */
            printf("call SetLampOutSideSwitch(uint8_t nArea, uint8_t nSwitch);\n");
            SetLampOutSideSwitch_TestDemo();
        }
        break;
    case '5':
        {
            /* set vehicle horn switch */
            printf("call SetHornSwitch(uint8_t nSwitch);\n");
            SetHornSwitch_TestDemo();
        }
        break;
    case '6':
        {
            /* set multimedia display switch */
            printf("call SetMultimediaDisplaySwitch(uint8_t nSwitch);\n");
            SetMultimediaDisplaySwitch_TestDemo();
        }
        break;
	case '7':
		{
            /* set Lateral control state */
            printf("call SetAutoDriveLateralState( uint8_t nAutodDriveLateralState );\n");
            SetAutoDriveLateralState_TestDemo();			
		}
		break;
    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void BydAuto_DeveloperAPI_SetTest( void )
  说明:           开发者平台API的Set操作测试
  调用:
  被调用:
  输入参数:       void		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口对整车进行控制   
*****************************************************************************/
void BydAuto_DeveloperAPI_SetTest( void )
{
    DeveloperAPI_SetTestItemDisplay();

    while (gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            break;
        }

        if (0x2 == gCharInput[0] && 
            ('s' == gCharInput[1] || 'S' == gCharInput[1])) {
            DeveloperAPI_TestSetFunc1_9(gCharInput[2]);
        }

        if (0x3 == gCharInput[0] &&
           ('s' == gCharInput[1] || 'S' == gCharInput[1])) {
            // S10 ~ S17
            if ('1' != gCharInput[2]) {
                continue;
            }
            DeveloperAPI_TestSetFunc10_17(gCharInput[3]);
        }
    }
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("API Set-func Testing Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("\n");
}

