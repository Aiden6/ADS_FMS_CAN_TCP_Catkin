/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test_set_demo.c
  摘 要:     API使用例程

  其 他:
  函数列表:

  当前版本: 1.0
  作 者:
  修改内容:
  完成日期: 2018年06月27日
*****************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include "../include/byd/api_test.h"


/** Global variables **/
extern int32_t  gTestFlag; 
extern int8_t   gCharInput[STRING_LEN];
extern sem_t    sem_event;
extern uint8_t  driving_mode;



/*****************************************************************************
  函数名称:       int32_t Calc_DecimalData( uint8_t *pString, uint8_t cnt, double *pNumber )
  说明:           计算10进制的数据
  调用:
  被调用:
  输入参数:       pString       cnt       pNumber
                  字符串     字符个数     数字
  输出参数:       
  返回值:         状态
  被访问变量:
  被修改变量:
  其他:           通过键盘输入字符串   
*****************************************************************************/
static int32_t Calc_DecimalData( uint8_t *pString, uint8_t cnt, double *pNumber )
{
    int32_t loop = 0, dot = 0, dcnt = 0, idx = 0;
    double angleInt = 0.0, angleFloat = 0.0;

    if (0x1 == cnt && '.' == pString[0]) {
        return -1;
    }

    for (loop = 0; loop < cnt; loop++) {
        if ((pString[loop] < '0' || pString[loop] > '9') &&
            (pString[loop] != '.')) {
                return -1;
        }
        if (pString[loop] == '.') {
            dot = 1;
            dcnt++;
            if (dcnt > 1) {
                return -1;
            }
            continue;
        }

        if (0 == dot) {
            angleInt = angleInt * 10 + (pString[loop] - '0');
        }
        else {
            idx++;
            angleFloat += (pString[loop] - '0')*pow(0.1, idx);
        }
    }
    *pNumber = angleInt + angleFloat;

    return 0;
}

/*****************************************************************************
  函数名称:       int32_t Xsfer_String2number( uint8_t *pString, double *pNumber )
  说明:           将字符串转换为数字
  调用:
  被调用:
  输入参数:       pString       pNumber
                  字符串        数字
  输出参数:       
  返回值:         状态
  被访问变量:
  被修改变量:
  其他:           通过键盘输入字符串   
*****************************************************************************/
static int32_t Xsfer_String2number( uint8_t *pString, double *pNumber )
{
    double tmpVal = 0.0;

    switch (pString[1]) {
    case '+': {
            if (pString[0] < 2) {
                return -1;
            }
            else {
                return Calc_DecimalData(&pString[2], pString[0]-1, pNumber);
            }
        }
        break;
    case '-': {
            if (pString[0] < 2) {
                return -1;
            }
            else {
                if (0 == Calc_DecimalData(&pString[2], pString[0]-1, &tmpVal)) {
                    *pNumber = 0 - tmpVal;
                }
                else {
                    return -1;
                }
            }
        }
        break;
    default: {
            if (0 != Calc_DecimalData(&pString[1], pString[0], pNumber)){
                return -1;
            }
        }
        break;
    }

    return 0;
}

/*****************************************************************************
  函数名称:       void SendAutoDriveModeRequest_TestDemo( void )
  说明:           设置自动驾驶模式请求
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SendAutoDriveModeRequest接口   
*****************************************************************************/
void SendAutoDriveModeRequest_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Invalid or Hold current drive mode.\n");
    printf("Number 1: Auto-drive mode request.\n");
    printf("Number 2: Manual-drive mode request.\n");

    while(gTestFlag) {      
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SendAutoDriveModeRequest() Testing Quit.\n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING)) {
                        printf("BYD_AUTO_DRIVE_MODE_HOLDING: Hold current drive mode.\n");
                    }
                    else {
                        printf("call SendAutoDriveModeRequest() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_OPEN)) {
                        printf("BYD_AUTO_DRIVE_MODE_OPEN: Auto-drive mode request.\n");
                    }
                    else {
                        printf("call SendAutoDriveModeRequest() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_CLOSE)) {
                        printf("BYD_AUTO_DRIVE_MODE_CLOSE: Manual-drive mode request.\n");
                    }
                    else {
                        printf("call SendAutoDriveModeRequest() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }  
}

/*****************************************************************************
  函数名称:       void SetLampAutoSwitch_TestDemo( void )
  说明:           设置灯光AUTO档请求
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetLampAutoSwitch接口   
*****************************************************************************/
void SetLampAutoSwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Invalid request information.\n");
    printf("Number 1: Lamp Auto ON.\n");
    printf("Number 2: Lamp Auto OFF.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetLampAutoSwitch() Testing Quit.\n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetLampAutoSwitch(BYD_AUTO_LAMP_AUTO_INVALID)) {
                        printf("BYD_AUTO_LAMP_AUTO_INVALID: Invalid request information.\n");
                    }
                    else {
                        printf("call SetLampAutoSwitch() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetLampAutoSwitch(BYD_AUTO_LAMP_AUTO_ON)) {
                        printf("BYD_AUTO_LAMP_AUTO_ON: Lamp Auto ON.\n");
                    }
                    else {
                        printf("call SetLampAutoSwitch() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetLampAutoSwitch(BYD_AUTO_LAMP_AUTO_OFF)) {
                        printf("BYD_AUTO_LAMP_AUTO_OFF: Lamp Auto OFF.\n");
                    }
                    else {
                        printf("call SetLampAutoSwitch() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }  
}

/*****************************************************************************
  函数名称:       void SetLampTurnSwitch_TestDemo( void )
  说明:           设置转向灯信号
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetLampTurnSwitch接口   
*****************************************************************************/
void SetLampTurnSwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Lamp Turn-left OFF, Lamp Turn-right OFF.\n");
    printf("Number 1: Lamp Turn-left ON, Lamp Turn-right OFF.\n");
    printf("Number 2: Lamp Turn-left OFF, Lamp Turn-right ON.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetLampTurnSwitch() Testing Quit.\n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetLampTurnSwitch(BYD_AUTO_LAMP_OFF, BYD_AUTO_LAMP_OFF)) {
                        printf("Lamp Turn-left OFF, Lamp Turn-right OFF.\n");
                    }
                    else {
                        printf("call SetLampTurnSwitch() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetLampTurnSwitch(BYD_AUTO_LAMP_ON, BYD_AUTO_LAMP_OFF)) {
                        printf("Lamp Turn-left ON, Lamp Turn-right OFF.\n");
                    }
                    else {
                        printf("call SetLampTurnSwitch() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetLampTurnSwitch(BYD_AUTO_LAMP_OFF, BYD_AUTO_LAMP_ON)) {
                        printf("Lamp Turn-left ON, Lamp Turn-right ON.\n");
                    }
                    else {
                        printf("call SetLampTurnSwitch() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetLampBrakeSwitch_TestDemo( void )
  说明:           设置刹车灯信号
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetLampBrakeSwitch接口   
*****************************************************************************/
void SetLampBrakeSwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Brake-Lamp OFF.\n");
    printf("Number 1: Brake-Lamp ON.\n");
    
    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetLampBrakeSwitch() Testing Quit.\n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetLampBrakeSwitch(BYD_AUTO_LAMP_OFF)) {
                        printf("Brake-Lamp OFF: BYD_AUTO_LAMP_OFF.\n");
                    }
                    else {
                        printf("call SetLampBrakeSwitch() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetLampBrakeSwitch(BYD_AUTO_LAMP_ON)) {
                        printf("Brake-Lamp ON: BYD_AUTO_LAMP_ON.\n");
                    }
                    else {
                        printf("call SetLampBrakeSwitch() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetAngular_TestDemo( void )
  说明:           设置目标方向盘转角信号
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetAngular接口   
*****************************************************************************/
void SetAngular_TestDemo( void )
{
    double  inputAngle = 0.0;
    uint16_t nAngular = 0;
    int32_t modValue = 0;

    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Valid angle: [-500.0, +500.0]°.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetAngular() Testing Quit.\n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0 != Xsfer_String2number((uint8_t *)gCharInput, &inputAngle)) {
            printf("this input value is not supported.\n");
        }
        else {
            modValue = (int32_t)(inputAngle * 10) % 10;
            inputAngle = (int32_t)inputAngle + modValue * 0.1;

            printf("angle: %.2f.\n", inputAngle);
            nAngular = (780 + inputAngle) * 10;

            if (ERR_BYD_AUTO_OK == SetAngular(nAngular)) {
                printf("nAngular: 0x%x.\n", nAngular);
            }
            else {
                printf("call SetAngular() Error.\n");
            }
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetEpbState_TestDemo( void )
  说明:           设置目标EPB状态
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetEpbState接口   
*****************************************************************************/
void SetEpbState_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Epb Invalid request signal.\n");
    printf("Number 1: Epb Applying request.\n");
    printf("Number 2: Epb Released request.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetEpbState() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetEpbState(BYD_AUTO_EPB_SET_INVALID)) {
                        printf("BYD_AUTO_EPB_SET_INVALID: Epb Invalid request.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetEpbState(BYD_AUTO_EPB_SET_APPLYING)) {
                        printf("BYD_AUTO_EPB_SET_APPLYING: Epb Applying request.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetEpbState(BYD_AUTO_EPB_SET_RELEASED)) {
                        printf("BYD_AUTO_EPB_SET_RELEASED: Epb Released request.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetRainWiper_TestDemo( void )
  说明:           设置目标雨刮档位
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetRainWiper接口   
*****************************************************************************/
void SetRainWiper_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 1: Rain-wiper OFF request.\n");
    printf("Number 2: Rain-wiper short press.\n");
    printf("Number 3: Rain-wiper long press.\n");
    printf("Number 4: Rain-wiper interval-1.\n");
    printf("Number 5: Rain-wiper interval-2.\n");
    printf("Number 6: Rain-wiper interval-3.\n");
    printf("Number 7: Rain-wiper interval-4.\n");
    printf("Number 8: Rain-wiper slow.\n");
    printf("Number 9: Rain-wiper quick.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetRainWiper() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_OFF)) {
                        printf("BYD_AUTO_RAIN_WIPER_OFF: Rain-wiper OFF.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_SHORT_PRESS)) {
                        printf("BYD_AUTO_RAIN_WIPER_SHORT_PRESS: Rain-wiper short press.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '3': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_LONG_PRESS)) {
                        printf("BYD_AUTO_RAIN_WIPER_LONG_PRESS: Rain-wiper long press.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '4': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_INTERVAL_1)) {
                        printf("BYD_AUTO_RAIN_WIPER_INTERVAL_1: Rain-wiper interval-1.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '5': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_INTERVAL_2)) {
                        printf("BYD_AUTO_RAIN_WIPER_INTERVAL_2: Rain-wiper interval-2.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '6': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_INTERVAL_3)) {
                        printf("BYD_AUTO_RAIN_WIPER_INTERVAL_3: Rain-wiper interval-3.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '7': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_INTERVAL_4)) {
                        printf("BYD_AUTO_RAIN_WIPER_INTERVAL_4: Rain-wiper interval-4.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '8': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_SLOW)) {
                        printf("BYD_AUTO_RAIN_WIPER_SLOW: Rain-wiper slow.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case '9': {
                    if (ERR_BYD_AUTO_OK == SetRainWiper(BYD_AUTO_RAIN_WIPER_QUICK)) {
                        printf("BYD_AUTO_RAIN_WIPER_QUICK: Rain-wiper quick.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetCarGear_TestDemo( void )
  说明:           设置目标档位
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetCarGear接口   
*****************************************************************************/
void SetCarGear_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Invalid Gear signal.\n");
    printf("Number 1: Gear-P.\n");
    printf("Number 2: Gear-R.\n");
    printf("Number 3: Gear-N.\n");
    printf("Number 4: Gear-D.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetCarGear() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_INVALID)) {
                        printf("BYD_AUTO_CAR_GEAR_INVALID: Invalid Gear signal.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_P)) {
                        printf("BYD_AUTO_CAR_GEAR_P: Gear-P.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_R)) {
                        printf("BYD_AUTO_CAR_GEAR_R: Gear-R.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case '3': {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_N)) {
                        printf("BYD_AUTO_CAR_GEAR_N: Gear-N.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case '4': {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_D)) {
                        printf("BYD_AUTO_CAR_GEAR_D: Gear-D.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetDriveModeFeedback_TestDemo( void )
  说明:           设置驾驶模式反馈信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetDriveModeFeedback接口   
*****************************************************************************/
void SetDriveModeFeedback_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Auto-drive abnormal-1.\n");
    printf("Number 1: Auto-drive mode feedback.\n");
    printf("Number 2: Manual-drive mode feedback.\n");
    printf("Number 3: Auto-drive abnormal-2.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetDriveModeFeedback() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1: Auto-drive abnormal-1.\n");
                    }
                    else {
                        printf("call SetDriveModeFeedback() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO: Auto-drive mode.\n");
                    }
                    else {
                        printf("call SetDriveModeFeedback() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_NORMAL)) {
                        printf("BYD_AUTO_DRIVE_MODE_NORMAL: Manual-drive mode.\n");
                    }
                    else {
                        printf("call SetDriveModeFeedback() Error.\n");
                    }
                }
                break;
            case '3': {
                    if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2: Auto-drive abnormal-2.\n");
                    }
                    else {
                        printf("call SetDriveModeFeedback() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetAcceleratedVelocity_TestDemo( void )
  说明:           设置目标加速度信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetAcceleratedVelocity接口   
*****************************************************************************/
void SetAcceleratedVelocity_TestDemo( void )
{
    double inputAccVal = 0.0;
    uint8_t nAcceleratedVelocity = 0;
    int32_t modValue = 0;

    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Valid acclerated-Velocity: [-5.00, +5.00]m/s^2.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetAcceleratedVelocity() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0 != Xsfer_String2number((uint8_t *)gCharInput, &inputAccVal)) {
            printf("this input value is not supported.\n");
        }
        else {
            modValue = (int32_t)(inputAccVal * 10) % 10;
            inputAccVal = (int32_t)inputAccVal + modValue * 0.1;
            printf("acclerated-velocity: %.2f.\n", inputAccVal);

            // --------------------------------------
            if (inputAccVal < -5 || inputAccVal > 5) {
                printf("Error: out of range.\n");
                continue;
            }
            nAcceleratedVelocity = (inputAccVal + 5) * 20;

            if (ERR_BYD_AUTO_OK == SetAcceleratedVelocity(nAcceleratedVelocity)) {
                printf("nAcceleratedVelocity: 0x%x.\n", nAcceleratedVelocity);
            }
            else {
                printf("call SetAcceleratedVelocity() Error.\n");
            }
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetAcceleratedVelocity_TestDemo( void )
  说明:           设置目标加速度信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetAcceleratedVelocity接口   
*****************************************************************************/
void SetDriveAutoState_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: VLC control OFF.\n");
    printf("Number 3: VLC control ON.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetDriveAutoState() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE: Manual-drive mode request.\n");
                    }
                    else {
                        printf("call SetDriveAutoState() Error.\n");
                    }
                }
                break;
            case '3': {
                    if (ERR_BYD_AUTO_OK == SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN: Auto-drive mode request.\n");
                    }
                    else {
                        printf("call SetDriveAutoState() Error.\n");
                    }
                }
                break;  
            default:
                printf("this input value is not supported.\n");
                break; 
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SendAliveCount_TestDemo( void )
  说明:           设置滚动计数器信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SendAliveCount接口   
*****************************************************************************/
void SendAliveCount_TestDemo( void )
{
    double inputCnt = 0.0;

    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Valid count: [0, 15].\n");

    while(gTestFlag) {
        sem_wait(&sem_event);
        
        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SendAliveCount() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0 != Xsfer_String2number((uint8_t *)gCharInput, &inputCnt)) {
            printf("this input value is not supported.\n");
        }
        else {
            printf("inputCnt: %d.\n", (int32_t)inputCnt);

            if (inputCnt < 0 || inputCnt > 15) {
                printf("Error: out of range.\n");
                continue;
            }

            if (ERR_BYD_AUTO_OK == SendAliveCount(inputCnt)) {
                // -----------
            }
            else {
                printf("call SendAliveCount() Error.\n");
            }
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetLampEmergencyWarningSwitch_TestDemo( void )
  说明:           设置紧急告警灯信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetLampEmergencyWarningSwitch接口   
*****************************************************************************/
void SetLampEmergencyWarningSwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Emergency Alarm Lamp OFF.\n");
    printf("Number 1: Emergency Alarm Lamp ON.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
            printf("SetLampEmergencyWarningSwitch() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetLampEmergencyWarningSwitch(BYD_AUTO_LAMP_OFF)) {
                        printf("BYD_AUTO_LAMP_OFF: Emergency Alarm Lamp OFF.\n");
                    }
                    else {
                        printf("call SetLampEmergencyWarningSwitch() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetLampEmergencyWarningSwitch(BYD_AUTO_LAMP_ON)) {
                        printf("BYD_AUTO_LAMP_ON: Emergency Alarm Lamp ON.\n");
                    }
                    else {
                        printf("call SetLampEmergencyWarningSwitch() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetLampOutSideSwitch_TestDemo( void )
  说明:           设置外围灯光信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetLampOutSideSwitch接口   
*****************************************************************************/
void SetLampOutSideSwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Please enter ',' or ' ' to break up two params.\n");
    printf("First param: \n");
    printf("            Number 0: Small Lamp.\n");
    printf("            Number 1: Low-Beam Lamp.\n");
    printf("            Number 2: High-Beam Lamp.\n");
    printf("            Number 3: Front-Fog Lamp.\n");
    printf("            Number 4: Rear-Fog Lamp.\n");
    printf("Second param: \n");
    printf("            Number 0: Lamp OFF request.\n");
    printf("            Number 1: Lamp ON request.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetLampOutSideSwitch() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x3 == gCharInput[0] &&
            (' ' == gCharInput[2] || ',' == gCharInput[2])) {           
            switch (gCharInput[1]) {
            case '0': {
                    if ('0' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_SMALL, BYD_AUTO_LAMP_OFF)) {
                            printf("Small Lamp: BYD_AUTO_LAMP_OFF.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else if ('1' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_SMALL, BYD_AUTO_LAMP_ON)) {
                            printf("Small Lamp: BYD_AUTO_LAMP_ON.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else {
                        printf("this input value is not supported.\n");
                    }
                }
                break;
            case '1': {
                    if ('0' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_LOW, BYD_AUTO_LAMP_OFF)) {
                            printf("Low-Beam Lamp: BYD_AUTO_LAMP_OFF.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else if ('1' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_LOW, BYD_AUTO_LAMP_ON)) {
                            printf("Low-Beam Lamp: BYD_AUTO_LAMP_ON.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else {
                        printf("this input value is not supported.\n");
                    }
                }
                break;
            case '2': {
                    if ('0' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_HIGH, BYD_AUTO_LAMP_OFF)) {
                            printf("High-Beam Lamp: BYD_AUTO_LAMP_OFF.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else if ('1' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_HIGH, BYD_AUTO_LAMP_ON)) {
                            printf("High-Beam Lamp: BYD_AUTO_LAMP_ON.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else {
                        printf("this input value is not supported.\n");
                    }
                }
                break;
            case '3': {
                    if ('0' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_FRONT_FOG, BYD_AUTO_LAMP_OFF)) {
                            printf("Front-Fog Lamp: BYD_AUTO_LAMP_OFF.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else if ('1' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_FRONT_FOG, BYD_AUTO_LAMP_ON)) {
                            printf("Front-Fog Lamp: BYD_AUTO_LAMP_ON.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else {
                        printf("this input value is not supported.\n");
                    }
                }
                break;
            case '4': {
                    if ('0' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_REAR_FOG, BYD_AUTO_LAMP_OFF)) {
                            printf("Rear-Fog Lamp: BYD_AUTO_LAMP_OFF.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }
                    }
                    else if ('1' == gCharInput[3]) {
                        if (ERR_BYD_AUTO_OK == SetLampOutSideSwitch(BYD_AUTO_LAMP_REAR_FOG, BYD_AUTO_LAMP_ON)) {
                            printf("Rear-Fog Lamp: BYD_AUTO_LAMP_ON.\n");
                        }
                        else {
                            printf("call SetLampOutSideSwitch() Error.\n");
                        }   
                    }
                    else {
                        printf("this input value is not supported.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;   
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetHornSwitch_TestDemo( void )
  说明:           设置喇叭信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetHornSwitch接口   
*****************************************************************************/
void SetHornSwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: Horn Switch OFF.\n");
    printf("Number 1: Horn Switch ON.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetHornSwitch() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetHornSwitch(BYD_AUTO_HORN_OFF)) {
                        printf("BYD_AUTO_HORN_OFF: Horn Switch OFF.\n");
                    }
                    else {
                        printf("call SetHornSwitch() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetHornSwitch(BYD_AUTO_HORN_ON)) {
                        printf("BYD_AUTO_HORN_ON: Horn Switch ON.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetMultimediaDisplaySwitch_TestDemo( void )
  说明:           设置多媒体显示开关信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetMultimediaDisplaySwitch接口   
*****************************************************************************/
void SetMultimediaDisplaySwitch_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 1: Multimedia display ON.\n");
    printf("Number 2: Multimedia display OFF.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetMultimediaDisplaySwitch() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetMultimediaDisplaySwitch(BYD_AUTO_MULTIMEDIA_DISPLAY_ON)) {
                        printf("BYD_AUTO_MULTIMEDIA_DISPLAY_ON: Multimedia display ON.\n");
                    }
                    else {
                        printf("call SetMultimediaDisplaySwitch() Error.\n");
                    }
                }
                break;
            case '2': {
                    if (ERR_BYD_AUTO_OK == SetMultimediaDisplaySwitch(BYD_AUTO_MULTIMEDIA_DISPLAY_OFF)) {
                        printf("BYD_AUTO_MULTIMEDIA_DISPLAY_OFF: Multimedia display OFF.\n");
                    }
                    else {
                        printf("call SetMultimediaDisplaySwitch() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}

/*****************************************************************************
  函数名称:       void SetAutoDriveLateralState_TestDemo( void )
  说明:           设置横向控制状态
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetAutoDriveLateralState接口   
*****************************************************************************/
void SetAutoDriveLateralState_TestDemo( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter ----\n");
    printf("Number 0: EPS AutoDrive OFF.\n");
    printf("Number 1: EPS AutoDrive ON.\n");

    while(gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
            printf("SetAutoDriveLateralState() Testing Quit. \n");
            printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
			if(NORMAL_MODE == driving_mode)
			{
				DeveloperAPI_SetTestItemDisplay();
			}else if(AUTO_MODE == driving_mode)
			{
				DeveloperAPI_AutoDrive_ControlItemDisplay();
			}
            break;
        }

        if (0x1 == gCharInput[0]) {
            switch (gCharInput[1]) {
            case '0': {
                    if (ERR_BYD_AUTO_OK == SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_CLOSE)) {
                        printf("BYD_AUTO_DRIVE_MODE_EPS_CLOSE: EPS AutoDrive OFF.\n");
                    }
                    else {
                        printf("call SetAutoDriveLateralState() Error.\n");
                    }
                }
                break;
            case '1': {
                    if (ERR_BYD_AUTO_OK == SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_OPEN)) {
                        printf("BYD_AUTO_DRIVE_MODE_EPS_OPEN: EPS AutoDrive ON.\n");
                    }
                    else {
                        printf("call SetAutoDriveLateralState() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        }
        else {
            printf("this input value is not supported.\n");
        }
        printf("\n");
    }
}
