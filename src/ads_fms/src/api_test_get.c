/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test_get.c
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
  函数名称:       void DeveloperAPI_TestGetFunc1_9(int8_t Index)
  说明:           开发者平台API的Get接口1～9
  调用:
  被调用:
  输入参数:       Index
                 测试编号		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口获取整车状态值   
*****************************************************************************/
void DeveloperAPI_TestGetFunc1_9(int8_t Index)
{
    switch (Index) {
    case '1':
        {
            /* get vehicle velocity information */
            printf("call GetVelocityInfo(uint16_t *pnVelocity, uint64_t *pnTimeStamp);\n");
            GetVelocityInfo_TestDemo();
        }
        break;
    case '2':
        {
            /* get wheel speed information */
            printf("call GetWheelSpeedInfo(uint16_t *pnFL, uint16_t *pnFR, uint16_t *pnRL, uint16_t *pnRR, uint64_t *pnTimeStamp);\n");
            GetWheelSpeedInfo_TestDemo();
        }
        break;
    case '3':
        {
            /* get lamp information, include high-beam and low-beam and small-lamp */
            printf("call GetLampInfo(uint8_t *pnSmall, uint8_t *pnNear, uint8_t *pnFar, uint64_t *pnTimeStamp);\n");
            GetLampInfo_TestDemo();
        }
        break;
    case '4':
        {
            /* get turn lamp information */
            printf("call GetLampTurnInfo(uint8_t *pnLeft, uint8_t *pnRight, uint64_t *pnTimeStamp);\n");
            GetLampTurnInfo_TestDemo();
        }
        break;
    case '5':
        {
            /* get fog lamp information */
            printf("call GetLampFogInfo(uint8_t *pnFront, uint8_t *pnRear, uint64_t *pnTimeStamp);\n");
            GetLampFogInfo_TestDemo();
        }
        break;
    case '6':
        {
            /* get rain wiper information */
            printf("call GetRainWiperInfo(uint8_t *pnRainWiper, uint64_t *pnTimeStamp);\n");
            GetRainWiperInfo_TestDemo();
        }
        break;
    case '7':
        {
            /* get mfs module drive-mode feedback information */
            printf("call GetSwitchDriveModeFeedbackInfo(uint8_t *pnSwitchDriveModeFeedback, uint64_t *pnTimeStamp);\n");
            GetSwitchDriveModeFeedbackInfo_TestDemo();
        }
        break;
    case '8':
        {
            /* get lamp auto-mode information */
            printf("call GetLampAutoInfo(uint8_t *pnLampAuto, uint64_t *pnTimeStamp);\n");
            GetLampAutoInfo_TestDemo();
        }
        break;
    case '9':
        {
            /* get vehicle door information */
            printf("call GetDoorInfo(uint8_t *pnFL, uint8_t *pnFR, uint8_t *pnRL, uint8_t *pnRR, uint8_t *pnTrunk, uint64_t *pnTimeStamp);\n");
            GetDoorInfo_TestDemo();
        }
        break;
    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void DeveloperAPI_TestGetFunc10_19(int8_t Index)
  说明:           开发者平台API的Get接口10～19
  调用:
  被调用:
  输入参数:       Index
                 测试编号		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口获取整车状态值   
*****************************************************************************/
void DeveloperAPI_TestGetFunc10_19(int8_t Index)
{
    switch (Index) {
    case '0':
        {
            /* get wheel drive direction information */
            printf("call GetWheelDriveDirectionInfo(uint8_t *pnFL, uint8_t *pnFR, uint8_t *pnRL, uint8_t *pnRR, uint64_t *pnTimeStamp);\n");
            GetWheelDriveDirectionInfo_TestDemo();
        }
        break;
    case '1':
        {
            /* get ESP drive mode feedback information */
            printf("call GetESPDriveModeFeedbackInfo(uint8_t *pnESPDriveModeFeedback, uint64_t *pnTimeStamp);\n");
            GetESPDriveModeFeedbackInfo_TestDemo();
        }
        break;
    case '2':
        {
            /* get ESP Auto Drive Permission information */
            printf("call GetESPAutoDrivePermissionInfo(uint8_t *pnActive, uint64_t *pnTimeStamp);\n");
            GetESPAutoDrivePermissionInfo_TestDemo();
        }
        break;
    case '3':
        {
            /* get ESP Alive information */
            printf("call GetESPAliveInfo(uint8_t *pnAlive, uint64_t *pnTimeStamp);\n");
            GetESPAliveInfo_TestDemo();
        }
        break;
    case '4':
        {
            /* get Yaw-rate information */
            printf("call GetYawRateInfo(uint16_t *pnYawRate, uint16_t *pnYawRateOffset, uint64_t *pnTimeStamp);\n");
            GetYawRateInfo_TestDemo();
        }
        break;
    case '5':
        {
            /* get X-axis Acceleration information */
            printf("call GetAccelerationXInfo(uint16_t *pnAX, uint16_t *pnAXOffset, uint64_t *pnTimeStamp);\n");
            GetAccelerationXInfo_TestDemo();
        }
        break;
    case '6':
        {
            /* get Y-axis Acceleration information */
            printf("call GetAccelerationYInfo(uint16_t *pnAY, uint16_t *pnAYOffset, uint64_t *pnTimeStamp);\n");
            GetAccelerationYInfo_TestDemo();
        }
        break;
    case '7':
        {
            /* get wheel pulse counter information */
            printf("call GetWheelPulseCounterInfo( uint16_t *pnFL, uint16_t *pnFR, uint16_t *pnRL, uint16_t *pnRR, uint64_t *pnTimeStamp);\n");
            GetWheelPulseCounterInfo_TestDemo();
        }
        break;
    case '8':
        {
            /* get Acceleration Deepness information */
            printf("call GetAccelerateDeepnessInfo( uint8_t *pnAccelerateDeepness, uint64_t *pnTimeStamp);\n");
            GetAccelerateDeepnessInfo_TestDemo();
        }
        break;
    case '9':
        {
            /* get Brack Deepness information */
            printf("call GetBrakeDeepnessInfo(uint8_t *pnBrakeDeepness, uint64_t *pnTimeStamp);\n");
            GetBrakeDeepnessInfo_TestDemo();
        }
        break;
    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void DeveloperAPI_TestGetFunc20_29(int8_t Index)
  说明:           开发者平台API的Get接口20～29
  调用:
  被调用:
  输入参数:       Index
                 测试编号		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口获取整车状态值   
*****************************************************************************/
void DeveloperAPI_TestGetFunc20_29(int8_t Index)
{
    switch (Index) {
    case '0':
        {
            /* get Brake Pedal status information */
            printf("call GetBrakePedalSignalInfo(uint8_t *pnBrakePedalSignal, uint64_t *pnTimeStamp);\n");
            GetBrakePedalSignalInfo_TestDemo();
        }
        break;
    case '1':
        {
            /* get car Gear information */
            printf("call GetCarGearInfo(uint8_t *pnCarGear, uint64_t *pnTimeStamp);\n");
            GetCarGearInfo_TestDemo();
        }
        break;
    case '2':
        {
            /* get Gear drive mode feedback information */
            printf("call GetGearDriveModeFeedbackInfo(uint8_t* pnGearDriveModeFeedback, uint64_t* pnTimeStamp);\n");
            GetGearDriveModeFeedbackInfo_TestDemo();
        }
        break;
    case '3':
        {
            /* get EPB status information */
            printf("call GetEleBrakeInfo(uint8_t *pnEleBrake, uint64_t *pnTimeStamp);\n");
            GetEleBrakeInfo_TestDemo();
        }
        break;
    case '4':
        {
            /* get EPB drive mode feedback information */
            printf("call GetEleBrakeDriveModeFeedbackInfo(uint8_t* pnEleBrakeDriveModeFeedback, uint64_t* pnTimeStamp);\n");
            GetEleBrakeDriveModeFeedbackInfo_TestDemo();
        }
        break;
    case '5':
        {
            /* get steering wheel angle information */
            printf("call GetAngularInfo(uint16_t *pnAngular, uint64_t *pnTimeStamp);\n");
            GetAngularInfo_TestDemo();
        }
        break;
    case '6':
        {
            /* get EPS drive mode feedback information */
            printf("call GetEPSDriveModeFeedbackInfo(uint8_t *pnEPSDriveModeFeedback, uint64_t *pnTimeStamp);\n");
            GetEPSDriveModeFeedbackInfo_TestDemo();
        }
        break;
    case '7':
        {
            /* get EPS auto-drive permission information */
            printf("call GetDriveAutoPermissionInfo(uint8_t *pnDriveAutoPermission, uint64_t *pnTimeStamp);\n");
            GetDriveAutoPermissionInfo_TestDemo();
        }
        break;
    case '8':
        {
            /* get steering wheel rotation speed information */
            printf("call GetRotationSpeedInfo(uint8_t *pnRotationSpeed,  uint64_t *pnTimeStamp);\n");
            GetRotationSpeedInfo_TestDemo();
        }
        break;
    case '9':
        {
            /* get Motor controller torque request information */
            printf("call GetMotorTorqueInfo(uint16_t *pnMotorTorque, uint64_t *pnTimeStamp);\n");
            GetMotorTorqueInfo_TestDemo();
        }
        break;
    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void DeveloperAPI_TestGetFunc30_32(int8_t Index)
  说明:           开发者平台API的Get接口30～32
  调用:
  被调用:
  输入参数:       Index
                 测试编号		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口获取整车状态值   
*****************************************************************************/
void DeveloperAPI_TestGetFunc30_32(int8_t Index)
{
    switch (Index) {
    case '0':
        {
            /* get Motor controller drive mode feedback information */
            printf("call GetMCDriveModeFeedbackInfo(uint8_t *pnMCDriveModeFeedback, uint64_t *pnTimeStamp);\n");
            GetMCDriveModeFeedbackInfo_TestDemo();
        }
        break;
    case '1':
        {
            /* get auto-drive key press information */
            printf("call GetAutoDriveKeyInfo(uint8_t *pnAutoDriveKey, uint64_t *pnTimeStamp);\n");
            GetAutoDriveKeyInfo_TestDemo();
        }
        break;
    case '2':
        {
            /* get Emergency quit key press information */
            printf("call GetEmergencyExitKeyInfo(uint8_t *pnEmergencyExitKey, uint64_t *pnTimeStamp);\n");
            GetEmergencyExitKeyInfo_TestDemo();
        }
        break;
    default:
        break;
    }
}

/*****************************************************************************
  函数名称:       void BydAuto_DeveloperAPI_GetTest( void )
  说明:           开发者平台API的Get操作测试
  调用:
  被调用:
  输入参数:       void		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过API接口获取整车状态值   
*****************************************************************************/
void BydAuto_DeveloperAPI_GetTest( void )
{
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'q' or 'Q' to Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Otherwise, enter:\n");
    printf("g1  or G1:  GetVelocityInfo(uint16_t *pnVelocity, uint64_t *pnTimeStamp)\n");
    printf("g2  or G2:  GetWheelSpeedInfo(uint16_t *pnFL, uint16_t *pnFR, uint16_t *pnRL, uint16_t *pnRR, uint64_t *pnTimeStamp)\n");
    printf("g3  or G3:  GetLampInfo(uint8_t *pnSmall, uint8_t *pnNear, uint8_t *pnFar, uint64_t *pnTimeStamp)\n");
    printf("g4  or G4:  GetLampTurnInfo(uint8_t *pnLeft, uint8_t *pnRight, uint64_t *pnTimeStamp)\n");
    printf("g5  or G5:  GetLampFogInfo(uint8_t *pnFront, uint8_t *pnRear, uint64_t *pnTimeStamp)\n");
    printf("g6  or G6:  GetRainWiperInfo(uint8_t *pnRainWiper, uint64_t *pnTimeStamp)\n");
    printf("g7  or G7:  GetSwitchDriveModeFeedbackInfo(uint8_t *pnSwitchDriveModeFeedback, uint64_t *pnTimeStamp)\n");
    printf("g8  or G8:  GetLampAutoInfo(uint8_t *pnLampAuto, uint64_t *pnTimeStamp)\n");
    printf("g9  or G9:  GetDoorInfo(uint8_t *pnFL, uint8_t *pnFR, uint8_t *pnRL, uint8_t *pnRR, uint8_t *pnTrunk, uint64_t *pnTimeStamp)\n");
    printf("g10 or G10: GetWheelDriveDirectionInfo(uint8_t *pnFL, uint8_t *pnFR, uint8_t *pnRL, uint8_t *pnRR, uint64_t *pnTimeStamp)\n");
    printf("g11 or G11: GetESPDriveModeFeedbackInfo(uint8_t *pnESPDriveModeFeedback, uint64_t *pnTimeStamp)\n");
    printf("g12 or G12: GetESPActiveInfo(uint8_t *pnActive, uint64_t *pnTimeStamp)\n");
    printf("g13 or G13: GetESPAliveInfo(uint8_t *pnAlive, uint64_t *pnTimeStamp)\n");
    printf("g14 or G14: GetYawRateInfo(uint16_t *pnYawRate, uint16_t *pnYawRateOffset, uint64_t *pnTimeStamp)\n");
    printf("g15 or G15: GetAccelerationXInfo(uint16_t *pnAX, uint16_t *pnAXOffset, uint64_t *pnTimeStamp)\n");
    printf("g16 or G16: GetAccelerationYInfo(uint16_t *pnAY, uint16_t *pnAYOffset, uint64_t *pnTimeStamp)\n");
    printf("g17 or G17: GetWheelPulseCounterInfo( uint16_t *pnFL, uint16_t *pnFR, uint16_t *pnRL, uint16_t *pnRR, uint64_t *pnTimeStamp)\n");
    printf("g18 or G18: GetAccelerateDeepnessInfo( uint8_t *pnAccelerateDeepness, uint64_t *pnTimeStamp)\n");
    printf("g19 or G19: GetBrakeDeepnessInfo(uint8_t *pnBrakeDeepness, uint64_t *pnTimeStamp)\n");
    printf("g20 or G20: GetBrakePedalSignalInfo(uint8_t *pnBrakePedalSignal, uint64_t *pnTimeStamp)\n");
    printf("g21 or G21: GetCarGearInfo(uint8_t *pnCarGear, uint64_t *pnTimeStamp)\n");
    printf("g22 or G22: GetGearDriveModeFeedbackInfo(uint8_t* pnGearDriveModeFeedback, uint64_t* pnTimeStamp)\n");
    printf("g23 or G23: GetEleBrakeInfo(uint8_t *pnEleBrake, uint64_t *pnTimeStamp)\n");
    printf("g24 or G24: GetEleBrakeDriveModeFeedbackInfo(uint8_t* pnEleBrakeDriveModeFeedback, uint64_t* pnTimeStamp)\n");
    printf("g25 or G25: GetAngularInfo(uint16_t *pnAngular, uint64_t *pnTimeStamp)\n");
    printf("g26 or G26: GetEPSDriveModeFeedbackInfo(uint8_t *pnEPSDriveModeFeedback, uint64_t *pnTimeStamp)\n");
    printf("g27 or G27: GetDriveAutoPermissionInfo(uint8_t *pnDriveAutoPermission, uint64_t *pnTimeStamp)\n");
    printf("g28 or G28: GetRotationSpeedInfo(uint8_t *pnRotationSpeed,  uint64_t *pnTimeStamp)\n");
    printf("g29 or G29: GetMotorTorqueInfo(uint16_t *pnMotorTorque, uint64_t *pnTimeStamp)\n");
    printf("g30 or G30: GetMCDriveModeFeedbackInfo(uint8_t *pnMCDriveModeFeedback, uint64_t *pnTimeStamp)\n");
    printf("g31 or G31: GetAutoDriveKeyInfo(uint8_t *pnAutoDriveKey, uint64_t *pnTimeStamp)\n");
    printf("g32 or G32: GetEmergencyExitKeyInfo(uint8_t *pnEmergencyExitKey, uint64_t *pnTimeStamp)\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");

    while (gTestFlag) {
        sem_wait(&sem_event);

        if (0x1 == gCharInput[0] && 
            ('q' == gCharInput[1] || 'Q' == gCharInput[1])) {
            break;
        }

        if (0x2 == gCharInput[0] && 
            ('g' == gCharInput[1] || 'G' == gCharInput[1])) {
            DeveloperAPI_TestGetFunc1_9(gCharInput[2]);
        }

        if (0x3 == gCharInput[0] &&
            ('g' == gCharInput[1] || 'G' == gCharInput[1])) {
            switch (gCharInput[2]) {
            case '1':
                {
                    DeveloperAPI_TestGetFunc10_19(gCharInput[3]);
                }
                break;
            case '2':
                {
                    DeveloperAPI_TestGetFunc20_29(gCharInput[3]);
                }
                break;
            case '3':
                {
                    DeveloperAPI_TestGetFunc30_32(gCharInput[3]);
                }
                break;
            default:
                break;
            }
        }

    }
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("API Get-func Testing Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("\n");
}

