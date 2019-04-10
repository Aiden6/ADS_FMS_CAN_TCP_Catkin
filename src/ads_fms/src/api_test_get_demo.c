/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test_get_demo.c
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
#include "../include/byd/api_test.h"


/** Global variables **/
extern uint8_t  driving_mode;


/*****************************************************************************
  函数名称:       void GetVelocityInfo_TestDemo( void )
  说明:           获取车速信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetVelocityInfo接口   
*****************************************************************************/
void GetVelocityInfo_TestDemo( void )
{
    uint16_t currVelocity = 0;
    uint64_t timeStamp = 0;
    int32_t realSpeed = 0.0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetVelocityInfo(&currVelocity, &timeStamp)) {       
        realSpeed = (int32_t)(currVelocity * 0.06875 + 0.5);

        printf("*pnVelocity: 0x%x, *pnTimeStamp: 0x%lx.\n", currVelocity, timeStamp);
        printf("real vehicle speed: %d.\n", realSpeed);
    }
    else {
        printf("call GetVelocityInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetWheelSpeedInfo_TestDemo( void )
  说明:           获取轮速的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetWheelSpeedInfo接口   
*****************************************************************************/
void GetWheelSpeedInfo_TestDemo( void )
{
    uint16_t frontLeftVelocity = 0,
             frontRightVelocity = 0,
             rearLeftVelocity = 0,
             rearRightVelocity = 0;
    uint64_t timeStamp = 0;

    int32_t realFLVelocity = 0.0,
            realFRVelocity = 0.0,
            rearRLVelocity = 0.0,
            rearRRVelocity = 0.0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetWheelSpeedInfo(&frontLeftVelocity, &frontRightVelocity,
                                             &rearLeftVelocity, &rearRightVelocity, &timeStamp)) {      
        realFLVelocity = (int32_t)(frontLeftVelocity * 0.06875 + 0.5);
        realFRVelocity = (int32_t)(frontRightVelocity * 0.06875 + 0.5);
        rearRLVelocity = (int32_t)(rearLeftVelocity * 0.06875 + 0.5);
        rearRRVelocity = (int32_t)(rearRightVelocity * 0.06875 + 0.5);

        printf("*pnFL: 0x%x, *pnFR: 0x%x, *pnRL: 0x%x, *pnRR: 0x%x, *pnTimeStamp: 0x%lx.\n", \
               frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity, timeStamp );
        printf("real front-real: %d, front-right: %d, rear-left: %d, rear-right: %d.\n",
               realFLVelocity, realFRVelocity, rearRLVelocity, rearRRVelocity);
    }
    else {
        printf("call GetWheelSpeedInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetLampInfo_TestDemo( void )
  说明:           获取灯光信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetLampInfo接口   
*****************************************************************************/
void GetLampInfo_TestDemo( void )
{
    uint8_t smallLamp = 0,
            lowLamp = 0, hightLamp = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetLampInfo(&smallLamp, &lowLamp, &hightLamp, &timeStamp)) {
        printf("*pnSmall: 0x%x, *pnNear: 0x%x, *pnFar: 0x%x, *pnTimeStamp: 0x%lx.\n", smallLamp, lowLamp, hightLamp, timeStamp);

        switch (smallLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("small lamp is OFF; ");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("small lamp is ON; ");
            }
            break;
        default:
            break;
        }

        switch (lowLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("low-beam lamp is OFF; ");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("low-beam lamp is ON; ");
            }
            break;
        default:
            break;
        }

        switch (hightLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("high-beam lamp is OFF.\n");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("high-beam lamp is ON.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetLampInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetLampTurnInfo_TestDemo( void )
  说明:           获取转向灯的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetLampTurnInfo接口   
*****************************************************************************/
void GetLampTurnInfo_TestDemo( void )
{
    uint8_t turnLeftLamp = 0,
            turnRightLamp = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetLampTurnInfo(&turnLeftLamp, &turnRightLamp, &timeStamp)) {
        printf("*pnLeft: 0x%x, *pnRight: 0x%x, *pnTimeStamp: 0x%lx.\n", turnLeftLamp, turnRightLamp, timeStamp);

        switch (turnLeftLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("turn-left lamp is OFF; ");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("turn-left lamp is ON; ");
            }
            break;
        default:
            break;
        }

        switch (turnRightLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("turn-right lamp is OFF.\n");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("turn-right lamp is ON.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetLampTurnInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetLampFogInfo_TestDemo( void )
  说明:           获取雾灯的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetLampFogInfo接口   
*****************************************************************************/
void GetLampFogInfo_TestDemo( void )
{
    uint8_t frontFogLamp = 0,
            rearFogLamp = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetLampFogInfo(&frontFogLamp, &rearFogLamp, &timeStamp)) {
        printf("*pnFront: 0x%x, *pnRear: 0x%x, *pnTimeStamp: 0x%lx.\n", frontFogLamp, rearFogLamp, timeStamp);

        switch (frontFogLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("front-fog lamp is OFF; ");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("front-fog lamp is ON; ");
            }
            break;
        default:
            break;
        }

        switch (rearFogLamp) {
        case BYD_AUTO_LAMP_OFF: {
                printf("rear-fog lamp is OFF.\n");
            }
            break;
        case BYD_AUTO_LAMP_ON: {
                printf("rear-fog lamp is ON.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetLampFogInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetRainWiperInfo_TestDemo( void )
  说明:           获取雨刮档位的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetRainWiperInfo接口   
*****************************************************************************/
void GetRainWiperInfo_TestDemo( void )
{
    uint8_t rainWiperStatus = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetRainWiperInfo(&rainWiperStatus, &timeStamp)) {
        printf("*pnRainWiper: 0x%x, *pnTimeStamp: 0x%lx.\n", rainWiperStatus, timeStamp);

        switch (rainWiperStatus) {
        case BYD_AUTO_RAIN_WIPER_INVALID: {
                printf("invalid signal.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_OFF: {
                printf("rain-wiper OFF.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_SHORT_PRESS: {
                printf("rain-wiper short press.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_LONG_PRESS: {
                printf("rain-wiper long press.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_INTERVAL_1: {
                printf("rain-wiper interval-1.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_INTERVAL_2: {
                printf("rain-wiper interval-2.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_INTERVAL_3: {
                printf("rain-wiper interval-3.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_INTERVAL_4: {
                printf("rain-wiper interval-4.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_SLOW: {
                printf("rain-wiper slow.\n");
            }
            break;
        case BYD_AUTO_RAIN_WIPER_QUICK: {
                printf("rain-wiper quick.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetRainWiperInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetSwitchDriveModeFeedbackInfo_TestDemo( void )
  说明:           获取组合开关驾驶模式反馈信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetSwitchDriveModeFeedbackInfo接口   
*****************************************************************************/
void GetSwitchDriveModeFeedbackInfo_TestDemo( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetSwitchDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnSwitchDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_INVALID: {
                printf("invalid signal.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_NORMAL: {
                printf("normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_AUTO: {
                printf("auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetSwitchDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetLampAutoInfo_TestDemo( void )
  说明:           获取灯光AUTO档信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetLampAutoInfo接口   
*****************************************************************************/
void GetLampAutoInfo_TestDemo( void )
{
    uint8_t lampAuto = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetLampAutoInfo(&lampAuto, &timeStamp)) {
        printf("*pnLampAuto: 0x%x, *pnTimeStamp: 0x%lx.\n", lampAuto, timeStamp);

        switch (lampAuto) {
        case BYD_AUTO_LAMP_AUTO_INVALID: {
                printf("invalid signal.\n");
            }
            break;
        case BYD_AUTO_LAMP_AUTO_ON: {
                printf("lamp-AUTO ON.\n");
            }
            break;
        case BYD_AUTO_LAMP_AUTO_OFF: {
                printf("lamp-AUTO OFF.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetLampAutoInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetDoorInfo_TestDemo( void )
  说明:           获取车门信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetDoorInfo接口   
*****************************************************************************/
void GetDoorInfo_TestDemo( void )
{
    uint8_t frontLeftDoor = 0,
            frontRightDoor = 0,
            rearLeftDoor = 0,
            rearRightDoor = 0,
            trunkLid = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetDoorInfo(&frontLeftDoor, &frontRightDoor,
                                       &rearLeftDoor, &rearRightDoor, &trunkLid, &timeStamp)) {
        printf("*pnFL: 0x%x, *pnFR: 0x%x, *pnRL: 0x%x, *pnRR: 0x%x, *pnTrunk: 0x%x, *pnTimeStamp: 0x%lx.\n",
               frontLeftDoor, frontRightDoor, rearLeftDoor, rearRightDoor, trunkLid, timeStamp);

        switch (frontLeftDoor) {
        case BYD_AUTO_DOOR_OFF: {
                printf("front-left door is closed; ");
            }
            break;
        case BYD_AUTO_DOOR_ON: {
                printf("front-left door is opened; ");
            }
            break;
        default:
            break;
        }

        switch (frontRightDoor) {
        case BYD_AUTO_DOOR_OFF: {
                printf("front-right door is closed; ");
            }
            break;
        case BYD_AUTO_DOOR_ON: {
                printf("front-right door is opened; ");
            }
            break;
        default:
            break;
        }

        switch (rearLeftDoor) {
        case BYD_AUTO_DOOR_OFF: {
                printf("rear-left door is closed; ");
            }
            break;
        case BYD_AUTO_DOOR_ON: {
                printf("rear-left door is opened; ");
            }
            break;
        default:
            break;
        }

        switch (rearRightDoor) {
        case BYD_AUTO_DOOR_OFF: {
                printf("rear-right door is closed; ");
            }
            break;
        case BYD_AUTO_DOOR_ON: {
                printf("rear-right door is opened; ");
            }
            break;
        default:
            break;
        }

        switch (trunkLid) {
        case BYD_AUTO_TRUNK_OFF: {
                printf("trunk is closed.\n");
            }
            break;
        case BYD_AUTO_TRUNK_ON: {
                printf("trunk is opened.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetDoorInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetWheelDriveDirectionInfo_TestDemo( void )
  说明:           获取车轮行驶方向信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetWheelDriveDirectionInfo接口   
*****************************************************************************/
void GetWheelDriveDirectionInfo_TestDemo( void )
{
    uint8_t frontLeftDir = 0,
            frontRightDir = 0,
            rearLeftDir = 0,
            rearRightDir = 0;
    uint64_t timeStamp = 0;
            
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetWheelDriveDirectionInfo(&frontLeftDir, &frontRightDir,
                                                      &rearLeftDir, &rearRightDir, &timeStamp)) {
        printf("*pnFL: 0x%x, *pnFR: 0x%x, *pnRL: 0x%x, *pnRR: 0x%x, *pnTimeStamp: 0x%lx.\n",
               frontLeftDir, frontRightDir, rearLeftDir, rearRightDir, timeStamp);

        switch (frontLeftDir) {
        case BYD_AUTO_DRIVE_DIRECTION_UNDEFINE: {
                printf("front-left wheel invalid signal; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_FORWARD: {
                printf("front-left wheel forward; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_BACKWARD: {
                printf("front-left wheel backward; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_STOP: {
                printf("front-left wheel stop; ");
            }
            break;
        default:
            break;
        }

        switch (frontRightDir) {
        case BYD_AUTO_DRIVE_DIRECTION_UNDEFINE: {
                printf("front-right wheel invalid signal; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_FORWARD: {
                printf("front-right wheel forward; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_BACKWARD: {
                printf("front-right wheel backward; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_STOP: {
                printf("front-right wheel stop; ");
            }
            break;
        default:
            break;
        }

        switch (rearLeftDir) {
        case BYD_AUTO_DRIVE_DIRECTION_UNDEFINE: {
                printf("rear-left wheel invalid signal; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_FORWARD: {
                printf("rear-left wheel forward; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_BACKWARD: {
                printf("rear-left wheel backward; ");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_STOP: {
                printf("rear-left wheel stop; ");
            }
            break;
        default:
            break;
        }

        switch (rearRightDir) {
        case BYD_AUTO_DRIVE_DIRECTION_UNDEFINE: {
                printf("rear-right wheel invalid signal.\n");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_FORWARD: {
                printf("rear-right wheel forward.\n");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_BACKWARD: {
                printf("rear-right wheel backward.\n");
            }
            break;
        case BYD_AUTO_DRIVE_DIRECTION_STOP: {
                printf("rear-right wheel stop.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetWheelDriveDirectionInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetESPDriveModeFeedbackInfo_TestDemo( void )
  说明:           获取ESP驾驶模式反馈信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetESPDriveModeFeedbackInfo接口   
*****************************************************************************/
void GetESPDriveModeFeedbackInfo_TestDemo( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetESPDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnESPDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_CLOSE: {
                printf("ESP normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_OPEN: {
                printf("ESP auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetESPDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetESPAutoDrivePermissionInfo_TestDemo( void )
  说明:           获取ESP是否允许进入智能驾驶模式的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetESPActiveInfo接口   
*****************************************************************************/
void GetESPAutoDrivePermissionInfo_TestDemo( void )
{
    uint8_t espPermissionInfo = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetESPAutoDrivePermissionInfo(&espPermissionInfo, &timeStamp)) {
        printf("*pnESPDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", espPermissionInfo, timeStamp);

        switch (espPermissionInfo) {
        case BYD_AUTO_DRIVE_ESP_NOT_PERMISSION: {
                printf("ESP Auto Drive Mode is not allowable.\n");
            }
            break;
        case BYD_AUTO_DRIVE_ESP_PERMISSION_OK: {
                printf("ESP Auto Drive Mode is allowable.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetESPAutoDrivePermissionInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetESPAliveInfo_TestDemo( void )
  说明:           获取ESP滚动计数信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetESPAliveInfo接口   
*****************************************************************************/
void GetESPAliveInfo_TestDemo( void )
{
    uint8_t aliveCount = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetESPAliveInfo(&aliveCount, &timeStamp)) {

        printf("*pnAlive: 0x%x, *pnTimeStamp: 0x%lx.\n", aliveCount, timeStamp);
    }
    else {
        printf("call GetESPAliveInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetYawRateInfo_TestDemo( void )
  说明:           获取横摆角速度信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetYawRateInfo接口   
*****************************************************************************/
void GetYawRateInfo_TestDemo( void )
{
    uint16_t yawRate = 0,
             yawRateOffset = 0;
    uint64_t timeStamp = 0;

    double realYawRate = 0.0,
           realYawRateOffset = 0.0;
    double wholeYawRate = 0.0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetYawRateInfo(&yawRate, &yawRateOffset, &timeStamp)) {
        realYawRate = yawRate * 0.002132603 - 2.0943;
        realYawRateOffset = yawRateOffset * 0.002132603 - 0.13;
        wholeYawRate = realYawRate - realYawRateOffset;

        printf("*pnYawRate: 0x%x, *pnYawRateOffset: 0x%x, *pnTimeStamp: 0x%lx.\n", yawRate, yawRateOffset, timeStamp);
        printf("realYawRate: %lf, realYawRateOffset: %lf, wholeYawRate: %lf.\n", realYawRate, realYawRateOffset, wholeYawRate);
    }
    else {
        printf("call GetYawRateInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetAccelerationXInfo_TestDemo( void )
  说明:           获取纵向加速度信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetAccelerationXInfo接口   
*****************************************************************************/
void GetAccelerationXInfo_TestDemo( void )
{
    uint16_t accXInfo = 0,
             accXInfoOffset = 0;
    uint64_t timeStamp = 0;

    double realAccXInfo = 0.0,
           realAccXInfoOffset = 0.0;
    double wholeAccXInfo = 0.0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAccelerationXInfo(&accXInfo, &accXInfoOffset, &timeStamp)) {
        realAccXInfo = accXInfo * 0.027126736 - 21.593;
        realAccXInfoOffset = accXInfoOffset * 0.027126736 - 21.593;
        wholeAccXInfo = realAccXInfo - realAccXInfoOffset;

        printf("*pnAX: 0x%x, *pnAXOffset: 0x%x, *pnTimeStamp: 0x%lx.\n", accXInfo, accXInfoOffset, timeStamp);
        printf("realAccXInfo: %lf, realAccXInfoOffset: %lf, wholeAccXInfo: %lf.\n", realAccXInfo, realAccXInfoOffset, wholeAccXInfo);
    }
    else {
        printf("call GetAccelerationXInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetAccelerationYInfo_TestDemo( void )
  说明:           获取横向加速度信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetAccelerationYInfo接口   
*****************************************************************************/
void GetAccelerationYInfo_TestDemo( void )
{
    uint16_t accYInfo = 0,
             accYInfoOffset = 0;
    uint64_t timeStamp = 0;

    double realAccYInfo = 0.0,
           realAccYInfoOffset = 0.0;
    double wholeAccYInfo = 0.0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAccelerationYInfo(&accYInfo, &accYInfoOffset, &timeStamp)) {
        realAccYInfo = accYInfo * 0.027126736 - 21.593;
        realAccYInfoOffset = accYInfoOffset * 0.027126736 - 21.593;
        wholeAccYInfo = realAccYInfo - realAccYInfoOffset;

        printf("*pnAY: 0x%x, *pnAYOffset: 0x%x, *pnTimeStamp: 0x%lx.\n", accYInfo, accYInfoOffset, timeStamp);
        printf("realAccYInfo: %lf, realAccYInfoOffset: %lf, wholeAccYInfo: %lf.\n", realAccYInfo, realAccYInfoOffset, wholeAccYInfo);
    }
    else {
        printf("call GetAccelerationYInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetWheelPulseCounterInfo_TestDemo( void )
  说明:           获取轮速脉冲计数信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetWheelPulseCounterInfo接口   
*****************************************************************************/
void GetWheelPulseCounterInfo_TestDemo( void )
{
    uint16_t frontLeftCnt = 0,
             frontRightCnt = 0,
             rearLeftCnt = 0,
             rearRightCnt = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetWheelPulseCounterInfo(&frontLeftCnt, &frontRightCnt,
                                                    &rearLeftCnt, &rearRightCnt, &timeStamp)) {

        printf("*pnFL: 0x%x, *pnFR: 0x%x, *pnRL: 0x%x, *pnRR: 0x%x, *pnTimeStamp: 0x%lx.\n",
               frontLeftCnt, frontRightCnt, rearLeftCnt, rearRightCnt, timeStamp);
    }
    else {
        printf("call GetWheelPulseCounterInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetAccelerateDeepnessInfo_TestDemo( void )
  说明:           获取油门深度信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetAccelerateDeepnessInfo接口   
*****************************************************************************/
void GetAccelerateDeepnessInfo_TestDemo( void )
{
    uint8_t accDeepness = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAccelerateDeepnessInfo(&accDeepness, &timeStamp)) {

        printf("*pnAccelerateDeepness: 0x%x, *pnTimeStamp: 0x%lx.\n", accDeepness, timeStamp);
        printf("Acceleration deepness: %d percent.\n", accDeepness);
    }
    else {
        printf("call GetAccelerateDeepnessInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetBrakeDeepnessInfo_TestDemo( void )
  说明:           获取制动深度信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetBrakeDeepnessInfo接口   
*****************************************************************************/
void GetBrakeDeepnessInfo_TestDemo( void )
{
    uint8_t brakeDeepness = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetBrakeDeepnessInfo(&brakeDeepness, &timeStamp)) {
        
        printf("*pnBrakeDeepness: 0x%x, *pnTimeStamp: 0x%lx.\n", brakeDeepness, timeStamp);
        printf("Brake deepness: %d percent.\n", brakeDeepness);
    }
    else {
        printf("call GetBrakeDeepnessInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetBrakePedalSignalInfo_TestDemo( void )
  说明:           获取制动踏板状态信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetBrakePedalSignalInfo接口   
*****************************************************************************/
void GetBrakePedalSignalInfo_TestDemo( void )
{
    uint8_t brakePedalStatus = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetBrakePedalSignalInfo(&brakePedalStatus, &timeStamp)) {
        printf("*pnBrakePedalSignal: 0x%x, *pnTimeStamp: 0x%lx.\n", brakePedalStatus, timeStamp);

        switch (brakePedalStatus) {
        case BYD_AUTO_BRAKE_PEDAL_SIGNAL_NO_PRESSED: {
                printf("brake pedal is not pressed.\n");
            }
            break;
        case BYD_AUTO_BRAKE_PEDAL_SIGNAL_PRESSED: {
                printf("brake pedal is pressed.\n");
            }
            break;
        case BYD_AUTO_BRAKE_PEDAL_SIGNAL_ERR: {
                printf("brake pedal signal error.\n"); 
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetBrakePedalSignalInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetCarGearInfo_TestDemo( void )
  说明:           获取档位信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetCarGearInfo接口   
*****************************************************************************/
void GetCarGearInfo_TestDemo( void )
{
    uint8_t carGear = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetCarGearInfo(&carGear, &timeStamp)) {
        printf("*pnCarGear: 0x%x, *pnTimeStamp: 0x%lx.\n", carGear, timeStamp);

        switch (carGear) {
        case BYD_AUTO_CAR_GEAR_INVALID: {
                printf("car-Gear signal invalid.\n");
            }
            break;
        case BYD_AUTO_CAR_GEAR_P: {
                printf("Gear-P.\n");
            }
            break;
        case BYD_AUTO_CAR_GEAR_R: {
                printf("Gear-R.\n");
            }
            break;
        case BYD_AUTO_CAR_GEAR_N: {
                printf("Gear-N.\n");
            }
            break;
        case BYD_AUTO_CAR_GEAR_D: {
                printf("Gear-D.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetCarGearInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetGearDriveModeFeedbackInfo_TestDemo( void )
  说明:           获取档位系统驾驶模式反馈信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetGearDriveModeFeedbackInfo接口   
*****************************************************************************/
void GetGearDriveModeFeedbackInfo_TestDemo( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetGearDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnGearDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_NORMAL: {
                printf("Gear normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_AUTO: {
                printf("Gear auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetGearDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetEleBrakeInfo_TestDemo( void )
  说明:           获取电子手刹状态信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetEleBrakeInfo接口   
*****************************************************************************/
void GetEleBrakeInfo_TestDemo( void )
{
    uint8_t epbInfo = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetEleBrakeInfo(&epbInfo, &timeStamp)) {
        printf("*pnEleBrake: 0x%x, *pnTimeStamp: 0x%lx.\n", epbInfo, timeStamp);

        switch (epbInfo) {
        case BYD_AUTO_EPB_GET_RELEASING: {
                printf("EPB is releasing.\n");
            }
            break;
        case BYD_AUTO_EPB_GET_RELEASED: {
                printf("EPB is released.\n");
            }
            break;
        case BYD_AUTO_EPB_GET_APPLYING: {
                printf("EPB is applying.\n");
            }
            break;
        case BYD_AUTO_EPB_GET_APPLIED: {
                printf("EPB is applied.\n");
            }
            break;
        case BYD_AUTO_EPB_GET_BEAKE_FAULT: {
                printf("EPB signal error.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetEleBrakeInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetEleBrakeDriveModeFeedbackInfo_TestDemo( void )
  说明:           获取电子手刹驾驶模式反馈信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetEleBrakeDriveModeFeedbackInfo接口   
*****************************************************************************/
void GetEleBrakeDriveModeFeedbackInfo_TestDemo( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetEleBrakeDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnEleBrakeDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_INVALID: {
                printf("EPB feedback signal invalid.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_NORMAL: {
                printf("EPB normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_AUTO: {
                printf("EPB auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetEleBrakeDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetAngularInfo_TestDemo( void )
  说明:           获取方向盘角度信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetAngularInfo接口   
*****************************************************************************/
void GetAngularInfo_TestDemo( void )
{
    uint16_t angleValue = 0;
    uint64_t timeStamp = 0;

    double  realAngleInfo = 0.0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAngularInfo(&angleValue, &timeStamp)) {
        printf("*pnAngular: 0x%x, *pnTimeStamp: 0x%lx.\n", angleValue, timeStamp);

        realAngleInfo = (int16_t)angleValue * 0.1;
        printf("realAngleInfo: %lf.\n", realAngleInfo);
		if(AUTO_MODE == driving_mode)
		{
			if(500 < realAngleInfo || realAngleInfo < -500)	
			{
				printf("The realAngleInfo = %lf is out the range [-500, 500]!,"
					   "the EPS may exit into nomal drive!\n",realAngleInfo);
			}
		}
    }
    else {
        printf("call GetAngularInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetEPSDriveModeFeedbackInfo_TestDemo( void )
  说明:           获取EPS驾驶模式反馈信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetEPSDriveModeFeedbackInfo接口   
*****************************************************************************/
void GetEPSDriveModeFeedbackInfo_TestDemo( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetEPSDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnEPSDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_INVALID: {
                printf("EPS feedback signal invalid.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_NORMAL: {
                printf("EPS normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_AUTO: {
                printf("EPS auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetEPSDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetDriveAutoPermissionInfo_TestDemo( void )
  说明:           获取EPS自动驾驶允许的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetDriveAutoPermissionInfo接口   
*****************************************************************************/
void GetDriveAutoPermissionInfo_TestDemo( void )
{
    uint8_t epsAutoPermission = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetDriveAutoPermissionInfo(&epsAutoPermission, &timeStamp)) {
        printf("*pnDriveAutoPermission: 0x%x, *pnTimeStamp: 0x%lx.\n", epsAutoPermission, timeStamp);

        switch (epsAutoPermission) {
        case BYD_AUTO_EPS_PERMISSION_INVALID: {
                printf("EPS auto drive permission signal invalid.\n");
            }
            break;
        case BYD_AUTO_EPS_PERMISSION_OK: {
                printf("EPS auto drive permission.\n");
            }
            break;
        case BYD_AUTO_EPS_NOT_PERMISSION: {
                printf("EPS auto drive not permission.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetDriveAutoPermissionInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetRotationSpeedInfo_TestDemo( void )
  说明:           获取方向盘旋转速度的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetRotationSpeedInfo接口   
*****************************************************************************/
void GetRotationSpeedInfo_TestDemo( void )
{
    uint8_t steerWheelRotationSpeed = 0;
    uint64_t timeStamp = 0;

    uint16_t realRotationSpeed = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetRotationSpeedInfo(&steerWheelRotationSpeed, &timeStamp)) {
        printf("*pnRotationSpeed: 0x%x, *pnTimeStamp: 0x%lx.\n", steerWheelRotationSpeed, timeStamp);

        realRotationSpeed = 4 * steerWheelRotationSpeed;
        printf("steering-wheel's real rotation speed: %d", realRotationSpeed);
    }
    else {
        printf("call GetRotationSpeedInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetMotorTorqueInfo_TestDemo( void )
  说明:           获取电机扭矩信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetMotorTorqueInfo接口   
*****************************************************************************/
void GetMotorTorqueInfo_TestDemo( void )
{
    uint16_t motorTorque = 0;
    uint64_t timeStamp = 0;
    int32_t realMotorTorque = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetMotorTorqueInfo(&motorTorque, &timeStamp)) {
        printf("*pnMotorTorque: 0x%x, *pnTimeStamp: 0x%lx.\n", motorTorque, timeStamp);

        realMotorTorque = motorTorque - 12000;
        printf("real motor-torque: %d Nm.\n", realMotorTorque);
    }
    else {
        printf("call GetMotorTorqueInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetMCDriveModeFeedbackInfo_TestDemo( void )
  说明:           获取电机控制器驾驶模式反馈信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetMCDriveModeFeedbackInfo接口   
*****************************************************************************/
void GetMCDriveModeFeedbackInfo_TestDemo( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetMCDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnMCDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_NORMAL: {
                printf("Motor controller normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_AUTO: {
                printf("Motor controller auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetMCDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetAutoDriveKeyInfo_TestDemo( void )
  说明:           获取智能驾驶主开关信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetAutoDriveKeyInfo接口   
*****************************************************************************/
void GetAutoDriveKeyInfo_TestDemo( void )
{
    uint8_t autoDriveKey = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAutoDriveKeyInfo(&autoDriveKey, &timeStamp)) {
        printf("*pnAutoDriveKey: 0x%x, *pnTimeStamp: 0x%lx.\n", autoDriveKey, timeStamp); 

        switch (autoDriveKey) {
        case BYD_AUTO_MEDIUM_KEY_INVALID: {
                printf("auto-drive key is not pressed.\n");
            }
            break;
        case BYD_AUTO_MEDIUM_KEY_DRIVE_AUTO_SWITCH: {
                printf("auto-drive key is pressed.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetAutoDriveKeyInfo() Error.\n");
    }
    printf("\n");
}

/*****************************************************************************
  函数名称:       void GetEmergencyExitKeyInfo_TestDemo( void )
  说明:           获取紧急退出按键信息的API接口测试
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用GetEmergencyExitKeyInfo接口   
*****************************************************************************/
void GetEmergencyExitKeyInfo_TestDemo( void )
{
    uint8_t emergencyExitKey = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetEmergencyExitKeyInfo(&emergencyExitKey, &timeStamp)) {
        printf("*pnEmergencyExitKey: 0x%x, *pnTimeStamp: 0x%lx.\n", emergencyExitKey, timeStamp);

        switch (emergencyExitKey) {
        case BYD_AUTO_MEDIUM_KEY_INVALID: {
                printf("emergency quit key is not pressed.\n");
            }
            break;
        case BYD_AUTO_MEDIUM_KEY_STOP_SWITCH: {
                printf("emergency quit key is pressed.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetEmergencyExitKeyInfo() Error.\n");
    }
    printf("\n");
}
