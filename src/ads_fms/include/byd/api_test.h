/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test.h
  摘 要:     API使用例程

  其 他:
  函数列表:

  当前版本: 1.0
  作 者:
  修改内容:
  完成日期: 2018年06月27日
*****************************************************************************/
#ifndef _API_TEST_H_
#define _API_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Include files **/
#include "./controlcan.h"
#include "./idriving_api.h"
#include "./std_types.h"
#include <curses.h>

/** Macros **/
#define STRING_LEN                (32)
#define BYD_AUTODRIVE_REQUEST_OK  (0)
#define BYD_AUTODRIVE_REQUEST_ERR (-1)
#define AUTODRIVE_REQUEST         (0)
#define AUTODRIVE_QUIT            (1)
#define BYD_AUTODRIVE_QUIT_FAIL   (-1)
#define BYD_AUTODRIVE_QUIT_OK     (0)

#define AUTO_MODE                 (1)
#define NORMAL_MODE				        (0)

#define msleep(ms)      usleep(ms*1000)

/** Data structures **/
typedef struct 
{
    int32_t  devType;
    int32_t  devIndex;
    int32_t  channelNum;
}
CAN_DEV_INFO;

/** Function prototypes **/
void BydAuto_DeveloperAPI_Start( void );

void BydAuto_DeveloperAPI_ReceiveTest( PVCI_CAN_OBJ pCAN_Obj, 
                                       uint32_t objCount );

int32_t BydAuto_DeveloperAPI_TransmitTest( uint32_t msgId,
                                           uint32_t Count,
                                           PVCI_CAN_OBJ pCAN_Obj );

void BydAuto_DeveloperAPI_ControlTest( int8_t *pTestRq );

void BydAuto_DeveloperAPI_GetTest( void );

void DeveloperAPI_TestSetFunc1_9(int8_t Index);

void DeveloperAPI_TestSetFunc10_17(int8_t Index);

void DeveloperAPI_TestGetFunc1_9(int8_t Index);

void DeveloperAPI_TestGetFunc10_19(int8_t Index);

void DeveloperAPI_TestGetFunc20_29(int8_t Index);

void DeveloperAPI_TestGetFunc30_32(int8_t Index);

void BydAuto_DeveloperAPI_SetTest( void );

void DeveloperAPI_TestItemDisplay( void );


// Get functions Test
void GetVelocityInfo_TestDemo( void );                    // 01
void GetWheelSpeedInfo_TestDemo( void );                  // 02
void GetLampInfo_TestDemo( void );                        // 03
void GetLampTurnInfo_TestDemo( void );                    // 04
void GetLampFogInfo_TestDemo( void );                     // 05
void GetRainWiperInfo_TestDemo( void );                   // 06
void GetSwitchDriveModeFeedbackInfo_TestDemo( void );     // 07
void GetLampAutoInfo_TestDemo( void );                    // 08
void GetDoorInfo_TestDemo( void );                        // 09
void GetWheelDriveDirectionInfo_TestDemo( void );         // 10
void GetESPDriveModeFeedbackInfo_TestDemo( void );        // 11
void GetESPAutoDrivePermissionInfo_TestDemo( void );      // 12
void GetESPAliveInfo_TestDemo( void );                    // 13
void GetYawRateInfo_TestDemo( void );                     // 14
void GetAccelerationXInfo_TestDemo( void );               // 15
void GetAccelerationYInfo_TestDemo( void );               // 16
void GetWheelPulseCounterInfo_TestDemo( void );           // 17
void GetAccelerateDeepnessInfo_TestDemo( void );          // 18
void GetBrakeDeepnessInfo_TestDemo( void );               // 19
void GetBrakePedalSignalInfo_TestDemo( void );            // 20
void GetCarGearInfo_TestDemo( void );                     // 21
void GetGearDriveModeFeedbackInfo_TestDemo( void );       // 22
void GetEleBrakeInfo_TestDemo( void );                    // 23
void GetEleBrakeDriveModeFeedbackInfo_TestDemo( void );   // 24
void GetAngularInfo_TestDemo( void );                     // 25
void GetEPSDriveModeFeedbackInfo_TestDemo( void );        // 26
void GetDriveAutoPermissionInfo_TestDemo( void );         // 27
void GetRotationSpeedInfo_TestDemo( void );               // 28
void GetMotorTorqueInfo_TestDemo( void );                 // 29
void GetMCDriveModeFeedbackInfo_TestDemo( void );         // 30
void GetAutoDriveKeyInfo_TestDemo( void );                // 31
void GetEmergencyExitKeyInfo_TestDemo( void );            // 32


// Set functions Test
void DeveloperAPI_SetTestItemDisplay( void );
void SendAutoDriveModeRequest_TestDemo( void );           // 01
void SetLampAutoSwitch_TestDemo( void );                  // 02
void SetLampTurnSwitch_TestDemo( void );                  // 03
void SetLampBrakeSwitch_TestDemo( void );                 // 04
void SetAngular_TestDemo( void );                         // 05
void SetEpbState_TestDemo( void );                        // 06
void SetRainWiper_TestDemo( void );                       // 07
void SetCarGear_TestDemo( void );                         // 08
void SetDriveModeFeedback_TestDemo( void );               // 09
void SetAcceleratedVelocity_TestDemo( void );             // 10
void SetDriveAutoState_TestDemo( void );                  // 11
void SendAliveCount_TestDemo( void );                     // 12
void SetLampEmergencyWarningSwitch_TestDemo( void );      // 13
void SetLampOutSideSwitch_TestDemo( void );               // 14
void SetHornSwitch_TestDemo( void );                      // 15
void SetMultimediaDisplaySwitch_TestDemo( void );         // 16
void SetAutoDriveLateralState_TestDemo( void );           // 17


//Auto Driving Mode
void DeveloperAPI_AutoDrive_Init( void );                  //需结合实际情景进行设置，尤其是档位、手刹、加速度等
int32_t DeveloperAPI_AutoDrive_ModeRequest( void );
int32_t DeveloperAPI_AutoDrive_ModeFeedback( uint8_t mode_flag );
void DeveloperAPI_AutoDrive_Control( void );
int32_t DeveloperAPI_AutoDrive_QuitRequest( void );
void DeveloperAPI_AutoDrive_ControlItemDisplay( void );


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

