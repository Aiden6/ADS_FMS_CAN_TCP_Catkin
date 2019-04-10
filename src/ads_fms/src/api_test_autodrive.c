/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称:  api_test_autodrive.c
  摘 要:     API使用例程

  其 他:
  函数列表:

  当前版本: 1.0
  作 者:
  修改内容:
  完成日期: 2018年11月14日
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include "../include/byd/api_test.h"

/** Global variables **/
extern int32_t  gTestFlag; 
extern int8_t   gCharInput[STRING_LEN];
extern sem_t    sem_event;
extern uint8_t  driving_mode;


/*****************************************************************************
  函数名称:       void DeveloperAPI_AutoDrive_Init( void )
  说明:           此为示例，所以此初始化数据需结合实际情景
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过SET接口对各个执行器进行控制   
*****************************************************************************/
void DeveloperAPI_AutoDrive_Init( void )
{
	uint16_t nAngular = 0;
	uint64_t nTimeStamp = 0;
	int16_t tempAng = 0;
	SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING);	    //ACU发出保持当前驾驶模式
	SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_CLOSE);    //智能驾驶横向控制关闭
	SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE);	//智能驾驶纵向控制关闭
	SetCarGear(BYD_AUTO_CAR_GEAR_N);                            //设置当前示例档位为N档
	SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_NORMAL);	        //设置ACU模式反馈为正常驾驶模式
	SetLampAutoSwitch(BYD_AUTO_LAMP_AUTO_ON);	                //设置为自动灯光，进入之后才相应操作
	SetEpbState(BYD_AUTO_EPB_SET_RELEASED);	                    //设置EPB为释放状态
	GetAngularInfo(&nAngular,&nTimeStamp);
	tempAng = (int16_t)nAngular;
	tempAng = (int16_t)(0.1*tempAng);
	nAngular = (uint16_t)((780 + tempAng)*10);
	SetAngular(nAngular); 	                                    //设置方向盘角度
//	SetAngularVelocity(200);                                    //设置方向盘角速度为200°/s
	SetAcceleratedVelocity(100);                                //设置目标加速度为0m/s^2
	printf("The AutoDrive init is done!\n");
}

/*****************************************************************************
  函数名称:       int32_t DeveloperAPI_AutoDrive_Request( void )
  说明:           获取执行器的驾驶模式反馈，ACU也进行反馈
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         返回成功项
  被访问变量:
  被修改变量:
  其他:           通过SET接口对各个执行器进行控制   
*****************************************************************************/
int32_t DeveloperAPI_AutoDrive_Request( void )
{
	static int i = 0;
	int status = BYD_AUTODRIVE_REQUEST_ERR;
	uint16_t nAngular = 0;
	int16_t tempAng = 0;
	uint64_t nTimeStamp = 0;
	uint8_t m_automode_esp_feedback = 0;
	uint8_t m_automode_sc_feedback = 0;
	uint8_t m_automode_eps_feedback = 0;
	uint8_t m_automode_vcu_feedback = 0;
	uint8_t m_automode_epb_feedback = 0;
	uint8_t m_automode_scu_feedback = 0;

	while(i < 100)
	{
	  	i++;
	  	usleep(5000);
		if(ERR_BYD_AUTO_OK != GetESPDriveModeFeedbackInfo(&m_automode_esp_feedback,&nTimeStamp))       //获取ESP驾驶模式反馈
		{
			printf("State Acquisition：GetESPDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetEPSDriveModeFeedbackInfo(&m_automode_eps_feedback,&nTimeStamp))       //获取EPS驾驶模式反馈
		{
			printf("State Acquisition：GetEPSDriveModeFeedbackInfo error!\n");
		}	
		if(ERR_BYD_AUTO_OK != GetSwitchDriveModeFeedbackInfo(&m_automode_sc_feedback,&nTimeStamp))     //获取组合开关驾驶模式反馈
		{
			printf("State Acquisition：GetSwitchDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetEleBrakeDriveModeFeedbackInfo(&m_automode_epb_feedback,&nTimeStamp))  //获取电子手刹驾驶模式反馈
		{
			printf("State Acquisition：GetEleBrakeDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetMCDriveModeFeedbackInfo(&m_automode_vcu_feedback,&nTimeStamp))        //获取电机控制器驾驶模式反馈
		{
			printf("State Acquisition：GetMCDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetGearDriveModeFeedbackInfo(&m_automode_scu_feedback,&nTimeStamp))     //获取档位驾驶模式反馈
		{
			printf("State Acquisition：GetGearDriveModeFeedbackInfo error!\n");
		}

	  	if(	BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_OPEN == m_automode_esp_feedback &&
			BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_AUTO == m_automode_sc_feedback &&
			BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_AUTO == m_automode_eps_feedback &&
			BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_AUTO == m_automode_vcu_feedback &&
			BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_AUTO == m_automode_epb_feedback &&
			BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_AUTO == m_automode_scu_feedback)
	  	{
			if(ERR_BYD_AUTO_OK != SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING))
			{
				printf("Command Setting：SendAutoDriveModeRequest Holding error!\n");
			}
			if(ERR_BYD_AUTO_OK != SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO))    //设置ACU智能驾驶模式并反馈
			{
				printf("Command Setting：SetDriveModeFeedback error!\n");
				status = BYD_AUTODRIVE_REQUEST_ERR;
			}
			else
			{	
				
				driving_mode = AUTO_MODE;
				GetAngularInfo(&nAngular,&nTimeStamp);
				tempAng = (int16_t)nAngular;
				tempAng = (int16_t)(0.1*tempAng);
				nAngular = (uint16_t)((780 + tempAng)*10);
				SetAngular(nAngular); 	                                             //设置方向盘角度
				/*  if(ERR_BYD_AUTO_OK != SetAngularVelocity(300))                     //设置方向盘角速度，秦PRO不需要设置，暂时留着
				{
					printf("Command Setting：SetAngularVelocity error!\n");		
				}*/

				status = BYD_AUTODRIVE_REQUEST_OK;
				i = 0;
				break;
			}  	 		 
	  	}
	  	else
	  	{
			if(99 == i)
			{
				/*用来判断哪些进入，哪些未进入*/					
				switch(m_automode_esp_feedback)
				{
				case 0: 
					printf("The ESP is close！\n");
					break;
				case 1:
					printf("The ESP is open！\n");
				break;
				default:
					break;
		
				}

				switch(m_automode_eps_feedback)
				{
				case 0:
					printf("The EPS is invalid!\n");
					break;
				case 1:
					printf("The EPS is in the normal drive!\n");
					break;
				case 2:
					printf("The EPS is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_sc_feedback)
				{
				case 0:
					printf("The Switch is invalid!\n");
					break;
				case 1:
					printf("The Switch is in the normal drive!\n");
					break;
				case 2:
					printf("The Switch is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_vcu_feedback)
				{
				case 0:
					printf("The MOTOR is enter the normal drive!\n");
					break;
				case 1:
					printf("The MOTOR is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_epb_feedback)
				{
				case 0:
					printf("The EPB is invalid!\n");
					break;
				case 1:
					printf("The EPB is in the normal drive!\n");
					break;
				case 2:
					printf("The EPB is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_scu_feedback)
				{
				case 0:
					printf("The Gear is enter the normal drive!\n");
					break;
				case 1:
					printf("The Gear is in the auto drive!\n");
					break;
				default:
					break;
				}						 
						 
				SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_NORMAL);                 //请求进入智能驾驶失败后，发送人工驾驶模式
				SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_CLOSE);              //ACU向整车发出退出智能驾驶请求	
				SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_CLOSE);          //智能驾驶横向控制关闭
				SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE);	      //智能驾驶纵向控制关闭
				driving_mode = NORMAL_MODE;
				GetAngularInfo(&nAngular,&nTimeStamp);
				tempAng = (int16_t)nAngular;
				tempAng = (int16_t)(0.1*tempAng);
				nAngular = (uint16_t)((780 + tempAng)*10);
				SetAngular(nAngular); 
				printf("the autodrive request is failed!\n");
				status = BYD_AUTODRIVE_REQUEST_ERR; 
			} 	 		 	
	  	}
	}
	if(100 == i)
	{
		i = 0;
	}
	return status;
}

/*****************************************************************************
  函数名称:       int32_t DeveloperAPI_AutoDrive_Quit( void )
  说明:           获取执行器的驾驶模式反馈，ACU请求退出
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         返回成功项
  被访问变量:
  被修改变量:
  其他:           退出自动驾驶请求   
*****************************************************************************/
int32_t DeveloperAPI_AutoDrive_Quit( void )
{
	static int j = 0;
	int status = BYD_AUTODRIVE_QUIT_FAIL;
	uint64_t nTimeStamp = 0;
	uint8_t m_automode_esp_feedback = 0;
	uint8_t m_automode_sc_feedback = 0;
	uint8_t m_automode_eps_feedback = 0;
	uint8_t m_automode_vcu_feedback = 0;
	uint8_t m_automode_epb_feedback = 0;
	uint8_t m_automode_scu_feedback = 0;

	while(j < 200)
	{
		j++;
	  	usleep(5000);
		if(ERR_BYD_AUTO_OK != GetESPDriveModeFeedbackInfo(&m_automode_esp_feedback,&nTimeStamp))       //获取ESP驾驶模式反馈
		{
			printf("State Acquisition：GetESPDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetEPSDriveModeFeedbackInfo(&m_automode_eps_feedback,&nTimeStamp))       //获取EPS驾驶模式反馈
		{
			printf("State Acquisition：GetEPSDriveModeFeedbackInfo error!\n");
		}	
		if(ERR_BYD_AUTO_OK != GetSwitchDriveModeFeedbackInfo(&m_automode_sc_feedback,&nTimeStamp))     //获取组合开关驾驶模式反馈
		{
			printf("State Acquisition：GetSwitchDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetEleBrakeDriveModeFeedbackInfo(&m_automode_epb_feedback,&nTimeStamp))  //获取电子手刹驾驶模式反馈
		{
			printf("State Acquisition：GetEleBrakeDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetMCDriveModeFeedbackInfo(&m_automode_vcu_feedback,&nTimeStamp))        //获取电机控制器驾驶模式反馈
		{
			printf("State Acquisition：GetMCDriveModeFeedbackInfo error!\n");
		}
		if(ERR_BYD_AUTO_OK != GetGearDriveModeFeedbackInfo(&m_automode_scu_feedback,&nTimeStamp))     //获取档位驾驶模式反馈
		{
			printf("State Acquisition：GetGearDriveModeFeedbackInfo error!\n");
		}

		if(BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_CLOSE == m_automode_esp_feedback &&
		   BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_NORMAL == m_automode_eps_feedback &&   
		   BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_NORMAL == m_automode_vcu_feedback &&
		   BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_NORMAL == m_automode_sc_feedback &&
		   BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_NORMAL == m_automode_epb_feedback &&
		   BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_NORMAL == m_automode_scu_feedback)
	  	{ 
			printf("The AutoDrive is quit!\n");
	  	    driving_mode = NORMAL_MODE; 
			status = BYD_AUTODRIVE_QUIT_OK;
			SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING);
			j = 0;
			break;
	  	}	
	  	else
	  	{
	  	    if(99 == j)
			{
				/*用来判断哪些进入，哪些未进入*/					
				switch(m_automode_esp_feedback)
				{
				case 0: 
					printf("The ESP is close！\n");
					break;
				case 1:
					printf("The ESP is open！\n");
				break;
				default:
					break;
		
				}

				switch(m_automode_eps_feedback)
				{
				case 0:
					printf("The EPS is invalid!\n");
					break;
				case 1:
					printf("The EPS is in the normal drive!\n");
					break;
				case 2:
					printf("The EPS is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_sc_feedback)
				{
				case 0:
					printf("The Switch is invalid!\n");
					break;
				case 1:
					printf("The Switch is in the normal drive!\n");
					break;
				case 2:
					printf("The Switch is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_vcu_feedback)
				{
				case 0:
					printf("The MOTOR is enter the normal drive!\n");
					break;
				case 1:
					printf("The MOTOR is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_epb_feedback)
				{
				case 0:
					printf("The EPB is invalid!\n");
					break;
				case 1:
					printf("The EPB is in the normal drive!\n");
					break;
				case 2:
					printf("The EPB is in the auto drive!\n");
					break;
				default:
					break;
				}

				switch(m_automode_scu_feedback)
				{
				case 0:
					printf("The Gear is enter the normal drive!\n");
					break;
				case 1:
					printf("The Gear is in the auto drive!\n");
					break;
				default:
					break;
				}
				SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_CLOSE);              //ACU向整车发出退出智能驾驶请求	  
				SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE);	      //智能驾驶纵向控制关闭
				SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_CLOSE);          //智能驾驶横向控制关闭
				status = BYD_AUTODRIVE_QUIT_FAIL;
			}
	  	}	
	}
	if(100 == j)
	{
		j = 0;
	}
	return status;
}


/*****************************************************************************
  函数名称:       int32_t DeveloperAPI_AutoDrive_ModeFeedback( uint8_t mode_flag )
  说明:           获取执行器的驾驶模式反馈，ACU也进行反馈
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         返回成功项
  被访问变量:
  被修改变量:
  其他:           通过SET接口对各个执行器进行控制   
*****************************************************************************/
int32_t DeveloperAPI_AutoDrive_ModeFeedback( uint8_t mode_flag )
{
	int autodrive_flag = mode_flag;
	int status = BYD_AUTODRIVE_REQUEST_OK;
	switch(autodrive_flag)
	{
	  	case AUTODRIVE_REQUEST:
	  	{
		 	printf("Now in the state of AutoDrive Request!\n");
	  		status = DeveloperAPI_AutoDrive_Request();		
	  		break;
	  	}
	  	case AUTODRIVE_QUIT:
	  	{
		  	printf("Now in the state of AutoDrive Quit!\n");
			status = DeveloperAPI_AutoDrive_Quit();
	  	    break;	
	  	}
	  	default:
	  		break;
    }
	
	return status;
}


/*****************************************************************************
  函数名称:       void DeveloperAPI_AutoDrive_PointBrake( void )
  说明:           点刹方式来减少车速最终停止
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           平稳停止车速  
*****************************************************************************/
void DeveloperAPI_AutoDrive_PointBrake( void )
{
	SetAcceleratedVelocity(100);   //0
	msleep(40);
	SetAcceleratedVelocity(90);    //-0.5
	msleep(100);
	SetAcceleratedVelocity(80);    //-1
	msleep(100);
	SetAcceleratedVelocity(70);    //-1.5
	msleep(100);
	SetAcceleratedVelocity(60);    //-2
	msleep(100);
	SetAcceleratedVelocity(50);    //-2.5
	msleep(100); 
	SetAcceleratedVelocity(40);    //-3
	msleep(100); 
	SetAcceleratedVelocity(30);    //-3.5
	msleep(100); 
	SetAcceleratedVelocity(60);    //-2
	msleep(100);
	SetAcceleratedVelocity(70);    //-1.5
	msleep(200);
	SetAcceleratedVelocity(80);    //-1
	msleep(200);
}


/*****************************************************************************
  函数名称:       void DeveloperAPI_AutoDrive_StopVehicle( void )
  说明:           通过设置加速度来减少车速最终停止
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           用于在车控过程中，要将运行中的车辆快速停止  
*****************************************************************************/
void DeveloperAPI_AutoDrive_StopVehicle( void )
{
	uint16_t nVelocity = 0;
    uint64_t nTimeStamp = 0;

	if(ERR_BYD_AUTO_OK == GetVelocityInfo(&nVelocity,&nTimeStamp))
	{
		nVelocity = (uint16_t)(nVelocity*0.0687 + 0.5);
	}
	
	while(0 < nVelocity)
	{
		//如果遇到紧急情况，请人工踩刹车，以下仅作为测试
		DeveloperAPI_AutoDrive_PointBrake();
		if(ERR_BYD_AUTO_OK == GetVelocityInfo(&nVelocity,&nTimeStamp))
		{
			nVelocity = (uint16_t)(nVelocity*0.0687 + 0.5);
		}
		printf("The current speed of vehicle is %d Km/h.\n",nVelocity);
	}

	SetAcceleratedVelocity(100);   //0 保持当前车速状态，此时应该是车速零
	printf("The current speed of vehicle is %d Km/h.\n",nVelocity);

}


/*****************************************************************************
  函数名称:       void DeveloperAPI_AutoDrive_ControlItemDisplay( void )
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
void DeveloperAPI_AutoDrive_ControlItemDisplay( void )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Please enter 'o' or 'O' to Quit AutoDrive.\n");
    printf("Please enter 'h' or 'H' to Show the Item again.\n");
	printf("********************************************************************\n");
	printf("**Under the automatic driving mode,please don't set s1、s9 and s11**\n"
		   "**that will change the state of the automatic driving mode!       **\n"
		   "**And enter 'b' or 'B' can reduce the speed and stop the vehicle! **\n");
	printf("********************************************************************\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Otherwise, enter:\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+Get the state of vehicle under the automatic driving mode!-+-+-+-+-+-+-+-+-+-+-+-+\n");
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
    printf("g12 or G12: GetESPAutoDrivePermissionInfo(uint8_t *pnAutoDrivePermission, uint64_t *pnTimeStamp)\n");
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
	printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+Set the state of vehicle under the automatic driving mode!-+-+-+-+-+-+-+-+-+-+-+-+\n");
	printf("b   or B:   It' used to reduce the speed of vehicle and stop the vehicle!\n");
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
  函数名称:       void DeveloperAPI_AutoDrive_Control( void )
  说明:           进入智能驾驶，进行控制操作
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         返回成功项
  被访问变量:
  被修改变量:
  其他:           通过SET接口对各个执行器进行控制   
*****************************************************************************/
void DeveloperAPI_AutoDrive_Control( void )
{
    DeveloperAPI_AutoDrive_ControlItemDisplay();
    while (gTestFlag) 
    {
        sem_wait(&sem_event);

	/*	if(ERR_BYD_AUTO_OK == DeveloperAPI_AutoDrive_ModeFeedback(AUTODRIVE_QUIT))   //实际车辆测试，需改回“==”
		{
    		printf("The actuator of vehicle is out of automatic driving mode!\n");
			break;
		}*/


        if(0x1 == gCharInput[0] && 
            ('o' == gCharInput[1] || 'O' == gCharInput[1])) 
        {
            if(BYD_AUTODRIVE_QUIT_OK == DeveloperAPI_AutoDrive_QuitRequest())
            {                
                break;
            }
			DeveloperAPI_AutoDrive_ControlItemDisplay();
        }
		if(0x1 == gCharInput[0] &&
			('b' == gCharInput[1] || 'B' == gCharInput[1]))
		{
			DeveloperAPI_AutoDrive_StopVehicle();
			DeveloperAPI_AutoDrive_ControlItemDisplay();
		}

		if(0x1 == gCharInput[0] &&
			('h' == gCharInput[1] || 'H' == gCharInput[1]))
		{
			DeveloperAPI_AutoDrive_ControlItemDisplay();
		}


        if (0x2 == gCharInput[0] && 
            ('s' == gCharInput[1] || 'S' == gCharInput[1])) {
            DeveloperAPI_TestSetFunc1_9(gCharInput[2]);
        }

  /*      if (0x2 == gCharInput[0] && 0xe0 == gCharInput[1]) {
            printf("the 键盘 is pressed！\n");
        }*/


        if (0x3 == gCharInput[0] &&
           ('s' == gCharInput[1] || 'S' == gCharInput[1])) {
            // S10 ~ S17
            if ('1' != gCharInput[2]) {
                continue;
            }
            DeveloperAPI_TestSetFunc10_17(gCharInput[3]);
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
    printf("API AutoDriving Testing Quit.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("\n");    
}

/*****************************************************************************
  函数名称:       int32_t DeveloperAPI_AutoDrive_ModeRequest( void )
  说明:           调用相关SET的API函数请求进入智能驾驶模式
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过SET接口对各个执行器进行控制   
*****************************************************************************/
int32_t DeveloperAPI_AutoDrive_ModeRequest( void )
{
	uint16_t nAngular = 0;
	int16_t tempAng = 0;
	uint64_t nTimeStamp = 0;
	GetAngularInfo(&nAngular,&nTimeStamp);
	tempAng = (int16_t)nAngular;
	tempAng = (int16_t)(0.1*tempAng);
	nAngular = (uint16_t)((780 + tempAng)*10);
	SetAngular(nAngular);
	SetCarGear(BYD_AUTO_CAR_GEAR_N);
	SetEpbState(BYD_AUTO_EPB_SET_RELEASED);
	SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN);	 //智能驾驶纵向控制开启
	SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_OPEN); //智能驾驶横向控制关闭
	SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_OPEN);      //ACU向整车发出进入智能驾驶请求
    if(ERR_BYD_AUTO_OK == DeveloperAPI_AutoDrive_ModeFeedback(AUTODRIVE_REQUEST))
    {
		return BYD_AUTODRIVE_REQUEST_OK;
    }
    else
    {
    	return BYD_AUTODRIVE_REQUEST_ERR;	
    }
}

/*****************************************************************************
  函数名称:       int32_t DeveloperAPI_AutoDrive_QuitRequest( void )
  说明:           调用相关SET的API函数请求退出智能驾驶模式
  调用:
  被调用:
  输入参数:       void
                 		
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           通过SET接口对各个执行器进行控制   
*****************************************************************************/
int32_t DeveloperAPI_AutoDrive_QuitRequest( void )
{
	//设置加速度为0m/s^2，本案例是在车辆车速为零下，请求智能驾驶退出,实际行驶中，加速度另外控制。
    SetAcceleratedVelocity(100);
	SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_CLOSE);              //ACU向整车发出退出智能驾驶请求	
	SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_CLOSE);          //智能驾驶横向控制关闭
	SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE);	      //智能驾驶纵向控制关闭                         
    if(ERR_BYD_AUTO_OK == DeveloperAPI_AutoDrive_ModeFeedback(AUTODRIVE_QUIT))
    {
    	return BYD_AUTODRIVE_QUIT_OK;
    }
    else
    {
	  	printf("Command Setting：Automatic driving exit failure!\n"); 
		return BYD_AUTODRIVE_QUIT_FAIL;	
    }   
}
