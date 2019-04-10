/*****************************************************************************
  Copyright (c) 2018, 比亚迪汽车
  All rights reserved.

  文件名称: idriving_api.h
  摘 要:    API接口的定义

  其他:
  函数列表:

  当前版本: 1.0
  作 者:
  修改内容:
  完成日期: 2018年05月17日
*****************************************************************************/
#ifndef _IDRIVING_API_H_
#define _IDRIVING_API_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "std_types.h"
/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
#define VEHICLE_HADE

/* 返回值状态 */
#define ERR_BYD_AUTO_OK								(0)		// 成功 
#define ERR_BYD_AUTO_CAN_ENABLE						(-1)	// CAN使能错误 
#define ERR_BYD_AUTO_MEM_MALLOC						(-2)	// 内存申请错误 
#define ERR_BYD_AUTO_INVALID_PARAM					(-3)	// 输入参数错误 
#define ERR_BYD_AUTO_DISENABLE						(-4)	// 失败 
#define ERR_BYD_AUTO_WRITE							(-5)	// 向CAN网络写失败 
#define ERR_BYD_AUTO_READ							(-6)	// 读CAN数据失败 
#define ERR_BYD_AUTO_REPEATE_CALL					(-7) 	// 多次重复调用（不能重复调用，取消使能后才能再次调用） 

/* 驾驶模式 */ 
#define BYD_AUTO_DRIVE_MODE_HOLDING 				(0)		// 保持当前驾驶模式
#define BYD_AUTO_DRIVE_MODE_OPEN    				(1)		// 开启智能驾驶模式 
#define BYD_AUTO_DRIVE_MODE_CLOSE   				(2)		// 关闭智能驾驶模式

/* 灯光 */ 
#define BYD_AUTO_LAMP_AUTO_INVALID					(0) 	// AUTO档无效 
#define BYD_AUTO_LAMP_AUTO_ON       				(1)		// AUTO档
#define BYD_AUTO_LAMP_AUTO_OFF      				(2)		// 非AUTO档

#define BYD_AUTO_LAMP_ON							(1)		// 小灯/近光灯/远光灯/左转灯/右转灯/刹车灯打开 
#define BYD_AUTO_LAMP_OFF							(0)		// 小灯/近光灯/远光灯/左转灯/右转灯/刹车灯关闭 

#define BYD_AUTO_LAMP_SMALL                         (0)     // 小灯
#define BYD_AUTO_LAMP_LOW                           (1)     // 近光灯
#define BYD_AUTO_LAMP_HIGH                          (2)     // 远光灯
#define BYD_AUTO_LAMP_FRONT_FOG                     (3)     // 前雾灯
#define BYD_AUTO_LAMP_REAR_FOG                      (4)     // 后雾灯

/* 喇叭 */
#define BYD_AUTO_HORN_ON							(1)
#define BYD_AUTO_HORN_OFF							(0)

/* EPB */ 
#define BYD_AUTO_EPB_SET_INVALID					(0)		// EPB无效
#define BYD_AUTO_EPB_SET_APPLYING					(1)		// EPB拉起
#define BYD_AUTO_EPB_SET_RELEASED					(2)		// EPB释放  


/* 雨刮 */ 
#define BYD_AUTO_RAIN_WIPER_INVALID                 (0)     // 无效
#define BYD_AUTO_RAIN_WIPER_OFF						(1)		// OFF档 
#define BYD_AUTO_RAIN_WIPER_SHORT_PRESS				(2)		// 点刮短按 
#define BYD_AUTO_RAIN_WIPER_LONG_PRESS				(3)		// 点刮长按 
#define BYD_AUTO_RAIN_WIPER_INTERVAL_1				(4)		// 间隙1档 
#define BYD_AUTO_RAIN_WIPER_INTERVAL_2				(5)		// 间隙2档 
#define BYD_AUTO_RAIN_WIPER_INTERVAL_3				(6)		// 间隙3档 
#define BYD_AUTO_RAIN_WIPER_INTERVAL_4				(7)		// 间隙4档 
#define BYD_AUTO_RAIN_WIPER_SLOW					(8)		// 慢刮 
#define BYD_AUTO_RAIN_WIPER_QUICK					(9)		// 快刮 
#define BYD_AUTO_RAIN_WIPER_FAULT					(15)	// 预留 

/* 档位 */ 
#define BYD_AUTO_CAR_GEAR_INVALID					(0)		// 无效（无换档动作） 
#define BYD_AUTO_CAR_GEAR_P							(1)		// P
#define BYD_AUTO_CAR_GEAR_R							(2)		// R
#define BYD_AUTO_CAR_GEAR_N							(3)		// N
#define BYD_AUTO_CAR_GEAR_D							(4)		// D
#define BYD_AUTO_CAR_GEAR_FAULT						(7)		// 错误

/* 驾驶模式反馈 */ 
#define BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1			(0)		// 智能驾驶异常1（取消显示） 
#define BYD_AUTO_DRIVE_MODE_AUTO					(1)		// 智能驾驶模式 
#define BYD_AUTO_DRIVE_MODE_NORMAL					(2)		// 正常驾驶模式 
#define BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2			(3)		// 智能驾驶异常2（故障模式） 

/* 智能驾驶纵向控制 */ 
#define BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE		(0)		// 纵向控制关闭状态 
#define BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN			(3)		// 纵向控制开启状态 

/* 组合开关智能驾驶模式反馈 */ 
#define BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_INVALID	(0)		// 无效 
#define BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_NORMAL	(1)		// 正常驾驶模式 
#define BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_AUTO	(2)		// 智能驾驶模式

/* 车门状态 */
#define BYD_AUTO_DOOR_OFF							(0)     // 车门关闭
#define BYD_AUTO_DOOR_ON							(1)     // 车门打开
#define BYD_AUTO_TRUNK_OFF                          (0)     // 行李箱关闭
#define BYD_AUTO_TRUNK_ON                           (1)     // 行李箱打开

/* 车轮行驶方向 */ 
#define BYD_AUTO_DRIVE_DIRECTION_UNDEFINE			(0)		// 未定义 
#define BYD_AUTO_DRIVE_DIRECTION_FORWARD			(1)		// 向前 
#define BYD_AUTO_DRIVE_DIRECTION_BACKWARD			(2)		// 向后 
#define BYD_AUTO_DRIVE_DIRECTION_STOP				(3)		// 停止

/* ESP进入智能驾驶允许 */
#define BYD_AUTO_DRIVE_ESP_NOT_PERMISSION           (0)     // 不允许进入
#define BYD_AUTO_DRIVE_ESP_PERMISSION_OK            (1)     // 允许进入


/* ESP当前驾驶模式 */ 
#define BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_CLOSE		      (0)     // 纵向控制关闭 
#define BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_OPEN		      (1)	  // 纵向控制开启 

/* ESP故障信息 */
#define BYD_AUTO_ESP_ERROR_NO_FAILURE                     (0)     // 无故障
#define BYD_AUTO_ESP_ERROR_FAILURE                        (1)     // 有故障

/* ESP激活状态信息 */ 
// #define BYD_AUTO_DRIVE_MODE_ESP_NO_ACTIVE			  (0)	  // 纵向控制未激活 
// #define BYD_AUTO_DRIVE_MODE_ESP_ACTIVE				  (1)	  // 纵向控制已激活 

/* 制动踏板的状态信息 */
#define BYD_AUTO_BRAKE_PEDAL_SIGNAL_NO_PRESSED            (0)     // 未踩下
#define BYD_AUTO_BRAKE_PEDAL_SIGNAL_PRESSED               (1)     // 踩下
#define BYD_AUTO_BRAKE_PEDAL_SIGNAL_ERR                   (3)     // 信号错误

/* 档位的智能驾驶模式反馈 */
#define BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_NORMAL          (0)     // 正常驾驶模式
#define BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_AUTO            (1)     // 智能驾驶模式

/* 电子手刹（EPB）状态 */ 
#define BYD_AUTO_EPB_GET_RELEASING					      (0)	  // 正在释放 
#define BYD_AUTO_EPB_GET_RELEASED					      (1)	  // 释放 
#define BYD_AUTO_EPB_GET_APPLYING					      (2)	  // 正在拉起 
#define BYD_AUTO_EPB_GET_APPLIED					      (3)	  // 拉起 
#define BYD_AUTO_EPB_GET_BEAKE_FAULT				      (4)	  // 错误 

/* 电子手刹（EPB）驾驶模式反馈 */
#define BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_INVALID    (0)     // 无效
#define BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_NORMAL     (1)     // 正常驾驶模式
#define BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_AUTO       (2)     // 智能驾驶模式


/* EPS进入智能驾驶模式请求 */
#define BYD_AUTO_DRIVE_MODE_EPS_CLOSE    			      (0)		// 关闭EPS智能驾驶模式
#define BYD_AUTO_DRIVE_MODE_EPS_OPEN   				      (1)		// 开启EPS智能驾驶模式 

/* EPS当前驾驶模式 */ 
#define BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_INVALID	      (0)	  // 无效 
#define BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_NORMAL		      (1)	  // 正常驾驶模式 
#define BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_AUTO		      (2)	  // 智能驾驶模式 

/* EPS进入智能驾驶允许 */ 
#define BYD_AUTO_EPS_PERMISSION_INVALID				      (0)	  // 无效 
#define BYD_AUTO_EPS_PERMISSION_OK					      (1)     // 允许进入智能驾驶 
#define BYD_AUTO_EPS_NOT_PERMISSION					      (2)	  // 不允许进入智能驾驶


/* 多媒体相关按键信息 */ 
#define BYD_AUTO_MEDIUM_KEY_INVALID					      (0)	  // 无效 
#define BYD_AUTO_MEDIUM_KEY_STOP_SWITCH				      (2)	  // 紧急停止开关 
#define BYD_AUTO_MEDIUM_KEY_DRIVE_AUTO_SWITCH		      (5)	  // 智能驾驶主开关 

/* 电机控制器驾驶模式反馈 */ 
#define BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_NORMAL		      (0)	  // 正常驾驶模式 
#define BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_AUTO		      (1)	  // 智能驾驶模式 

/* MC压力值状态 & MC压力偏移值状态 */ 
#define BYD_AUTO_DRIVE_MODE_MC_P_VALID				      (0)	  // 有效 
#define BYD_AUTO_DRIVE_MODE_MC_P_INVALID			      (1)     // 无效 



/* 控制报文ID标识 */ 
#define BYD_AUTO_IDRIVING_CMD_TOTE                        (6)

#define BYD_AUTO_IDRIVING_CMD_0						(0x12F82340u)	//  周期，10ms
#define BYD_AUTO_IDRIVING_CMD_1						(0x12F83A30u)	//  周期，20ms
#define BYD_AUTO_IDRIVING_CMD_2						(0x12F832E0u)	//  周期，20ms
#define BYD_AUTO_IDRIVING_CMD_3						(0x12F832D0u)	//  周期，20ms
#define BYD_AUTO_IDRIVING_CMD_4						(0x12F832F0u)	//  周期，20ms
#define BYD_AUTO_IDRIVING_CMD_5						(0x12F83380u)	//  事件周期，100ms



/*=========================*/
/*  Add for HADE vehicle   */
/*=========================*/

/* 多媒体显示开关 */
#define BYD_AUTO_MULTIMEDIA_DISPLAY_INVALID         (0)   // 无效
#define BYD_AUTO_MULTIMEDIA_DISPLAY_ON              (1)   // 开启
#define BYD_AUTO_MULTIMEDIA_DISPLAY_OFF             (2)   // 关闭


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/** #########################################################################
 *
 *  \智能驾驶初始化相关API.
 */
 


/** ----------------------------------------------------------------
 *  \功能： 智能驾驶API数据初始化 
 *
 *  \参数： pSetBuf	[输入]		智能驾驶设置模块指针
 *			pGetBuf	[输入]		智能驾驶接收模块指针 
 *
 *  \返回值： 初始化成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t BydAutoApiDataInit();


/** #########################################################################
 *
 *  \智能驾驶 Runtime API.
 */

/** ----------------------------------------------------------------
 *  \功能： CAN报文接收后处理 
 *
 *  \参数： messageId	  [输入]		  报文ID
 *  \参数： messageLen    [输入]          报文长度
 *  \参数： pMessageData  [输入]          报文数据
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t BydAutoCANRuntimeRcv( uint32_t messageId,
                              uint32_t messageLen, 
                              uint8_t *pMessageData );


/** ----------------------------------------------------------------
 *  \功能： CAN报文发送前处理 
 *
 *  \参数： messageId	  [输入]		  报文ID
 *  \参数： pMessageLen   [输入 & 输出]   报文长度
 *  \参数： pMessageData  [输入 & 输出]   报文数据
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t BydAutoCANRuntimeXmt( uint32_t messageId,
                              uint32_t *pMessageLen, 
                              uint8_t  *pMessageData );


/** #########################################################################
 *
 *  \智能驾驶模块设置相关API.
 */

/** ----------------------------------------------------------------
 *  \功能： 向各功能模块发送驾驶模式请求 
 *
 *  \参数： nModeRequest	[输入]		驾驶模式 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SendAutoDriveModeRequest( uint8_t nModeRequest );
  

/** ----------------------------------------------------------------
 *  \功能： 智能驾驶横向控制状态设置  
 *
 *  \参数： nAutodDriveLateralState	[输入]		智能驾驶横向控制状态控制
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetAutoDriveLateralState( uint8_t nAutodDriveLateralState );

/** ----------------------------------------------------------------
 *  \功能： 设置灯光AUTO档开关 
 *
 *  \参数： nAutoSwitch		[输入]		灯光AUTO档开关
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetLampAutoSwitch( uint8_t nAutoSwitch );


/** ----------------------------------------------------------------
 *  \功能： 转向灯设置 
 *
 *  \参数： nLeft	[输入]		左转向灯开关
 *			nRight	[输入]		右转向灯开关
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetLampTurnSwitch( uint8_t nLeft, uint8_t nRight );


/** ----------------------------------------------------------------
 *  \功能： 制动灯设置 
 *
 *  \参数： nBrakeSwitch	[输入]		制动灯开关
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetLampBrakeSwitch( uint8_t nBrakeSwitch );


/** ----------------------------------------------------------------
 *  \功能： 目标方向盘角度设置 
 *
 *  \参数： nAngular	[输入]		目标方向盘角度
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetAngular( uint16_t nAngular );


/** ----------------------------------------------------------------
 *  \功能： 目标EPB状态设置
 *
 *  \参数： nEpbState	[输入]		目标EPB状态	
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetEpbState( uint8_t nEpbState );


/** ----------------------------------------------------------------
 *  \功能： 目标雨刮档位设置 
 *
 *  \参数： nRainWiper	[输入]		目标雨刮档位
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetRainWiper( uint8_t nRainWiper );


/** ----------------------------------------------------------------
 *  \功能： 目标档位设置 
 *
 *  \参数： nCarGear	[输入]		目标档位
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetCarGear( uint8_t nCarGear );


/** ----------------------------------------------------------------
 *  \功能： 驾驶模式反馈设置  
 *
 *  \参数： nDriveModeFeedback	[输入]		驾驶模式反馈 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetDriveModeFeedback( uint8_t nDriveModeFeedback );


/** ----------------------------------------------------------------
 *  \功能： 目标方向盘角速度设置  
 *
 *  \参数： nAngularVelocity	[输入]		目标方向盘角速度
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetAngularVelocity( uint16_t nAngularVelocity );


/** ----------------------------------------------------------------
 *  \功能： 目标加速度设置  
 *
 *  \参数： nAcceleratedVelocity	[输入]		目标加速度
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetAcceleratedVelocity( uint8_t nAcceleratedVelocity );
  

/** ----------------------------------------------------------------
 *  \功能： 智能驾驶纵向控制状态设置  
 *
 *  \参数： nDriveAutoState	[输入]		智能驾驶纵向控制状态控制
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetDriveAutoState( uint8_t nDriveAutoState );


/** ----------------------------------------------------------------
 *  \功能： 智能驾驶API滚动计数及控制发送时序  
 *
 *  \参数： nCount	[输入]		滚动计数 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SendAliveCount( uint8_t nCount );


/** ----------------------------------------------------------------
 *  \功能： 紧急告警灯设置  
 *
 *  \参数： nSwitch	[输入]		开关状态 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetLampEmergencyWarningSwitch( uint8_t nSwitch );


/** ----------------------------------------------------------------
 *  \功能： 其他灯光信号设置  
 *
 *  \参数： nArea [输入]        灯光类型
 *          nSwitch	[输入]		开关状态 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetLampOutSideSwitch( uint8_t nArea, uint8_t nSwitch );


/** ----------------------------------------------------------------
 *  \功能： 喇叭设置  
 *
 *  \参数： nSwitch	[输入]		开关状态 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetHornSwitch( uint8_t nSwitch );


/** ----------------------------------------------------------------
 *  \功能： 多媒体显示设置  
 *
 *  \参数： nSwitch	[输入]		开关状态 
 *
 *  \返回值： 设置成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t SetMultimediaDisplaySwitch( uint8_t nSwitch );



/** #########################################################################
 *  \智能驾驶模块接收信息相关API.
 */

/** ----------------------------------------------------------------
 *  \功能： 获取车速信息 
 *
 *  \参数： pnVelocity	[输出]		车速
 *			pnTimeStamp	[输出]		时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetVelocityInfo( uint16_t *pnVelocity, uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车轮实时轮速信息
 *
 *  \参数：	 pnFL			[输出]	左前轮转速
 *			 pnFR			[输出]	右前轮转速
 *			 pnRL			[输出]	左后前轮转速
 *			 pnRR			[输出]	右后轮转速
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetWheelSpeedInfo( uint16_t *pnFL, uint16_t *pnFR, 
						   uint16_t *pnRL, uint16_t *pnRR, 
						   uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车辆灯光信息
 *
 *  \参数：	 pnSmall		[输出]	小灯开关状态
 *			 pnNear			[输出]	近光灯开关状态
 *			 pnFar			[输出]	远光灯开关状态
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetLampInfo( uint8_t *pnSmall, uint8_t *pnNear, 
					 uint8_t *pnFar, uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车辆转向灯信息
 *
 *  \参数：	 pnLeft			[输出]	左转向灯开关状态
 *			 pnRight		[输出]	右转向灯开关状态
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetLampTurnInfo( uint8_t *pnLeft, uint8_t *pnRight,
						 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车辆雾灯信息
 *
 *  \参数：	 pnFront		[输出]	前雾灯开关状态
 *			 pnRear			[输出]	后雾灯开关状态
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetLampFogInfo( uint8_t *pnFront, uint8_t *pnRear, 
						uint64_t *pnTimeStamp );		


/** ----------------------------------------------------------------
 *  \功能： 获取雨刮档位信息
 *
 *  \参数：	 pnRainWiper	[输出]	雨刮档位 
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */				 
int32_t GetRainWiperInfo( uint8_t *pnRainWiper, uint64_t *pnTimeStamp );	


/** ----------------------------------------------------------------
 *  \功能： 获取组合开关智能驾驶模式反馈信息
 *
 *  \参数：	 pnSwitchDriveModeFeedback	[输出]	组合开关智能驾驶模式反馈 
 *			 pnTimeStamp				[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */		
int32_t GetSwitchDriveModeFeedbackInfo( uint8_t *pnSwitchDriveModeFeedback, 
										uint64_t *pnTimeStamp );				 


/** ----------------------------------------------------------------
 *  \功能： 获取灯光AUTO档信息
 *
 *  \参数：	 pnLampAuto		[输出]	灯光AUTO档状态 
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetLampAutoInfo( uint8_t *pnLampAuto, uint64_t *pnTimeStamp );
 

/** ----------------------------------------------------------------
 *  \功能： 获取车门状态信息 
 *
 *  \参数：	 pnFL			[输出]	左前门状态
 *			 pnFR			[输出]	右前门状态
 * 			 pnRL			[输出]	左后门状态 
 * 			 pnRR			[输出]	右后门状态 
 * 			 pnTrunk		[输出]	行李箱状态
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetDoorInfo( uint8_t *pnFL, uint8_t *pnFR, 
					 uint8_t *pnRL, uint8_t *pnRR, 
					 uint8_t *pnTrunk, uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车轮行驶方向信息
 *
 *  \参数：	 pnFL			[输出]	左前轮行驶方向
 *			 pnFR			[输出]	右前轮行驶方向
 * 			 pnRL			[输出]	左后轮行驶方向 
 * 			 pnRR			[输出]	右后轮行驶方向
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetWheelDriveDirectionInfo( uint8_t *pnFL, uint8_t *pnFR, 
									uint8_t *pnRL, uint8_t *pnRR, 
									uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取ESP进入智能驾驶允许信息
 *
 *  \参数：	 pnAutoDrivePermission		[输出]	ESP进入允许
 *			 pnTimeStamp	            [输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */								
int32_t GetESPAutoDrivePermissionInfo( uint8_t *pnAutoDrivePermission, 
                                       uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取ESP当前驾驶模式信息
 *
 *  \参数：	 pnESPDriveModeFeedback		[输出]	ESP当前驾驶模式
 *			 pnTimeStamp				[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetESPDriveModeFeedbackInfo( uint8_t *pnESPDriveModeFeedback,
									 uint64_t *pnTimeStamp );	


/** ----------------------------------------------------------------
 *  \功能： 获取ESP滚动计数信息 
 *
 *  \参数：	 pnAlive		[输出]	ESP滚动计数变量 
 *			 pnTimeStamp	[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetESPAliveInfo( uint8_t *pnAlive, uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取ESP模块故障信息 
 *
 *  \参数：	 pnTractionControl		[输出]	牵引力控制故障 
 *			 pnBodyDynamic	        [输出]	车身动态故障
 *           pnTimeStamp            [输出]  时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetESPErrInfo( uint8_t *pnTractionControl, uint8_t *pnBodyDynamic,
                       uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取横摆角速度信息
 *
 *  \参数：	 pnYawRate			[输出]	横摆角速度 
 *			 pnYawRateOffset	[输出]	横摆角速度偏移量 
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetYawRateInfo( uint16_t *pnYawRate, uint16_t *pnYawRateOffset, 
						uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车辆纵向加速度
 *
 *  \参数：	 pnAX				[输出]	车辆纵向加速度 
 *			 pnAXOffset			[输出]	车辆纵向加速度偏移量 
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetAccelerationXInfo( uint16_t *pnAX, uint16_t *pnAXOffset, 
							  uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车辆横向加速度
 *
 *  \参数：	 pnAY				[输出]	车辆横向加速度 
 *			 pnAYOffset			[输出]	车辆横向加速度偏移量 
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetAccelerationYInfo( uint16_t *pnAY, uint16_t *pnAYOffset, 
							  uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取轮速脉冲信息
 *
 *  \参数：	 pnFL				[输出]	左前轮速脉冲变量
 *			 pnFR				[输出]	右前轮速脉冲变量
 * 			 pnRL				[输出]	左后轮速脉冲变量 
 * 			 pnRR				[输出]	右后轮速脉冲变量 
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetWheelPulseCounterInfo( uint16_t *pnFL, uint16_t *pnFR,
  								  uint16_t *pnRL, uint16_t *pnRR,  
								  uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取油门深度信息
 *
 *  \参数：	 pnAccelerateDeepness	[输出]	油门深度  
 *			 pnTimeStamp			[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetAccelerateDeepnessInfo( uint8_t *pnAccelerateDeepness,  
								   uint64_t *pnTimeStamp );

/** ----------------------------------------------------------------
 *  \功能： 获取虚拟油门深度信息
 *
 *  \参数：	 pnVirtualAccelerateDeepness	[输出]	虚拟油门深度  
 *			 pnTimeStamp			        [输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetVirtualAccelerateDeepnessInfo( uint8_t *pnVirtualAccelerateDeepness,  
								          uint64_t *pnTimeStamp );

/** ----------------------------------------------------------------
 *  \功能： 获取制动深度信息
 *
 *  \参数：	 pnBrakeDeepness	[输出]	制动深度  
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetBrakeDeepnessInfo( uint8_t *pnBrakeDeepness,  
							  uint64_t *pnTimeStamp );	


/** ----------------------------------------------------------------
 *  \功能： 获取制动踏板状态信息
 *
 *  \参数：	 pnBrakePedalSignal 	[输出]	制动踏板状态信息  
 *			 pnTimeStamp		    [输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetBrakePedalSignalInfo( uint8_t *pnBrakePedalSignal,  
								 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取档位信息
 *
 *  \参数：	 pnCarGear			[输出]	档位信息  
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */							   
int32_t GetCarGearInfo( uint8_t *pnCarGear,  
						uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取档位驾驶模式反馈
 *
 *  \参数：	 pnGearDriveModeFeedback	 [输出]	档位驾驶模式反馈  
 *			 pnTimeStamp		         [输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetGearDriveModeFeedbackInfo( uint8_t* pnGearDriveModeFeedback, 
	                                  uint64_t* pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取电子手刹状态信息
 *
 *  \参数：	 pnEleBrake			[输出]	电子手刹状态信息  
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetEleBrakeInfo( uint8_t *pnEleBrake,  
						 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取电子手刹驾驶模式反馈信息
 *
 *  \参数：	 pnEleBrakeDriveModeFeedback   [输出]	驾驶模式反馈  
 *			 pnTimeStamp		           [输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetEleBrakeDriveModeFeedbackInfo( uint8_t* pnEleBrakeDriveModeFeedback, 
	                                      uint64_t* pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取方向盘角度信息
 *
 *  \参数：	 pnAngular			[输出]	方向盘角度信息  
 *			 pnTimeStamp		[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetAngularInfo( uint16_t *pnAngular,  
  						uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取EPS当前驾驶模式信息 
 *
 *  \参数：	 pnEPSDriveModeFeedback		[输出]	EPS当前驾驶模式   
 *			 pnTimeStamp				[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetEPSDriveModeFeedbackInfo( uint8_t *pnEPSDriveModeFeedback,  
  									 uint64_t *pnTimeStamp );
 

/** ----------------------------------------------------------------
 *  \功能： 获取EPS进入智能驾驶允许信息 
 *
 *  \参数：	 pnDriveAutoPermission		[输出]	EPS进入智能驾驶允许标志    
 *			 pnTimeStamp				[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetDriveAutoPermissionInfo( uint8_t *pnDriveAutoPermission,  
  									uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取方向盘旋转速度信息（误差 ±2°）
 *
 *  \参数：	 pnRotationSpeed		[输出]	方向盘旋转速度   
 *			 pnTimeStamp			[输出]	时间计数
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetRotationSpeedInfo( uint8_t *pnRotationSpeed,  
  							  uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取多媒体触摸坐标信息（尚未实现） 
 *
 *  \参数：	 pnX				[输出]	触摸横坐标  
 *			 pnY				[输出]	触摸纵坐标    
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetMediumCoordinateInfo( uint16_t *pnX, uint16_t *pnY, 
  								 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取多媒体分辨率信息（尚未实现） 
 *
 *  \参数：	 pnW				[输出]	多媒体分辨率宽  
 *			 pnH				[输出]	多媒体分辨率高    
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetMediumResolutionInfo( uint16_t *pnW, uint16_t *pnH, 
  								 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取多媒体动作信息（尚未实现） 
 *
 *  \参数：	 pnMediumBehavior	[输出]	多媒体动作变量      
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetMediumBehaviorInfo( uint8_t *pnMediumBehavior, 
    						   uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取多媒体移动信息（尚未实现） 
 *
 *  \参数：	 pnX				[输出]	纵向行为变量   
 *			 pnY    			[输出]	横向行为变量  
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetMediumMovementInfo( uint16_t *pnX, uint16_t *pnY, 
    						   uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取障碍物距离信息（尚未实现）
 *
 *  \参数：	 pnFL				[输出]	前左角传感器障碍物距离  
 *			 pnFR				[输出]	前右角传感器障碍物距离
 *			 pnRL				[输出]	后左角传感器障碍物距离
 *			 pnRR				[输出]	后右角传感器障碍物距离
 *			 pnL				[输出]	左倒车传感器障碍物距离
 *			 pnR				[输出]	右倒车传感器障碍物距离
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetPASDistanceInfo( uint8_t *pnFL, uint8_t *pnFR, 
  							uint8_t *pnRL, uint8_t *pnRR, 
  							uint8_t *pnL, uint8_t *pnR, 
    						uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取驻车辅助系统电源开关状态信息（尚未实现）
 *
 *  \参数：	 pnPower			[输出]	电源开关状态 
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */
int32_t GetPASPowerInfo( uint8_t *pnPower, 
    					 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取电机扭矩信息
 *
 *  \参数：	 pnMotorTorque		[输出]	电机输出扭矩 
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */			
int32_t GetMotorTorqueInfo( uint16_t *pnMotorTorque, 
    						uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取电机控制器驾驶模式反馈信息
 *
 *  \参数：	 pnMCDriveModeFeedback		[输出]	电机控制器驾驶模式反馈 
 *			 pnTimeStamp				[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetMCDriveModeFeedbackInfo( uint8_t *pnMCDriveModeFeedback, 
    								uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取主缸压力值信息（尚未实现）
 *
 *  \参数：	 pnPressure					[输出]	MC压力值
 *			 pnPressureState			[输出]	MC压力值状态 
 *			 pnPressureOffset			[输出]	MC压力偏移值
 *			 pnPressureOffsetState		[输出]	MC压力偏移值状态
 *			 pnTimeStamp				[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetMCPressureInfo( uint16_t *pnPressure, uint8_t *pnPressureState, 
  						   uint16_t *pnPressureOffset, uint8_t *pnPressureOffsetState,
						   uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取智能驾驶开关信息
 *
 *  \参数：	 pnAutoDriveKey		[输出]	开关状态信息 
 *			 pnTimeStamp		[输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetAutoDriveKeyInfo( uint8_t *pnAutoDriveKey, 
	                         uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取智能驾驶紧急退出按键信息
 *
 *  \参数：	 pnEmergencyExitKey		[输出]	紧急退出按钮信息 
 *			 pnTimeStamp		    [输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetEmergencyExitKeyInfo( uint8_t *pnEmergencyExitKey, 
                                 uint64_t *pnTimeStamp );


/** ----------------------------------------------------------------
 *  \功能： 获取车辆VIN信息
 *
 *  \参数：	 pnVinInfo		    [输出]	车辆的VIN信息 
 *			 pnTimeStamp		    [输出]	时间计数 
 *
 *  \返回值： 获取成功后返回 ERR_BYD_AUTO_OK，否则返回相应的错误状态 
 */	
int32_t GetVinInfo( VinInfo *pnVinInfo, 
                    uint64_t *pnTimeStamp );
#ifdef __cplusplus
} 
#endif /* __cplusplus */
  
#endif
