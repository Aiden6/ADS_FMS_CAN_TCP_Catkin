#include "../include/byd_control_thread.h"


//ros
void* acu_ros_Thread( void *data )
{
    ros::NodeHandle nh_pub;	//节点句柄实例化
	ros::NodeHandle nh_sub;	//节点句柄实例化
	ros::Subscriber sub = nh_sub.subscribe("str_message", 1000, chatterCallback);	//向话题“str_message”订阅，一旦发布节点（node_a）在该话题上发布消息，本节点就会调用chatterCallbck函数。
 	ros::Publisher pub = nh_pub.advertise<std_msgs::String>("str_message", 1000);	//告诉系统要发布话题了，话题名为“str_message”，类型为std_msgs::String，缓冲队列为1000。
	ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz
	while(ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"Hello World";
		msg.data = ss.str();
		ROS_INFO("node_a is publishing %s", msg.data.c_str());
		pub.publish(msg);	//向话题“str_message”发布消息
		ros::spinOnce();	//不是必须，若程序中订阅话题则必须，否则回掉函数不起作用。
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
    }


}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("node_b is receiving [%s]", msg->data.c_str());
}
/*****************************************************************************
  函数名称:       void* can_rcvThread( void *data )
  说明:           CAN原始报文接收的线程，并传递给API封装一个接口供ECU获取
  调用:
  被调用:
  输入参数:       *data 			

  输出参数:       
  返回值:         void*
  被访问变量:
  被修改变量:
  其他:           用于查询接收到的CAN报文    
*****************************************************************************/


void* can_rcvThread( void *data )
{
    pthread_detach(pthread_self());
    CAN_DEV_INFO *pCAN_Info = (CAN_DEV_INFO *)data;

    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt;                       // current received
    while (gTestFlag) {
       // msleep(10);
        cnt = VCI_Receive(pCAN_Info->devType, pCAN_Info->devIndex,                         
                          pCAN_Info->channelNum, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        if (!cnt)
            continue;
        // Developer API test
        BydAuto_DeveloperAPI_ReceiveTest(can, cnt);
    }
    printf("can_rcvThread End.\n");
    pthread_exit(0);

    return 0;
}
/*****************************************************************************
  函数名称:       void* can_xmt( void )
  说明:           将要发的报文封装，并通过CAN报文发送的线程
  调用:
  被调用:
  输入参数:       void 			

  输出参数:       
  返回值:         void*
  被访问变量:
  被修改变量:
  其他:           用于定时对外CAN报文    
*****************************************************************************/
void can_xmt(union sigval v)
{
	VCI_CAN_OBJ canObj;
	int loop;
    static uint32_t Count = 0;
   
    // Developer API test
	canObj.DataLen = 0;
    canObj.ExternFlag = 1;
    canObj.RemoteFlag = 0;
    uint32_t MessageId[4] = {
					 //保证10ms：BYD_AUTO_IDRIVING_CMD_0优先发送
                     BYD_AUTO_IDRIVING_CMD_0,   //10ms
					 BYD_AUTO_IDRIVING_CMD_1,   //20ms
					 BYD_AUTO_IDRIVING_CMD_2,   //20ms
					 //BYD_AUTO_IDRIVING_CMD_3,
					 //BYD_AUTO_IDRIVING_CMD_4, 
					 BYD_AUTO_IDRIVING_CMD_5    //100ms
	};

    for (loop = 0; loop < 4; loop++) {
        if (BydAuto_DeveloperAPI_TransmitTest(MessageId[loop],
                                                  Count,
                                                  &canObj)) {
            /* Transmit the can message */
            VCI_Transmit(device.devType, device.devIndex, 
                         device.channelNum, &canObj, 1);
		}
	}
	if (10 == Count) {
		Count = 0;
	}
	Count++;

 //   return 0;
}

void tcp_xmt(union sigval v)
{
    static uint32_t Count = 0;
	if (10 == Count) {
        printf("1s1s1s1s11s1s1s1s1s1s1s1s11s1s");
		Count = 0;
	}
	Count++;

 //   return 0;

}




/*****************************************************************************
  函数名称:       void* cm_rcvThread( void *data)
  说明:           获取键盘命令输入
  调用:

  被调用:
  输入参数:       void 			

  输出参数:       
  返回值:         void*
  被访问变量:
  被修改变量:
  其他:           根据约定的规则对键盘输入进行判断
*****************************************************************************/
void* cm_rcvThread( void *data)
{
    pthread_detach(pthread_self());
    int8_t tmp_char, char_cnt = 0;
    int8_t *pChar = &gCharInput[1];
	for (;;) {  
        tmp_char = getchar();
        if ('\n' != tmp_char) {

            *pChar++ = tmp_char;
            char_cnt++;

            if (27 == gCharInput[1]) {  // ESC
                sem_post(&sem_event);
                break;          
            }
        }
        else {
            gCharInput[0] = char_cnt;
            char_cnt = 0;
            pChar = &gCharInput[1];
            sem_post(&sem_event);
        }
        
    }
    printf("cm_rcvThread End.\n");
    pthread_exit(0);
	return 0;
}

//自动驾驶操作
/*****************************************************************************
  函数名称:       void* acu_ctrlThread( void *data )
  说明:           ACU控制操作的线程
  调用:

  被调用:
  输入参数:       *data 			

  输出参数:       
  返回值:         void*
  被访问变量:
  被修改变量:
  其他:           通过API对各执行机构进行控制以及获取整车状态值
*****************************************************************************/

void* acu_ctrlThread( void *data )
{
    pthread_detach(pthread_self());
    while (gTestFlag) {
        DeveloperAPI_TestItemDisplay();
        sem_wait(&sem_event);
        // Developer API test
        BydAuto_DeveloperAPI_ControlTest(gCharInput); 
             
    }
    printf("acu_ctrlThread End.\n");
    pthread_exit(0);
    return 0;
}

void* acu_SetAcceleratedVelocity_Thread( void *data )
{
    pthread_detach(pthread_self()); 
    while (gTestFlag) {
        //DeveloperAPI_TestItemDisplay();
        sem_wait(&sem_event_SetAcceleratedVelocity);
        SetAcceleratedVelocity_BYD(AccVal);
        
    }
    printf("SetAcceleratedVelocity_Thread.\n");
    pthread_exit(0);
    return 0;
}

void* acu_SetAngular_Thread( void *data )
{
    pthread_detach(pthread_self());
    while (gTestFlag) {
        //DeveloperAPI_TestItemDisplay();
        sem_wait(&sem_event_SetAngular);
        SetAngular_BYD(nAngular);
        
    }
    printf("SetAngular_Thread End.\n");
    pthread_exit(0);

    return 0;
}
void* acu_CarGear_Thread( void *data )
{
    pthread_detach(pthread_self());
    while (gTestFlag) {
        //DeveloperAPI_TestItemDisplay();
        sem_wait(&sem_event_SetCarGear);
        SetCarGear_BYD(CarGear_val);
        
    }
    printf("CarGear_Thread End.\n");
    pthread_exit(0);

    return 0;
}
void* acu_SetEpbState_Thread( void *data )
{
    pthread_detach(pthread_self());
    while (gTestFlag) {
        //DeveloperAPI_TestItemDisplay();
        sem_wait(&sem_event_SetEpbState);
        SetEpbState_BYD(CarEpb_val );
        
    }
    printf("SetEpbState_Thread End.\n");
    pthread_exit(0);

    return 0;
}
void* acu_SetDrivingMode_Thread( void *data )
{
    pthread_detach(pthread_self());
    while (gTestFlag) {
        //DeveloperAPI_TestItemDisplay();
        sem_wait(&sem_event_SetDrivingMode);
        SetDrivingMode_BYD(DrivingMode_val);
        
    }
    printf("SetDrivingMode_Thread End.\n");
    pthread_exit(0);
    return 0;
}

void* acu_JudgeAutoDriveExit_thread( void *data )
{
    pthread_detach(pthread_self());
    while (gTestFlag) {
        sem_wait(&sem_event_GetCarInfo);
        //accDeepness_val  油门踏板深度
        //brakeDeepness_val  刹车踏板深度
        if((accDeepness_val > 5)||(brakeDeepness_val > 5)||(brakePedalStatus == BYD_AUTO_BRAKE_PEDAL_SIGNAL_PRESSED)){
        //退出模式
        DrivingMode_val = 0;
        sem_post(&sem_event_SetDrivingMode);

        printf("=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=The driver is controlling the car!-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=");
        }
        
    }
    printf("SetDrivingMode_Thread End.\n");
    pthread_exit(0);
    return 0;
}

void* acu_GetAutoDriveKey_Thread( void *data )
{
    pthread_detach(pthread_self());

    uint64_t timeStamp = 0;
    //检测键是否被按下，自动驾驶模式是否开启
    while (gTestFlag) {
        sem_wait(&sem_event_GetCarInfo);
        switch (autoDriveKey) {
        case BYD_AUTO_MEDIUM_KEY_INVALID: {
                //printf("auto-drive key is not pressed.\n");
            }
            break;
        case BYD_AUTO_MEDIUM_KEY_DRIVE_AUTO_SWITCH: {
            if(DrivingMode_val != 1){
                //进入模式
                DrivingMode_val = 1;
                sem_post(&sem_event_SetDrivingMode);
                //挂D档
                CarGear_val = 4;
				sem_post(&sem_event_SetCarGear);
                //松手刹
                CarEpb_val = 2;
				sem_post(&sem_event_SetEpbState);
                //方向盘回正
                nAngular = 0;
                sem_post(&sem_event_SetAngular);
            }
            else{
                //挂P档
                CarGear_val = 1;
				sem_post(&sem_event_SetCarGear);
                //拉手刹
                CarEpb_val = 1;
				sem_post(&sem_event_SetEpbState);
                //方向盘回正
                nAngular = 0;
		        sem_post(&sem_event_SetAngular);
                //发送减速度 停车
                AccVal = -0.5;
                sem_post(&sem_event_SetAcceleratedVelocity);
                //退出模式
                DrivingMode_val = 0;
                sem_post(&sem_event_SetDrivingMode);
            }
                printf("auto-drive key is pressed.\n");
            }
            break;
        default:
            break;
        }
    }

    printf("GetAutoDriveKey_Thread End.\n");
    pthread_exit(0);
    return 0;
}


/*****************************************************************************
  函数名称:       void Timer10ms( timer_t *timer )
  说明:           
  调用:

  被调用:
  输入参数:       *timer 			

  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           
*****************************************************************************/
void Timer10ms( timer_t *timer )
{
	int ret;	
	struct sigevent evp10ms;
	struct itimerspec ts10ms;

	memset(&evp10ms,0,sizeof(evp10ms));
	evp10ms.sigev_value.sival_ptr = timer;
	evp10ms.sigev_notify = SIGEV_THREAD;
	evp10ms.sigev_notify_function = can_xmt;//绑定CAN报文发送

    ret = timer_create(CLOCK_REALTIME,&evp10ms,timer);
	if(ret)
		printf("the timer10ms create error!\n");
	ts10ms.it_interval.tv_sec = 0;
	ts10ms.it_interval.tv_nsec = TIME_PERIOD;
	ts10ms.it_value.tv_sec = 0;
	ts10ms.it_value.tv_nsec = TIME_PERIOD;
	ret = timer_settime(*timer,TIMER_ABSTIME,&ts10ms,NULL);
	if(ret)
		printf("timer10s time_setting error!\n");
	return;
}

void Timer1s( timer_t *timer )
{
	int ret;	
	struct sigevent evp1s;
	struct itimerspec ts1s;

	memset(&evp1s,0,sizeof(evp1s));
	evp1s.sigev_value.sival_ptr = timer;
	evp1s.sigev_notify = SIGEV_THREAD;
	evp1s.sigev_notify_function = tcp_xmt;//绑定tcp报文发送

    ret = timer_create(CLOCK_REALTIME,&evp1s,timer);
	if(ret)
		printf("the timer1s create error!\n");
	ts1s.it_interval.tv_sec = 0;
	ts1s.it_interval.tv_nsec = tcp_TIME_PERIOD;
	ts1s.it_value.tv_sec = 0;
	ts1s.it_value.tv_nsec = tcp_TIME_PERIOD;
	ret = timer_settime(*timer,TIMER_ABSTIME,&ts1s,NULL);
	if(ret)
		printf("timer1s time_setting error!\n");
	return;

}
void TimerGetCarInfo1s(timer_t *timer)
{
    int ret;	
	struct sigevent evp1s;
	struct itimerspec ts1s;

	memset(&evp1s,0,sizeof(evp1s));
	evp1s.sigev_value.sival_ptr = timer;
	evp1s.sigev_notify = SIGEV_THREAD;
	evp1s.sigev_notify_function = GetCarInfo_BYD;//绑定tcp报文发送

    ret = timer_create(CLOCK_REALTIME,&evp1s,timer);
	if(ret)
		printf("the timer1s create error!\n");
	ts1s.it_interval.tv_sec = 0;
	ts1s.it_interval.tv_nsec = GetCarInfo_TIME_PERIOD;
	ts1s.it_value.tv_sec = 0;
	ts1s.it_value.tv_nsec = GetCarInfo_TIME_PERIOD;
	ret = timer_settime(*timer,TIMER_ABSTIME,&ts1s,NULL);
	if(ret)
		printf("timer1s time_setting error!\n");
	return;

}

/*****************************************************************************
  函数名称:       int32_t BydAuto_Can_Connect( void )
  说明:           CAN卡配置
  调用:

  被调用:
  输入参数:       void 			

  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           
*****************************************************************************/
int32_t BydAuto_Can_Connect( void )
{
    int32_t  devType = VCI_USBCAN2;        //CAN卡类型
    int32_t  devIndex = 0;
    int32_t  channelNum = 0;               //CAN通道

    device.devType = devType;              //CAN卡配置
    device.devIndex = devIndex;
    device.channelNum = channelNum;

    VCI_INIT_CONFIG config;                //CAN初始化结构体
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = 0xC0;
    config.Timing1 = 0x3A;  // 500 kbps

    //打开CAN卡
    if (!VCI_OpenDevice(devType, devIndex, 0)) {
        printf("VCI_OpenDevice failed\n");
        return 0;
    }
    printf("VCI_OpenDevice succeeded\n");
    //初始化CAN卡
    if (!VCI_InitCAN(devType, devIndex, channelNum, &config)) {
        printf("VCI_InitCAN failed\n");
        return 0;
    }
    //开始CAN接收
    if (!VCI_StartCAN(devType, devIndex, channelNum)) {
        printf("VCI_InitCAN failed\n");
        return 0;
    }
	return 1;
}
