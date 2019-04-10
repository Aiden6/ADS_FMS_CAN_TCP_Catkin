
#ifndef _TCP_CALLBAKCk_H_
#define _TCP_CALLBAKCk_H_

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/time.h>
#include <iostream>
#include <boost/asio.hpp>

#include "./message_encrypt.h"
#include "./ads_message.h"
#include "./async_client.h"

using namespace std;

// tcp session object
extern Async_Client *client;

// login result
extern char sessionKey[17];
extern char sessionIv[17];

extern bool isRoutingRecv;

extern sem_t sem_event_SetAcceleratedVelocity;
extern sem_t sem_event_SetAngular;
extern sem_t sem_event_SetCarGear;
extern sem_t sem_event_SetEpbState;
extern sem_t sem_event_SetDrivingMode;


//extern Global variable
extern double nAngular;
extern double AccVal;
extern uint8_t CarGear_val;
extern uint8_t CarEpb_val;
extern uint8_t DrivingMode_val;

extern string 		ipcID;     // IPCID
extern uint8_t 	securityType; 		// AES     //0:no encrypt;1:AES


//0x0101
//IPC login to FMS Server parameters
extern uint16_t 	applicationID_0x0101; 	// login   //IPC log in Application
//Dispatcher
extern uint16_t	messageID_0x0101;
//Application Data
extern string		messageData_0x0101;

extern uint16_t	messageDataLength_0x0101;
extern bool		isLogin_0x0101;


//0x0108
extern uint16_t 	applicationID_0x0102; 	
//Dispatcher
extern uint16_t	messageID_0x0102;
//Application Data
extern string		messageData_0x0102;

extern uint16_t	messageDataLength_0x0102;
extern bool		isLogin_0x0102;


//0x0108
extern uint16_t 	applicationID_0x0108; 	
//Dispatcher
extern uint16_t	messageID_0x0108;
//Application Data
extern string		messageData_0x0108;

extern uint16_t	messageDataLength_0x0108;
extern bool		isLogin_0x0108;


//0x0109
extern uint16_t 	applicationID_0x0109; 	
//Dispatcher
extern uint16_t	messageID_0x0109;
//Application Data
extern string		messageData_0x0109;

extern uint16_t	messageDataLength_0x0109;
extern bool		isLogin_0x0109;


//0x010A
extern uint16_t 	applicationID_0x010A; 	
//Dispatcher
extern uint16_t	messageID_0x010A;
//Application Data
extern string		messageData_0x010A;

extern uint16_t	messageDataLength_0x010A;
extern bool		isLogin_0x010A;

//0x010B
extern uint16_t 	applicationID_0x010B; 	
//Dispatcher
extern uint16_t	messageID_0x010B;
//Application Data
extern string		messageData_0x010B;

extern uint16_t	messageDataLength_0x010B;
extern bool		isLogin_0x010B;

void CallBackRead(const char* addr, int port, const char* data, const int len);

#endif