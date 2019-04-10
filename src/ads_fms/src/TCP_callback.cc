#include "../include/TCP_callback.h"



// tcp/ip call back

void CallBackRead(const char* addr, int port, const char* data, const int len) {
	char * tmp_ = (char*)malloc(len - 2);
	// remove header and tail
	for (int i = 1; i < len - 1; i++) {
		*(tmp_+ i - 1) = data[i];
	}
	// transform
	int lengthMinus = 0;
	// everytime check two bytes, so i < (len - 2) - 1
    for (int i = 0; i < len - 2 - 1; i++) {
        if (tmp_[i] == 0x5e && tmp_[i + 1] == 0x5d) {
            lengthMinus++;
        }
		else if (tmp_[i] == 0x5e && tmp_[i + 1] == 0x7d) {
			lengthMinus++;
		}
    }

	int messageTransformLength_ = len - 2 - lengthMinus;

    char *messageTransformed_ = (char*)malloc(messageTransformLength_);
    memset(messageTransformed_, 0, messageTransformLength_);

	int start = 0;
	for (int i = 0; i < len - 2 - 1; i++) {
		if (tmp_[i] == 0x5e && tmp_[i + 1] == 0x5d) {
			*(messageTransformed_ + start) = 0x5e;
			 start ++;
			 i ++;
			//  printf("i : %d, start: %d \n", i, start);
		}
		else if (tmp_[i] == 0x5e && tmp_[i + 1] == 0x7d) {
			*(messageTransformed_ + start) = 0x7e;
			 start ++;
			 i ++;
			//  printf("i : %d, start: %d \n", i, start);
		}
		else {
			if (i == (len - 2 - 2)) {
				*(messageTransformed_ + start) = tmp_[i];
				*(messageTransformed_ + start + 1) = tmp_[i + 1];
				// printf("i : %d, start: %d \n", i, start);
			} // at (tail - 1)
			else {
				*(messageTransformed_ + start) = tmp_[i];
				start ++;
				// printf("i : %d, start: %d \n", i, start);
			}
		}
	}

	// application ID
	uint16_t appID = (uint16_t)((messageTransformed_[4] << 8) + messageTransformed_[5]);

	// Message Body
	// length
	int messageEncryptedBodyLength_ = messageTransformLength_ - 2 - 1 - 1 - 2 - 17 - 1;
	//Message Body
	char *messageEncryptedBody_ = (char*)malloc(messageEncryptedBodyLength_);
	memset(messageEncryptedBody_, 0, messageEncryptedBodyLength_);

	for (int i = 0; i < messageEncryptedBodyLength_; i++) {
		*(messageEncryptedBody_ + i) = messageTransformed_[i + 23];
	}

    message_encrypt msgEncrypt;
	// IPC log in 
	if (appID == 0x0101) {
		char *messageBodyDecrypted_0x0101;	
		msgEncrypt.aes_cbc_pcsk5_decrypt(messageEncryptedBody_, 
										 messageEncryptedBodyLength_, 
										 messageBodyDecrypted_0x0101, 
										 false);    
										 
		char *KeyIv_ = (char*)malloc(32);
		memset(KeyIv_, 0, 32);

		for(int i = 0; i < 32; i++) {
				*(KeyIv_ + i) = messageBodyDecrypted_0x0101[i + 10 + 2];
		}

		for (int i = 0; i < 16; i++) {
			sessionKey[i] = KeyIv_[i];
			sessionIv[i]  = KeyIv_[i + 16];
		}

		printf("Reading session key and iv from server:\n");
		printf("sessionKey: %s \n",sessionKey);
		printf("sessionIv: %s \n\n",sessionIv);


		free(KeyIv_);
		free(messageBodyDecrypted_0x0101);
	}
	//path planning information 
	else if(appID == 0x0102) {
		char *messageBodyDecrypted_0x0102;
		msgEncrypt.transmitSessionKeyIv(sessionKey, sessionIv);
		msgEncrypt.aes_cbc_pcsk5_decrypt(messageEncryptedBody_, 
										 messageEncryptedBodyLength_, 
										 messageBodyDecrypted_0x0102, 
										 true);    

		
		uint8_t *Start_Point_Long = (uint8_t*)malloc(4);
		uint8_t *Start_Point_Lat = (uint8_t *)malloc(4);
		uint8_t *End_Point_Long = (uint8_t *)malloc(4);
		uint8_t *End_Point_Lat = (uint8_t *)malloc(4);

		memset(Start_Point_Long,0,4);
		memset(Start_Point_Lat,0,4);
		memset(End_Point_Long,0,4);
		memset(End_Point_Lat,0,4);


		for(char i = 0; i<4; i++){
            // 10 11 12 13
			//  3  2  1  0
		    Start_Point_Long[i] = messageBodyDecrypted_0x0102[13-i];
			Start_Point_Lat[i] = messageBodyDecrypted_0x0102[17-i];
			End_Point_Long[i] = messageBodyDecrypted_0x0102[21-i];
			End_Point_Lat[i] = messageBodyDecrypted_0x0102[25-i];
		}

	    uint32_t *Start_Point_Longtitude = (uint32_t*)malloc(1);
		uint32_t *Start_Point_Latitude = (uint32_t *)malloc(1);
		uint32_t *End_Point_Longtitude = (uint32_t *)malloc(1);
		uint32_t *End_Point_Latitude = (uint32_t *)malloc(1);

		memset(Start_Point_Longtitude,0,1);
		memset(Start_Point_Latitude,0,1);
		memset(End_Point_Longtitude,0,1);
		memset(End_Point_Latitude,0,1);

		memcpy(Start_Point_Longtitude, Start_Point_Long, 4); 
		memcpy(Start_Point_Latitude, Start_Point_Lat, 4); 
		memcpy(End_Point_Longtitude, End_Point_Long, 4); 
		memcpy(End_Point_Latitude, End_Point_Lat, 4); 

		printf("Start_Point_Longtitude is: %u\n",*Start_Point_Longtitude);
		printf("Start_Point_Latitude is: %u\n",*Start_Point_Latitude);
		printf("End_Point_Longtitude is: %u\n",*End_Point_Longtitude);
		printf("End_Point_Latitude is: %u\n",*End_Point_Latitude);


		free(Start_Point_Long);
		free(Start_Point_Lat);
		free(End_Point_Long);
		free(End_Point_Lat);


		isRoutingRecv = true;

		//client response
		char messageData_0x0102_Result = 1;
		char messageData_0x0102_ErrorCode = 0;

		messageData_0x0102[0] = messageData_0x0102_Result;
		messageData_0x0102[1] = messageData_0x0102_ErrorCode;

		cout << "messageData_0x0108 is:"<< messageData_0x0102 << endl;

		ads_message adsMsg_0x0102(securityType,applicationID_0x0102,ipcID,messageID_0x0102,messageData_0x0102,messageDataLength_0x0102,
					   isLogin_0x0102);
		boost_tcp_sync_send(client, adsMsg_0x0102.messageComplete_, adsMsg_0x0102.messageLength_); 
	
	}
	// vehicle start stop
	else if(appID == 0x0108){
		char *messageBodyDecrypted_0x0108;
		msgEncrypt.transmitSessionKeyIv(sessionKey, sessionIv);
		msgEncrypt.aes_cbc_pcsk5_decrypt(messageEncryptedBody_, 
										 messageEncryptedBodyLength_, 
										 messageBodyDecrypted_0x0108, 
										 true);    
		uint8_t *startOrStop = (uint8_t *)malloc(1);
		memset(startOrStop, 0, 1);

		*startOrStop = messageBodyDecrypted_0x0108[10];

		if(*startOrStop == 1){
			printf("The VCM start or stop commad is: %d\n", *startOrStop);
			printf("The VCM is cmmanding vehicle to power up ...\n");
		}
		else if(*startOrStop == 0){
			printf("The VCM start or stop commad is: %d\n", *startOrStop);
			printf("The VCM is cmmanding vehicle to power down...\n");
		}

		//sem_post(&sem_event);

		//client response
		char messageData_0x0108_Result = 1;
		char messageData_0x0108_ErrorCode = 0;

		messageData_0x0108[0] = messageData_0x0108_Result;
		messageData_0x0108[1] = messageData_0x0108_ErrorCode;

		cout << "messageData_0x0108 is:"<< messageData_0x0108 << endl;

		ads_message adsMsg_0x0108(securityType,applicationID_0x0108,ipcID,messageID_0x0108,messageData_0x0108,messageDataLength_0x0108,
					   isLogin_0x0108);
		boost_tcp_sync_send(client, adsMsg_0x0108.messageComplete_, adsMsg_0x0108.messageLength_); 

	}
	// vehicle steering
	else if(appID == 0x0109){
		char *messageBodyDecrypted_0x0109;
		msgEncrypt.transmitSessionKeyIv(sessionKey, sessionIv);
		msgEncrypt.aes_cbc_pcsk5_decrypt(messageEncryptedBody_, 
										 messageEncryptedBodyLength_, 
										 messageBodyDecrypted_0x0109, 
										 true);    

		uint8_t *directionControl = (uint8_t *)malloc(1);
		uint8_t *angleValueTemp = (uint8_t *)malloc(2);
		memset(directionControl,0,1);
		memset(angleValueTemp,0,2);

		*directionControl = messageBodyDecrypted_0x0109[10];

		for(char i = 0; i < 2; i++){
			angleValueTemp[i] = messageBodyDecrypted_0x0109[12-i];
		}

		uint16_t *angleValue = (uint16_t *)malloc(1);
		memset(angleValue, 0, 1);

		memcpy(angleValue, angleValueTemp, 2);
		free(angleValueTemp);

		//右转
		if(*directionControl == 0){
			printf("The VCM dircetion control command is: %d\n", *directionControl);
			printf("RIGHT Turning... \n>>>>>>>>>>>>>>>>>\n>>>>>>>>>>>>>>>>>\n>>>>>>>>>>>>>>>>>\n");
			nAngular =*angleValue;
		}
		//左转
		else if(*directionControl == 1){
			printf("The VCM dircetion control command is: %d\n", *directionControl);
			printf("LEFT Turning...  \n<<<<<<<<<<<<<<<<<\n<<<<<<<<<<<<<<<<<\n<<<<<<<<<<<<<<<<<\n");
			nAngular = (-1)*(*angleValue);
		}
		printf("The VCM dircetion control angle value is: %d\n", *angleValue);

		sem_post(&sem_event_SetAngular);
		
		//client response
		char messageData_0x0109_Result = 1;
		char messageData_0x0109_ErrorCode = 0;

		messageData_0x0109[0] = messageData_0x0109_Result;
		messageData_0x0109[1] = messageData_0x0109_ErrorCode;

		cout << "messageData_0x0109 is:"<< messageData_0x0109 << endl;
		ads_message adsMsg_0x0109(securityType,applicationID_0x0109,ipcID,messageID_0x0109,messageData_0x0109,messageDataLength_0x0109,
					   isLogin_0x0109);
		boost_tcp_sync_send(client, adsMsg_0x0109.messageComplete_, adsMsg_0x0109.messageLength_); 

	}
	//vehicle acceleration
	else if(appID == 0x010A){
		char *messageBodyDecrypted_0x010A;
		msgEncrypt.transmitSessionKeyIv(sessionKey, sessionIv);
		msgEncrypt.aes_cbc_pcsk5_decrypt(messageEncryptedBody_, 
										 messageEncryptedBodyLength_, 
										 messageBodyDecrypted_0x010A, 
										 true);    
		uint8_t *accelerationCommand = (uint8_t *)malloc(1);
		uint8_t *accelerationCommandValue = (uint8_t *)malloc(1);
		memset(accelerationCommand,0,1);
		memset(accelerationCommandValue,0,1);		

		*accelerationCommand = messageBodyDecrypted_0x010A[10];
		*accelerationCommandValue = messageBodyDecrypted_0x010A[11];
		//减速
		if(*accelerationCommand == 0){
			printf("The VCM acceleration command is: %d\n", *accelerationCommand);
			printf("The VCM acceleration command is: BRAKE\n");
			printf("----------------------\n----------------------\n----------------------\n");
			AccVal = (-1)*((double)(*accelerationCommandValue))/20;
		}
		//加速
		else if(*accelerationCommand == 1){
			printf("The VCM acceleration command is: %d\n", *accelerationCommand);
			printf("The VCM acceleration command is: ACCELERATION\n");
			printf("++++++++++++++++++++++\n++++++++++++++++++++++\n++++++++++++++++++++++\n");
			AccVal = ((double)(*accelerationCommandValue))/20;
		}
		printf("The VCM acceleration command value is: %d\n\n\n", *accelerationCommandValue);

		sem_post(&sem_event_SetAcceleratedVelocity);

		//client response
		char messageData_0x010A_Result = 1;
		char messageData_0x010A_ErrorCode = 0;

		messageData_0x010A[0] = messageData_0x010A_Result;
		messageData_0x010A[1] = messageData_0x010A_ErrorCode;

		cout << "messageData_0x010A is:"<< messageData_0x010A << endl;
		ads_message adsMsg_0x010A(securityType,applicationID_0x010A,ipcID,messageID_0x010A,messageData_0x010A,messageDataLength_0x010A,
					   isLogin_0x010A);
		boost_tcp_sync_send(client, adsMsg_0x010A.messageComplete_, adsMsg_0x010A.messageLength_); 

	}
	// vehicle others control:EPB PRDN
	else if(appID == 0x010B){
		char *messageBodyDecrypted_0x010B;
		msgEncrypt.transmitSessionKeyIv(sessionKey, sessionIv);
		msgEncrypt.aes_cbc_pcsk5_decrypt(messageEncryptedBody_, 
										 messageEncryptedBodyLength_, 
										 messageBodyDecrypted_0x010B, 
										 true); 
		uint8_t VehicleRemoteSetType = 0;

		//判断设置类型
		switch (messageBodyDecrypted_0x010B[10])
		{
			//驾驶模式
			case 1:
				VehicleRemoteSetType = 1;
				printf("VehicleRemoteSetType is %d\n", VehicleRemoteSetType);
				/* code */
				if(messageBodyDecrypted_0x010B[11] == 0){
					DrivingMode_val = 0;
				}
				else if(messageBodyDecrypted_0x010B[11] == 1){
					DrivingMode_val = 1;
				}
				else if(messageBodyDecrypted_0x010B[11] == 2){
					DrivingMode_val = 2;
				}
				else{
					break;
				}
				sem_post(&sem_event_SetDrivingMode);
				break;
			//EPB
			case 2:
				VehicleRemoteSetType = 2;
				printf("VehicleRemoteSetType is %d\n", VehicleRemoteSetType);
				if(messageBodyDecrypted_0x010B[11] == 0){
					CarEpb_val = 0;
				}
				else if(messageBodyDecrypted_0x010B[11] == 1){
					CarEpb_val = 1;
				}
				else if(messageBodyDecrypted_0x010B[11] == 2){
					CarEpb_val = 2;
				}
				else{
					break;
				}
				sem_post(&sem_event_SetEpbState);
				break;
			//Gear
			case 3:
				VehicleRemoteSetType = 3;

				printf("VehicleRemoteSetType is %d\n", VehicleRemoteSetType);
				if(messageBodyDecrypted_0x010B[11] == 0){
					CarGear_val = 0;
				}
				else if(messageBodyDecrypted_0x010B[11] == 1){
					CarGear_val = 1;
				}
				else if(messageBodyDecrypted_0x010B[11] == 2){
					CarGear_val = 2;
				}
				else if(messageBodyDecrypted_0x010B[11] == 3){
					CarGear_val = 3;
				}
				else if(messageBodyDecrypted_0x010B[11] == 4){
					CarGear_val = 4;					
				}
				else{
					break;
				}
				sem_post(&sem_event_SetCarGear);			
				break;
			default:
				VehicleRemoteSetType = 0;
				break;
		}
		printf("CarGear_val is: %d\n",CarGear_val);
		//sem_post(&sem_event);
		//client response
		messageData_0x010B[0] = VehicleRemoteSetType;
		if(VehicleRemoteSetType == 1){
			messageData_0x010B[1] = 0;

		}
		else if(VehicleRemoteSetType == 2){
			messageData_0x010B[1] = 0;
		}
		else if(VehicleRemoteSetType == 3){
			messageData_0x010B[1] = 0;
		}

		char messageData_0x010B_Result = 1;
		char messageData_0x010B_ErrorCode = 0;

		messageData_0x010B[2] = messageData_0x010B_Result;
		messageData_0x010B[3] = messageData_0x010B_ErrorCode;
		cout << "messageData_0x010B is:"<< messageData_0x010B << endl;
		ads_message adsMsg_0x010B(securityType,applicationID_0x010B,ipcID,messageID_0x010B,messageData_0x010B,messageDataLength_0x010B,
					   isLogin_0x010B);
		boost_tcp_sync_send(client, adsMsg_0x010B.messageComplete_, adsMsg_0x010B.messageLength_); 

	}
	free(tmp_);
	free(messageEncryptedBody_);
	free(messageTransformed_);
}




