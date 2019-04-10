#include "../include/ads_message.h"

void ads_message::updateMessage() {
    // 
    setEventCreationTime();
    // calculate each length of parts
    calMessageBodyLength();
    calMessageInfoLength();
    // 1: build message body and encrypt
    // free memory before update new messages
    free(messageBody_);
    buildMessageBody();
    free(messageBodyEncrypted_);
    messageBodyEncrypt();
    // when complete encrypt, calculate the original message length
    // and total message length
    calMessageOriginalLength();
    // 2: build message info
    //    build message original (+check byte)
    //    transform
    free(messageInfo_);
    buildMessageInfo();
    messageCRCComputeandSet();
    free(messageOriginal_);
    buildMessageOriginal();
    free(messageTransformed_);
    messageTransform();
    // 3: build message complete
    calMessageLength();
    free(messageComplete_);
    buildMessageComplete();
}

void ads_message::buildMessageBody() {
    // printf("now building message Body... \n");

    messageBody_ = (char*)malloc(messageBodyLength_);
    memset(messageBody_, 0, messageBodyLength_);
   
    int start = 0;

    combineUintX(messageBody_, &eventCreationTime_, start, sizeof(eventCreationTime_));
    start+=sizeof(eventCreationTime_);
    // printDebugInfo(messageBody_, messageBodyLength_);

    combineUintX(messageBody_, &dispatcherResult_, start, sizeof(dispatcherResult_));
    start+=sizeof(dispatcherResult_);
    // printDebugInfo(messageBody_, messageBodyLength_);

    combineUintX(messageBody_, &messageID_, start, sizeof(messageID_));
    start+=sizeof(messageID_);
    // printDebugInfo(messageBody_, messageBodyLength_);

    combineUintX(messageBody_, &messageDataLength_, start, sizeof(messageDataLength_));
    start+=sizeof(messageDataLength_);
    // printDebugInfo(messageBody_, messageBodyLength_);

    combineChar(messageBody_, messageData_, start, messageDataLength_);
    
    // printDebugInfo(messageBody_, messageBodyLength_, "messageBody: ");

    #ifdef DEBUG
        printf("the message body in string : ");
        for (int i = 0; i < messageBodyLength_; i++) {
            printf("%c", messageBody_[i]);
        }
        printf("\n");
    #endif
}

void ads_message::buildMessageInfo() {
    // printf("now building message Info... \n");

    messageInfo_ = (char*)malloc(messageInfoLength_);
    memset(messageInfo_, 0, messageInfoLength_);

    int start = 0;

    combineUintX(messageInfo_, &messageLength_, start, sizeof(messageLength_));
    start+=sizeof(messageLength_);
    // printDebugInfo(messageInfo_, messageInfoLength_, "messageInfo: ");

    combineUintX(messageInfo_, &securityType_, start, sizeof(securityType_));
    start+=sizeof(securityType_);
    // printDebugInfo(messageInfo_, messageInfoLength_, "messageInfo: ");

    combineUintX(messageInfo_, &messageCount_, start, sizeof(messageCount_));
    start+=sizeof(messageCount_);
    // printDebugInfo(messageInfo_, messageInfoLength_, "messageInfo: ");

    combineUintX(messageInfo_, &applicationID_, start, sizeof(applicationID_));
    start+=sizeof(applicationID_);
    // printDebugInfo(messageInfo_, messageInfoLength_, "messageInfo: ");

    combineChar(messageInfo_, ipcID_, start, IPC_ID_LENGTH);
    // printDebugInfo(messageInfo_, messageInfoLength_, "messageInfo: ");

}

void ads_message::messageBodyEncrypt() {
    // printf("now encrypting message Body... \n");

    message_encrypt msgEncrypt;
    messageBodyEncryptedLength_ = msgEncrypt.getEncryptLength(messageBodyLength_);

	msgEncrypt.transmitSessionKeyIv(sessionKey, sessionIv);

    messageBodyEncrypted_ = (char*)malloc(messageBodyEncryptedLength_);
    memset(messageBodyEncrypted_, 0, messageBodyEncryptedLength_);

    if (isLogin_) {
        msgEncrypt.aes_cbc_pcsk5_encrypt(messageBody_, 
                                         messageBodyLength_, 
                                         messageBodyEncrypted_, 
                                         false);   
    } // login
    else {
        msgEncrypt.aes_cbc_pcsk5_encrypt(messageBody_, 
                                         messageBodyLength_, 
                                         messageBodyEncrypted_, 
                                         true);
    } // session

    // printDebugInfo(messageBodyEncrypted_, messageBodyEncryptedLength_, "messageBodyEncrypted: ");
}

void ads_message::messageCRCComputeandSet() {
    // printf("now calculating message CRC... \n");

    // message info and body BYTE bit XOR
    char start = 0x00;
    int i = 0; 
    for (i = 0; i < messageInfoLength_; i++) {
        start = start ^ messageInfo_[i];
    }

    for (i = 0; i < messageBodyEncryptedLength_; i++) {
        start = start ^ messageBodyEncrypted_[i];
    }

    messageCheck_ = start;
}

void ads_message::buildMessageOriginal() {
    // printf("now building message Original... \n");

    messageOriginal_ = (char*)malloc(messageOriginalLength_);
    memset(messageOriginal_, 0, messageOriginalLength_);

    // exclude header and tail
    int start = 0;

    combineChar(messageOriginal_, messageInfo_, start, messageInfoLength_);
    start += messageInfoLength_;

    combineChar(messageOriginal_, messageBodyEncrypted_, start, messageBodyEncryptedLength_);
    start += messageBodyEncryptedLength_;

    combineUintX(messageOriginal_, &messageCheck_, start, sizeof(messageCheck_));
    
    #ifdef DEBUG
        printf("the message original : ");
        for (int i = 0; i < messageOriginalLength_; i++) {
            printf("%.2X ", 0xff & messageOriginal_[i]);
        }
        printf("\n");
    #endif
    
}

void ads_message::messageTransform() {
    // printf("now transforming Original, Info + Body + CRC... \n");

    int lengthAdd = 0;
    for (int i = 0; i < messageOriginalLength_; i++) {
        if (messageOriginal_[i] == 0x5e || messageOriginal_[i] == 0x7e) {
            lengthAdd++;
        }
    }
    // printf("message transform adds message length : %d \n", lengthAdd);
    messageTransformLength_ = messageOriginalLength_ + lengthAdd;
    // printf("message messageTransformLength_ : %d \n", messageTransformLength_);

    messageTransformed_ = (char*)malloc(messageTransformLength_);
    memset(messageTransformed_, 0, messageTransformLength_);

    int start = 0;
    char trans5E1 = MESSAGE_TRANSFORM_5E_1;
    char trans5E2 = MESSAGE_TRANSFORM_5E_2;
    char trans7E1 = MESSAGE_TRANSFORM_7E_1;
    char trans7E2 = MESSAGE_TRANSFORM_7E_2;

    for (int i = 0; i < messageOriginalLength_; i++) {
        // printf("the %d char in original message is : %.2X \n", i, 0xff & messageOriginal_[i]);
        if (messageOriginal_[i] == 0x5e) {
            combineChar(messageTransformed_, &trans5E1, start++, 1);
            combineChar(messageTransformed_, &trans5E2, start++, 1);
        }
        else if (messageOriginal_[i] == 0x7e) {
            combineChar(messageTransformed_, &trans7E1, start++, 1);
            combineChar(messageTransformed_, &trans7E2, start++, 1);
        }
        else {
            combineChar(messageTransformed_, &messageOriginal_[i], start++, 1); 
        }
    }

    #ifdef DEBUG
        printf("the message transfer :");
        for (int i = 0; i < messageTransformLength_; i++) {
            printf("%.2X ", 0xff & messageTransformed_[i]);
        }
        printf("\n");
    #endif
}

void ads_message::buildMessageComplete() {
    // printf("now building complete message... \n");

    messageComplete_ = (char*)malloc(messageLength_);
    memset(messageComplete_, 0, messageLength_);

    int start = 0;
    combineUintX(messageComplete_, &messageHeader_, start, sizeof(messageHeader_));
    start += sizeof(messageHeader_);

    combineChar(messageComplete_, messageTransformed_, start, messageTransformLength_);
    start += messageTransformLength_;

    combineUintX(messageComplete_, &messageTail_, start, sizeof(messageTail_));

#ifdef DEBUG
    printf("the message complete : ");
    for (int i = 0; i < messageLength_; i++) {
        printf("%.2X ", 0xff & messageComplete_[i]);
    }
    printf("\n");
 #endif
}



// char processing
void ads_message::convertX2Char(void* src, char* &dest, uint8_t len) {
    // Big-end
    for (int i = 0; i < len; i++) {
        switch (len) {
            case 1: *(dest + i) = *(uint8_t*)src >> ((len - i - 1) * 8); break;
            case 2: *(dest + i) = *(uint16_t*)src >> ((len - i - 1) * 8); break;
            case 4: *(dest + i) = *(uint32_t*)src >> ((len - i - 1) * 8); break;
        }
    }
}

void ads_message::combineUintX(char* &src, void* dest, uint8_t start, uint8_t len) {
    char * tmpChar = NULL;
    tmpChar = (char*)malloc(len);
    convertX2Char(dest, tmpChar, len);

// #ifdef DEBUG
//     printf("the tmpChar len: %d \n", len); //void* is 64bit pointer in 64-bit machine!
//     printf("the tmpChar :");
//     for (int i = 0; i < len; i++) {
//         printf("%.2X ", 0xff & tmpChar[i]);
//     }
//     printf("\n");
// #endif

    // strcat(src, tmpChar);
    combineChar(src, tmpChar, start, len);
    
    free(tmpChar);
}

void ads_message::combineChar(char* &src, char* dest, uint8_t start, uint8_t len) {
    // strcat(src, dest); find '\0'
    for (int i = 0; i < len; i++) {
        *(src + start + i) = *(dest + i);  
    }
}

// set
void ads_message::setHeader(uint8_t msgHeader) {
    messageHeader_ = msgHeader;
}

void ads_message::setTail(uint8_t msgTail) {
    messageTail_ = msgTail;
}

void ads_message::setMessageSecurityType(uint8_t msgSecurType) {
    securityType_ = msgSecurType;
}

void ads_message::setMesasgeCount(uint8_t msgCount) {
    messageCount_ = msgCount;
}

void ads_message::setApplicationID(uint16_t appID) {
    applicationID_ = appID;
}

void ads_message::setIpcID(string ipcID) {
    // printf("now setting ipcID... \n");

    if (ipcID.length() != IPC_ID_LENGTH) {
        printf("set ipc id errer, length of id out of %d. \n", IPC_ID_LENGTH);
        return;
    }
    else {
        ipcID.copy(ipcID_, IPC_ID_LENGTH, 0);
    }
    // printDebugInfo(ipcID_, IPC_ID_LENGTH, "ipcID: ");
}

void ads_message::setMessageData(string msgData) {
    // printf("now setting messageData... \n");

    if (msgData.length() != messageDataLength_) {
        printf("set message data errer, length of data out of %d. \n", messageDataLength_);
        return;
    }
    else {
        messageData_ = (char*)malloc(messageDataLength_);
        memset(messageData_, 0, messageDataLength_);
        msgData.copy(messageData_, messageDataLength_, 0);
    }
    // printDebugInfo(messageData_, messageDataLength_, "messageData: ");
}

void ads_message::setMessageDataLength(uint16_t msgDataLength) {
    messageDataLength_ = msgDataLength;
}

void ads_message::setEventCreationTime() {
    // printf("now setting eventCreationTime... \n");

    long long time_last;  
    time_last = time(NULL);
    eventCreationTime_ = time_last;

#ifdef DEBUG
    printf("the eventCreationTime_ : %d  \n", eventCreationTime_);
    printf("the eventCreationTime_ : %.8X\n", eventCreationTime_);
#endif    
}

void ads_message::setDispatcherResult() {
    // printf("now setting dispatcherResult... \n");
    dispatcherResult_ = 0x0000;
}


// calculate
void ads_message::calMessageBodyLength() {
    // printf("now calculating message Body Length... \n");

    messageBodyLength_ = sizeof(eventCreationTime_) +
                         sizeof(dispatcherResult_) +
                         sizeof(messageID_) +
                         sizeof(messageDataLength_) +
                         messageDataLength_;
#ifdef DEBUG
    printf("the message messageBodyLength_ : %.2X\n", messageBodyLength_);
#endif
}

void ads_message::calMessageInfoLength() {
    // printf("now calculating message Info Length... \n");

    messageInfoLength_ = sizeof(messageLength_) +
                         sizeof(securityType_) +
                         sizeof(messageCount_) +
                         sizeof(applicationID_) +
                         IPC_ID_LENGTH;
#ifdef DEBUG
    printf("the message messageInfoLength_ : %.2X\n", messageInfoLength_);
#endif
}

void ads_message::calMessageOriginalLength() {
    // printf("now calculating message Original Length... \n");

    messageOriginalLength_ = messageInfoLength_ +
                             messageBodyEncryptedLength_ +
                             sizeof(messageCheck_);

#ifdef DEBUG
    printf("the message messageOriginalLength_ : %.2X\n", messageOriginalLength_);
#endif
}

void ads_message::calMessageLength() {
    // printf("now calculating message Total Length... \n");

    messageLength_ = messageTransformLength_ +
                     sizeof(messageHeader_) +
                     sizeof(messageTail_);
#ifdef DEBUG
    printf("the message messageLength_ : %.2X\n", messageLength_);
#endif
}

// get
int ads_message::getMessageBodyLength() {
    return messageBodyLength_;
}

int ads_message::getMessageInfoLength() {
    return messageInfoLength_;
}

void ads_message::printDebugInfo(char * toPrinted, uint8_t len, char *name) {
#ifdef DEBUG
    printf("%s", name);
    // printf("the toPrinted char: "); //void* is 64bit pointer in 64-bit machine!
    for (int i = 0; i < len; i++) {
        printf("%.2X ", 0xff & toPrinted[i]);
    }
    printf("\n");
#endif
}