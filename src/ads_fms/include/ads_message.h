#ifndef _ADS_MESSAGE_H_
#define _ADS_MESSAGE_H_

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <boost/asio.hpp>
#include <time.h>
#include <string>
//#include <curses.h>

#include "./message_define.h"
#include "./message_encrypt.h"


using namespace	std;

extern char sessionKey[17];
extern char sessionIv[17];

class ads_message
{
public:
    ads_message(uint8_t  securityType, 
                uint16_t applicationID,
                string ipcID,
                uint16_t messageID,
                string messageData,
                uint16_t messageDataLength,
                bool isLogin):
                securityType_(securityType),
                applicationID_(applicationID),
                messageID_(messageID),
                messageDataLength_(messageDataLength),
                isLogin_(isLogin) {
                    // string to char
                    setHeader();
                    setTail();
                    setIpcID(ipcID);
                    setMessageData(messageData);
                    setEventCreationTime();
                    setDispatcherResult();
                    // calculate each length of parts
                    calMessageBodyLength();
                    calMessageInfoLength();
                    // 1: build message body and encrypt
                    buildMessageBody();
                    messageBodyEncrypt();
                    // when complete encrypt, calculate the original message length
                    // and total message length
                    calMessageOriginalLength();
                    // 2: build message info
                    //    build message original (+check byte)
                    //    transform
                    buildMessageInfo();
                    messageCRCComputeandSet();
                    buildMessageOriginal();
                    messageTransform();
                    // 3: build message complete
                    calMessageLength();
                    buildMessageComplete();
    };

    ~ads_message(){};

    void updateMessage();

    // Message
    char *messageBodyEncrypted_ = NULL;
    char *messageOriginal_ = NULL;
    char *messageTransformed_ = NULL;
    char *messageComplete_ = NULL;

    char *messageInfo_ = NULL; // header without identifier
    char *messageBody_ = NULL; // dispatcher + data
    char *messageData_ = NULL;

//private:
    // 1: encrypted : for message body
    // 2: CRC check : for message info + body
    // 3: 7E & 5E transfrom : exclude message header and tail
    // 4: add header and tail
    
    // message dispatcher + messageData_
    void buildMessageBody();

    void messageBodyEncrypt();

    void buildMessageInfo();

    void messageCRCComputeandSet();

    void buildMessageOriginal();

    void messageTransform();

    void buildMessageComplete();

    // about char processing
    void convertX2Char(void* src, char* &dest, uint8_t len);

    void combineUintX(char* &src, void* dest, uint8_t start, uint8_t len);

    void combineChar(char* &src, char* dest, uint8_t start, uint8_t len);

    // member set functions
    void setHeader(uint8_t msgHeader = 0x7e);

    void setTail(uint8_t msgTail = 0x7e);

    void setMessageLength(uint16_t msgLen);

    void setMessageSecurityType(uint8_t msgSecurType);

    void setMesasgeCount(uint8_t msgCount);

    void setApplicationID(uint16_t appID);

    void setIpcID(string ipcID);

    void setMessageData(string msgData);

    void setMessageDataLength(uint16_t msgDataLength);

    void setEventCreationTime();

    void setDispatcherResult();

    // calculate
    void calMessageBodyLength();

    void calMessageInfoLength();

    void calMessageOriginalLength();

    void calMessageLength();

    // get 
    int getMessageBodyLength();

    int getMessageInfoLength();

    void printDebugInfo(char * toPrinted, uint8_t len, char *name);

    // Message Header and Tail
    uint8_t     messageHeader_;
    uint8_t     securityType_;
    uint8_t     messageCount_;
    uint16_t    applicationID_;
    char        ipcID_[17];
    uint8_t     messageCheck_;
    uint8_t     messageTail_;

    // Message Dispatcher
    uint32_t eventCreationTime_;
    uint16_t dispatcherResult_;
    uint16_t messageID_;
    uint16_t messageDataLength_;

    // Message length related
    uint16_t messageLength_;
    uint32_t messageBodyLength_;
    uint32_t messageBodyEncryptedLength_;
    uint32_t messageInfoLength_;
    uint32_t messageOriginalLength_;
    uint32_t messageTransformLength_;

    // is login
    bool isLogin_ = false;

};

#endif