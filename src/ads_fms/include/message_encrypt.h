#ifndef _MESSAGE_ENCRYPT_H_
#define _MESSAGE_ENCRYPT_H_

#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/aes.h>

class message_encrypt {

public:
    void aes_cbc_pcsk5_encrypt(char* pcInput, int nLen, char* pcOut, bool isSession);

    void aes_cbc_pcsk5_decrypt(char* pcInput, int nLen, char* &pcOut, bool isSession);
    
    int base64_encode(char *in_str, int in_len, char *out_str);

    int getEncryptLength(int nLen);

    void transmitSessionKeyIv(char * new_sessionKey, char * new_sessionIv);
    
    // sessionKey should in the loginResp_message or directly in login_message?
    char sessionKey[17];
    char sessionIv[17];

    // encrypt related parameters
    int encryptTotalLength_;
    
private:
    // Key and Iv
    char rootKey[17] = "gajklgghjaklghjg";
    char rootIv[17] = "gajklgghjaklghjg";
};

#endif