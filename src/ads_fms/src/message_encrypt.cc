#include "../include/message_encrypt.h"

void message_encrypt::aes_cbc_pcsk5_encrypt(char* pcInput, int nLen, char* pcOut, bool isSession) {
    char *key = NULL, *iv = NULL;
    if (isSession) {
        key = sessionKey;
        iv  = sessionIv;
    }
    else {
        key = rootKey;
        iv = rootIv;
    }

    //printf("sessionKey used now is: %s \n",key);
	//printf("sessionIv used now is: %s \n",iv);

    char encrypt_string[1024] = { 0 };
    AES_KEY aes;
    int n = 0;

    int nBei = nLen / AES_BLOCK_SIZE + 1;
    int nTotal = nBei * AES_BLOCK_SIZE;

    encryptTotalLength_ = getEncryptLength(nLen);

    char *enc_s = (char*)malloc(nTotal);
    int nNumber = 0;
    // printf("nBei=%d, nTotal=%d,nLen=%d\n",nBei, nTotal, nLen);

    //KCS5Padding：填充的原则是,如果长度少于16个字节，需要补满16个字节，补(16-len)个(16-len)例如：
    //"31325980"这个节符串是8个字节，16-8=8,补满后如：31325980+8个十进制的8
    //如果字符串长度正好是16字节，则需要再补16个字节的十进制的16。
    if (nLen % 16 > 0)
    {
        nNumber = nTotal - nLen;
        // printf("number=%d\n", nNumber);
    }
    else
    {
        nNumber = 16;
    }

    memset(enc_s, nNumber, nTotal);
    memcpy(enc_s, pcInput, nLen);
    // printf("enc_s=%s\n", enc_s);

    //设置加密密钥，16字节
    if (AES_set_encrypt_key((unsigned char*)key, 128, &aes) < 0)
    {
        fprintf(stderr, "Unable to set encryption key in AES\n");
        exit(-1);
    }

    AES_cbc_encrypt((unsigned char *)enc_s, (unsigned char*)encrypt_string, nTotal, &aes, (unsigned char*)iv, AES_ENCRYPT);
    n = strlen(encrypt_string);
    // printf("encrypt_string n:%d, %ld\n", n, sizeof(encrypt_string));

    // AES_decrypt();
    // base64_encode(encrypt_string, nTotal, pcOut);

    // n = strlen(pcOut);
    // printf("n:%d\n", n);

    //pcOut = encrypt_string;
    memcpy(pcOut, encrypt_string, nTotal);

    free(enc_s);
}

void message_encrypt::aes_cbc_pcsk5_decrypt(char* pcInput, int nLen, char* &pcOut, bool isSession) {
    // free(pcOut); 
    // NULL pointer cannot be freed! how to ? pre-set to NULL and check here
  
    char *key = NULL, *iv = NULL;
    if (isSession) {
        key = sessionKey;
        iv  = sessionIv;
    }
    else {
        key = rootKey;
        iv = rootIv;
    }

    //printf("sessionKey used now is: %s \n",key);
	//printf("sessionIv used now is: %s \n",iv);
    char decrypt_string[1024] = { 0 };
    AES_KEY aes;
    // int n = 0;

    // int nBei = nLen / AES_BLOCK_SIZE + 1;
    // int nTotal = nBei * AES_BLOCK_SIZE;

    // encryptTotalLength_ = getEncryptLength(nLen);

    // char *enc_s = (char*)malloc(nTotal);
    // int nNumber = 0;
    // printf("nBei=%d, nTotal=%d,nLen=%d\n",nBei, nTotal, nLen);

    //KCS5Padding：填充的原则是,如果长度少于16个字节，需要补满16个字节，补(16-len)个(16-len)例如：
    //"31325980"这个节符串是8个字节，16-8=8,补满后如：31325980+8个十进制的8
    //如果字符串长度正好是16字节，则需要再补16个字节的十进制的16。
    // if (nLen % 16 > 0)
    // {
    //     nNumber = nTotal - nLen;
    //     printf("number=%d\n", nNumber);
    // }
    // else
    // {
    //     nNumber = 16;
    // }

    // memset(enc_s, nNumber, nTotal);
    // memcpy(enc_s, pcInput, nLen);
    // printf("enc_s=%s\n", enc_s);

    //设置加密密钥，16字节
    if (AES_set_decrypt_key((unsigned char*)key, 128, &aes) < 0)
    {
        fprintf(stderr, "Unable to set encryption key in AES\n");
        exit(-1);
    }

    // here the len is different (smaller than nLen)
    // is because the function strlen() seek the '\0'
    // so if the data in pcInput contains the '\0'
    // the len = strlen() will be smaller!
    int len = strlen(pcInput);
    // printf("pcInput len: %d \n", len);

    AES_cbc_encrypt((unsigned char *)pcInput, (unsigned char*)decrypt_string, nLen, &aes, (unsigned char*)iv, AES_DECRYPT);
    
    // same as len 
    // smaller than its real size
    int n = strlen(decrypt_string);

    // sizeof return its pre-set memory size before compile
    // so the input should be static, like char c[1024];
    // or if input is a pointer, it will return the size of the pointer, 
    // but not the data size pointed by this pointer.
    // printf("decrypt_string n:%d, %ld\n", n, sizeof(decrypt_string));
    // printf("decrypt_string : %s \n",decrypt_string);

    // for (int i = 0; i < 1024; i++) {
    //     printf("%.2X ", 0xff & decrypt_string[i]);
    // }

    // AES_decrypt();
    // base64_encode(encrypt_string, nTotal, pcOut);

    //pcOut = encrypt_string;
    pcOut = (char*)malloc(nLen);
    memcpy(pcOut, decrypt_string, nLen);

    // free(enc_s);
}


int message_encrypt::base64_encode(char *in_str, int in_len, char *out_str)
{
    BIO *b64, *bio;
    BUF_MEM *bptr = NULL;
    size_t size = 0;

    if (in_str == NULL || out_str == NULL)
        return -1;

    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new(BIO_s_mem());
    bio = BIO_push(b64, bio);

    BIO_write(bio, in_str, in_len);
    BIO_flush(bio);

    BIO_get_mem_ptr(bio, &bptr);
    memcpy(out_str, bptr->data, bptr->length-1);
    out_str[bptr->length-1] = '\0';
    size = bptr->length-1;

    BIO_free_all(bio);
    return size;
}

int message_encrypt::getEncryptLength(int nLen) {
    return (nLen / AES_BLOCK_SIZE + 1) * AES_BLOCK_SIZE;
}

void message_encrypt::transmitSessionKeyIv(char * new_sessionKey, char * new_sessionIv)
{
    for(char i = 0; i<17; i++)
    {
        sessionKey[i] = new_sessionKey[i];
        sessionIv[i] = new_sessionIv[i];
    }

}