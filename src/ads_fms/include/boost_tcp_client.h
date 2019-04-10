#ifndef _BOOST_TCP_CLIENT_H_ 
#define _BOOST_TCP_CLIENT_H_ 

class Async_Client;

int boost_tcp_sync_send(Async_Client *client_, const char* msg, const int len); 
int boost_tcp_sync_read(Async_Client *client_, void* pVoid); 

#endif
