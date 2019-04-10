#include "../include/async_client.h"

void Async_Client::client_async_write(char *buf, int len) {
    sock->async_write_some(boost::asio::buffer(buf, len),
						   boost::bind(&Async_Client::handle_write, this,
            							boost::asio::placeholders::error));
}

void Async_Client::client_async_read() {
	sock->async_read_some(boost::asio::buffer(data_, max_length),
            boost::bind(&Async_Client::handle_read, this, 
            boost::asio::placeholders::error, 
            boost::asio::placeholders::bytes_transferred));    
}

bool Async_Client::client_return_status() {
    return connected_;
}

void Async_Client::handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!error) {
        // printf("From %s Received data Len:%d\n",this->ep.address().to_string().c_str(),
        //         bytes_transferred);

        (*fundata_)(this->ep.address().to_string().c_str(), this->ep.port(), data_, bytes_transferred);

        this->client_async_read();
    }
    else {
        printf("%s\n",error.message().c_str());
        connected_ = 0 ;
    }
}

void Async_Client::handle_write(const boost::system::error_code& error) {
    if (!error) {
		// printf("send complete. \n");
    }
    else {
		printf("send error. \n");
        printf("%s\n",error.message().c_str());
        connected_ = 0 ;
    }
}


int InitTcpConnection(const char* addr, int port, Async_Client **client_, fundata_t fundata_)
{
    io_service iosrv_;
	// printf("%s, %d\n", addr, port);
    ip::tcp::endpoint ep(ip::address::from_string(addr), port);

	// bug
    *client_ = new Async_Client(iosrv_, ep, fundata_);
    iosrv_.run();
    return 1 ;
}

int boost_tcp_init_connection(const char* addr,
							  int port, 
							  Async_Client **client_,
							  fundata_t fundata_)
{
	int timecnt=0;
	*client_ = NULL;
	// thread 
	boost::thread tmp(InitTcpConnection, addr, port, client_, fundata_);
	// guard thread 
	tmp.detach();

	while(timecnt < 50){
		timecnt++;
		usleep(20000); //20 ms
		if((*client_)->client_return_status()){
			return 0 ;
		}
	}

	*client_ = NULL ;
	return -1 ;
}

int boost_tcp_sync_send(Async_Client *client_, const char* msg, const int len)
{
	if(client_ == NULL || client_->client_return_status() == 0 ){
		printf("Not connected , please connect first \n\n");
		return -1 ;
	}
	else{
		client_->client_async_write((char*)msg, len);
		printf("send success\n");
		return 0 ;
	}

	return 1;
}

int boost_tcp_sync_read(Async_Client *client_)
{
	if(client_==NULL || client_->client_return_status()==0 ){
		printf("Not connected , please connect first \n\n");
		return -1 ;
	}
	else{
		client_->client_async_read();
		return 0 ;
	}
	return 1;
}