#ifndef _ASYNC_CLIENT_H_
#define _ASYNC_CLIENT_H_

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <unistd.h>
#include <memory>
//#include <curses.h>

using namespace std;
using namespace boost::asio;
using boost::asio::ip::udp;
using boost::asio::ip::tcp;



#define max_length 1024 

typedef void (*fundata_t)(const char* addr, int port, const char* data, const int len);

typedef ip::tcp::socket socket_type; 
typedef std::shared_ptr<socket_type> sock_ptr; 



class Async_Client
{
    public:
    Async_Client(boost::asio::io_service & io_service, ip::tcp::endpoint endpoint, fundata_t fundata_):iosrv_(io_service),socket_(iosrv_),
     ep(endpoint),connected_(0),sock(new socket_type(iosrv_)),fundata_(fundata_) {

    sock->async_connect(ep, bind(&Async_Client::connect_handler, this, std::placeholders::_1, sock)); //连无线会出错
        
    if(ec) {
        std::cout << boost::system::system_error(ec).what() << std::endl;
        connected_ = 0 ;
    }
    else {
        cout<<"TCP Connection is building!  Target IP is: " << ep.address().to_string() << endl << endl;
        connected_ = 1 ;
        }
    client_async_read();
    }

    void client_async_write(char *buf,int len);

    void client_async_read();

    bool client_return_status();

    private:


    void handle_read(const boost::system::error_code& error,
        size_t bytes_transferred);

    void handle_write(const boost::system::error_code& error);

    void connect_handler(const boost::system::error_code& ec, sock_ptr sock) { 
        if (ec) {
            return;
        }
        // std::cout << "receive from:" << sock->remote_endpoint().address() << std::endl; 
        connected_ = 1;
        // sock->async_read_some(buffer(m_buf), std::bind(&Client::read_handler, this, std::placeholders::_1)); 
    }
    io_service &iosrv_;
    socket_type socket_;
    ip::tcp::endpoint ep;
    boost::system::error_code ec;
    sock_ptr sock;

    char 		data_[max_length];
    bool 		connected_;
    fundata_t 	fundata_;
 

};

int InitTcpConnection(const char* addr, int port, Async_Client **client_, fundata_t fundata_);

int boost_tcp_init_connection(const char* addr, int port, Async_Client **client_, fundata_t fundata_);

int boost_tcp_sync_send(Async_Client *client_, const char* msg, const int len);

int boost_tcp_sync_read(Async_Client *client_);

#endif