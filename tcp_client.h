#ifndef tcp_client_h 
#define tcp_client_h 

#include<iostream>	//cout
#include<stdio.h>	//printf
#include<string.h>	//strlen
#include<string>	//string
#include<sys/socket.h>	//socket
#include<arpa/inet.h>	//inet_addr
#include<netdb.h>	//hostent


class tcp_client
{
private:
	int sock;
	std::string address;
	int port;
	struct sockaddr_in server;
	
public:
	tcp_client();
	bool conn(std::string host, int port);
	bool send_data(char* data, int length);
        std::string receive(int);
        void send_actual_data(char identifier, int value);
};


#endif
