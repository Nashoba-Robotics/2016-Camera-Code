/**
	C++ client example using sockets
*/
#include<iostream>	//cout
#include<stdio.h>	//printf
#include<string.h>	//strlen
#include<string>	//string
#include<sys/socket.h>	//socket
#include<arpa/inet.h>	//inet_addr
#include<netdb.h>	//hostent
#include<malloc.h>
#include "tcp_client.h"

//#define MAIN

using namespace std;

tcp_client::tcp_client()
{
	sock = -1;
	port = 0;
	address = "";
}

/**
	Connect to a host on a certain port number
*/
bool tcp_client::conn(string address , int port)
{
	//create socket if it is not already created
	if(sock == -1)
	{
		//Create socket
		sock = socket(AF_INET , SOCK_STREAM , 0);
		if (sock == -1)
		{
			perror("Could not create socket");
		}
		
		cout<<"Socket created\n";
	}
	else	{	/* OK , nothing */	}
	
	//setup address structure
	if(inet_addr(address.c_str()) == -1)
	{
		struct hostent *he;
		struct in_addr **addr_list;
		
		//resolve the hostname, its not an ip address
		if ( (he = gethostbyname( address.c_str() ) ) == NULL)
		{
			//gethostbyname failed
			herror("gethostbyname");
			cout<<"Failed to resolve hostname " << address.c_str() << endl;
			
			return false;
		}
		
		//Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
		addr_list = (struct in_addr **) he->h_addr_list;

		for(int i = 0; addr_list[i] != NULL; i++)
		{
			//strcpy(ip , inet_ntoa(*addr_list[i]) );
			server.sin_addr = *addr_list[i];
			
			cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;
			
			break;
		}
	}
	
	//plain ip address
	else
	{
		server.sin_addr.s_addr = inet_addr( address.c_str() );
	}
	
	server.sin_family = AF_INET;
	server.sin_port = htons( port );
	
	//Connect to remote server
	if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		perror("connect failed. Error");
		return false;
	}
	
	cout<<"Connected\n";
	return true;
}

/**
	Send data to the connected host
*/
bool tcp_client::send_data(char* data, int length)
{
	//Send some data
	if( send(sock , data , length , 0) < 0)
	{
		perror("Send failed : ");
		return false;
	}
	cout<<"Data send\n";
	
	return true;
}

/**
	Receive data from the connected host
*/
string tcp_client::receive(int size=512)
{
	char buffer[size];
	string reply;
	
	//Receive a reply from the server
	if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
	{
		puts("recv failed");
	}
	
	reply = buffer;
	return reply;
}

void tcp_client::send_actual_data(char identifier, int value)
{
         char* data = (char*) calloc(5, sizeof(char));
        data[0] = identifier;
       
        data[1] = (value & 0xFF000000) >> 24;
        data[2] = (value & 0x00FF0000) >> 16;
        data[3] = (value & 0x0000FF00) >> 8;
        data[4] = value & 0x000000FF; 
        
        send_data(data, 5);
        free(data);

}

#ifdef MAIN

int main(int argc , char *argv[])
{
	tcp_client c;
	string host;

        host = "roboRIO-1768-frc.local";        
	
	//connect to host
	c.conn(host , 5800);
	
        //send data
        c.send_actual_data('d',1000 );


	return 0;
}

#endif
