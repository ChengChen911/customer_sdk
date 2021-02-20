
#include <iostream>

#include "Comm.h"
#include "jsoncpp/json.h"
#include "TransferDevice.h"
#include "appSocket.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include "webinterface/NavikitInterface.h"

#include <signal.h>

HANDLE   handle = NULL;
appSocket m_socket;

bool brun=true;

void shutdown(int sig)
{
	std::cout<<"ctrl c shut down"<<std::endl;
	brun = false;
	m_socket.th_Stop();
	m_socket.Server_Close();
	SLEEP(500);

	return;
}

bool init_IP_Port(std::string str_init_name,std::string strIP,int iPort)
{
	str_init_name += ".ini";
	std::cout<<"init appNavikitinterface ini:"<<str_init_name<<std::endl;
	std::ifstream iff(str_init_name);
	std::string str;

	if(iff.is_open())
	{

		std::getline(iff,str);

		if(str.size() > 0){
			strIP = str;
		}

		std::getline(iff,str);
		if(str.size() > 0){
			iPort = atoi(str.c_str());
		}
	}
	std::cout<<"strIP:"<<strIP<<"iPort:"<<iPort<<std::endl;
	m_socket.Server_Init(strIP,iPort);
	m_socket.th_Run(400);
}

int main(int argc, char *argv[])
{
	::signal(SIGINT, shutdown);

	std::string init_name = argv[0];

	std::string  strIP = "192.168.1.90";
	int iPort(9999);
	init_IP_Port(init_name,strIP,iPort);
	int count=0;

//	char str[1024];
	while(brun)
	{

		if(!m_socket.Connected_Sta())
		{
			std::cout<<"No Connected ,WAIT now!"<<std::endl;
			m_socket.Server_Close();
			m_socket.th_Stop();
			SLEEP(8000);
			if(!m_socket.Server_Init(strIP,iPort))
			{
				continue;
			}
			m_socket.th_Run(400);
		}


		Socket_Date socket_dat;
		m_socket.Get_Socket_Date(socket_dat);
		std::vector<int>::iterator it=socket_dat.client_list.begin();
		for(;it!=socket_dat.client_list.end();it++)
		{
			if(*it!=0)
			{
				socklen_t size=sizeof(socket_dat.info);
				getsockopt(*it, IPPROTO_TCP, TCP_INFO, &socket_dat.info, (socklen_t *)&size);
//				sprintf(str," socket FD:%d   socket_dat.info:%d",*it,socket_dat.info.tcpi_state);
//				std::cout<<str<<std::endl;
				if(socket_dat.info.tcpi_state!=1)
				{
					count++;
					*it=0;
				}
			}
		}
		if(count!=0)
		{
			count=0;
			std::sort(socket_dat.client_list.begin(),socket_dat.client_list.end());
			socket_dat.max_fd=socket_dat.client_list[socket_dat.client_list.size()-1];

			FD_ZERO(&socket_dat.rds);
			std::vector<int>::iterator it2=socket_dat.client_list.begin();
			for(;it2!=socket_dat.client_list.end();it2++)
			{
				if(*it2!=0)
				{
					if(socket_dat.max_fd==0)
						break;
					else
					{
						FD_SET(*it2,&socket_dat.rds);
					}
				}
			}
//			FD_SET(m_socket.ser_sock,&socket_dat.rds);
			FD_SET(m_socket.ser_sock,&socket_dat.rds);
			if(socket_dat.max_fd==0)
			{
//				std::cout<<"m_socket.ser_sock11:"<<m_socket.ser_sock<<std::endl;
				socket_dat.max_fd=m_socket.ser_sock;
			}
			m_socket.Set_Socket_Date(socket_dat);
		}

		SLEEP(200);


	}
//	m_socket.th_Stop();
//	m_socket.Server_Close();
	return 0;
}
