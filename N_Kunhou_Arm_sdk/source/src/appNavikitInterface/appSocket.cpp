/*
 * appSocket.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: dl
 */

#include "appSocket.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
//#include <linux/tcp.h>
#include <boost/thread.hpp>
#include <string.h>
#include <iostream>
#include "jsoncpp/json.h"
#include "webinterface/NavikitInterface.h"
#include "Comm.h"


HANDLE handle_ = NULL;
#define MAX_BUF_ 49000
std::string str_total_ = "";
unsigned char uch_total_[MAX_BUF_];

appSocket::appSocket() {
    max_index=0;
    m_socket.max_fd=-1;
    connect_status=false;
}

appSocket::~appSocket() {
}


bool appSocket::Server_Init(const STRING s_ip,const int s_port)
{
	  int flags;
	  char  *host   = "127.0.0.1";

	  NavikitInterfaceCreate( host,&handle_);
	  if (NULL == handle_)
	  {
	  	printf("Create Navikit Interface Handle Failed!\n");
	  }

	  ser_sock = socket(AF_INET, SOCK_STREAM, 0);
	  if(ser_sock < 0)
	  {
		 std::cout<<"socket  Create ERR!"<<std::endl;
	  }

	  struct sockaddr_in addr;
	  addr.sin_family = AF_INET;
	  addr.sin_port = htons(s_port);
	  addr.sin_addr.s_addr = inet_addr(s_ip.c_str());
	  //bool bReuseaddr=true;
	  //setsockopt(ser_sock,SOL_SOCKET ,SO_REUSEADDR,(const char*)&bReuseaddr,sizeof(bool));
//	  std::cout<<"ser_sock:"<<ser_sock<<std::endl;

      int reuse = 1;
      setsockopt(ser_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int));
	  if(bind(ser_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
	  {
		  std::cout<<"socket  Bind ERR!"<<std::endl;
		  connect_status=false;
		  close(ser_sock);
		  return false;
	  }

	  int listen_sock = listen(ser_sock, 6);
	  if(listen_sock < 0)
	  {
		  std::cout<<"socket  Listen  ERR!"<<std::endl;
	  }

	  int rcv_buf_size = 48 * 1024,snd_buf_size = 48 * 1024;
	  setsockopt(ser_sock, SOL_SOCKET, SO_RCVBUF, &rcv_buf_size, sizeof(int));
	  setsockopt(ser_sock, SOL_SOCKET, SO_SNDBUF, &snd_buf_size, sizeof(int));


	  m_socket.client_list.insert(m_socket.client_list.begin(),6,0);
	  if((flags=fcntl( ser_sock, F_SETFL, 0))<0)
		std::cout<<"fcntl F_SETFL"<<std::endl;
	  flags |= O_NONBLOCK;
	  if(fcntl(ser_sock,F_SETFL,flags)<0)
	  std::cout<<"fcntl"<<std::endl;

	  std::cout<<"Server Socket Create Success Now"<<std::endl;
	  connect_status=true;
	  return true;
}

bool appSocket::Connected_Sta(void)
{
	return connect_status;
}

void appSocket::Server_Close(void)
{
	close(ser_sock);
	NavikitInterfaceRelease(&handle_);
	connect_status=false;
}

void appSocket::clear_buf(void)
{
	memset(uch_total_,0,MAX_BUF_);
	str_total_ = "";
}

std::string appSocket::check_json_over(void)
{
	int left_size = cComm::PatternCount(str_total_,"{");

	int right_size = cComm::PatternCount(str_total_,"}");

	if( ( left_size > 0 ) || ( right_size > 0 ))
	{
		std::cout<<"left_size:"<<left_size<<" right_size:"<<right_size<<std::endl;
		std::cout<<"total string:"<<str_total_<<std::endl;
	}

	if( (left_size > 0) || (right_size > 0)){
		if(left_size == right_size){
			return str_total_;

		}else{
			return "";
		}

	}else{
		return "";
	}

}


void appSocket::getTCPData(std::string &res)
{

	res = "";
	std::string str_send = "";
	str_total_ = (char*)uch_total_;
	memset(uch_total_,0,MAX_BUF_);
	str_send = check_json_over();
    if(str_send.size() > 0)
    {
//		std::cout<<"clear buf!!!"<<std::endl;
		clear_buf();
		res = str_send;
	}
}


void appSocket::sentTCPData(std::string str_res,int fd_socket)
{
	unsigned char* uch_res = (unsigned char*)str_res.c_str();
	int len_res = str_res.length();

	if(len_res < 1){
		return;
	}
	send(fd_socket,uch_res,len_res,MSG_DONTWAIT);
	std::cout<<"str_res:"<<str_res<<std::endl;
}



std::string appSocket::transfer_data(std::string command)
{

	std::cout<<"command:"<<command<<std::endl;
	std::string  result;

	int state = NavikitInterfaceCall(&handle_,
		                             command,
									 result);


	return result;
}


void appSocket::th_Run(int ms_loop=400)
{
	ms_loop_=ms_loop;
	boost::thread th(boost::bind(&appSocket::Thread_loop,this));
}


void appSocket::th_Stop(void)
{
	thread_run_=false;
	SLEEP(1000);
	std::cout<<"stop appSocket loop"<<std::endl;
}

void appSocket::Thread_loop(void)
{
	int retval,recvbytes;
	fd_set fds_;


    FD_ZERO(&m_socket.rds);
    FD_SET(ser_sock,&m_socket.rds);

	struct sockaddr_in client_sockaddr;
	client_size=sizeof(struct sockaddr_in);
	thread_run_=true;

	int len=sizeof(m_socket.info);
	if(m_socket.max_fd<ser_sock)
		m_socket.max_fd=ser_sock;

//	std::cout<<"m_socket.max_fd:::"<<m_socket.max_fd<<std::endl;
	while(thread_run_)
	{
		if(connect_status==false)
			continue;
//		std::cout<<"Max FD"<<m_socket.max_fd<<"server FD"<<ser_sock<<std::endl;
		fds_=m_socket.rds;
		int ret=select(m_socket.max_fd+1,&fds_,NULL,NULL,(struct timeval *)0);
//		std::cout<<"    ret : "<<ret<<"m_socket.max_fd  :"<<m_socket.max_fd<<std::endl;
		if(ret<0)
		{
			std::cout<<"Wait Select Error!!"<<std::endl;
		}
		else
		{
			if(FD_ISSET(ser_sock,&m_socket.rds)>0)
			{
				client_fd=accept(ser_sock,(struct sockaddr *)&client_sockaddr,&client_size);
				if(client_fd>0)
				{
					{
						boost::mutex::scoped_lock lock(mu_client);
						std::vector<int>::iterator it1=m_socket.client_list.begin();
						for(;it1!=m_socket.client_list.end();it1++)
						{
							if(*it1==0)
							{
								max_index++;
								*it1=client_fd;
								FD_SET(client_fd,&m_socket.rds);
//								std::cout<<"Now Client Connect:"<<client_fd<<"client count:"<<max_index<<std::endl;
								break;
							}
						}
						std::sort(m_socket.client_list.begin(),m_socket.client_list.end());
						m_socket.max_fd=m_socket.client_list[m_socket.client_list.size()-1];
//						std::cout<<"max_fd:"<<m_socket.max_fd<<std::endl;
					}


					if(max_index>6)
					{
						std::cout<<"The Max of Connect  Error"<<std::endl;
					}
				}
			}

			{
				boost::mutex::scoped_lock lock(mu_client);
				std::vector<int>::const_iterator it2=m_socket.client_list.begin();
				for(;it2!=m_socket.client_list.end();it2++)
				{
//					std::cout<<"it2:"<<*it2<<std::endl;
					if(*it2!=0)
					{
						if(FD_ISSET(*it2,&m_socket.rds))
						{

							int recvs=recv(*it2,uch_total_,MAX_BUF_,MSG_DONTWAIT);
//							std::cout<<"recvs:"<<recvs<<std::endl;
							if(recvs>0&&recvs<MAX_BUF_)
							{
//								printf("recvd  date:   %d    %d  %s\r\n",recvs,*it2,uch_total_);
								std::string Get_Date;
								getTCPData(Get_Date);
								if(!Get_Date.empty())
								{
									std::string str_res = transfer_data(Get_Date);
									sentTCPData(str_res,*it2);
								}

							}
						}

					}
				}
			}

//			std::cout<<"  m_socket Date: "<<std::endl;
	    }

		dt_.ms_loop(ms_loop_);
	}
	std::cout<<"appSocket loop  end!"<<std::endl;
}

void appSocket::Get_Socket_Date(Socket_Date &_Socket_Date)
{
	boost::mutex::scoped_lock lock(mu_client);
	_Socket_Date=m_socket;
}



void appSocket::Set_Socket_Date(Socket_Date &_Socket_Date)
{
	boost::mutex::scoped_lock lock(mu_client);
	m_socket=_Socket_Date;
}

