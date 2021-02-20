/*
 * appScoket.h
 *
 *  Created on: Mar 8, 2018
 *      Author: dl
 */

#ifndef APPSOCKET_H_
#define APPSOCKET_H_


#include "RobotStruct.h"
#include <boost/thread.hpp>
#include "TimerDiff.h"
#include <netinet/tcp.h>

typedef struct _Socket_Date
{
	int max_fd;
    fd_set rds;
	struct tcp_info info;
	std::vector<int> client_list;
}Socket_Date;

class appSocket {
public:
	appSocket();
	virtual ~appSocket();


public:
	bool Server_Init(const STRING s_ip,const int s_port);
	void Server_Close(void);
	void th_Run(int ms_loop);
	void th_Stop(void);
	void Get_Socket_Date(Socket_Date &_Socket_Date);
	void Set_Socket_Date(Socket_Date &_Socket_Date);
	void clear_buf(void);
	std::string check_json_over(void);
	void getTCPData(std::string &res);
	void sentTCPData(std::string str_res,int fd_socket);
	std::string transfer_data(std::string command);
	bool Connected_Sta(void);
private:
	U16 ms_loop_;
	bool thread_run_;
	bool connect_status;
	cTimerDiff dt_;

	int client_fd;

	int max_index;
	unsigned int client_size;

	Socket_Date m_socket;
	boost::mutex mu_client;
public:
	int ser_sock;

private:
	void Thread_loop(void);
};

#endif /* APPSOCKET_H_ */
