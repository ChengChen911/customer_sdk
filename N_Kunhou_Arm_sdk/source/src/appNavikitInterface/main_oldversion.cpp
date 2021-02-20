
#include <iostream>

#include "Comm.h"
#include "jsoncpp/json.h"
#include "TransferDevice.h"

#include "webinterface/NavikitInterface.h"
#include "Log4cppArm.hpp"

HANDLE   handle = NULL;
cTransferDevice* pTrans = 0;

#define MAX_BUF 102400

std::string str_total = "";
cConnPara conn_para;
//ByteList list_total;
unsigned char uch_total[MAX_BUF];

int b_print_debug = 0;
//int len_total = 0;
bool gb_run = true;

typedef boost::shared_ptr<boost::asio::ip::tcp::socket> socket_ptr;
ByteList m_lReadBuffer;

int iListen_port = 7700;
boost::asio::io_service service;
boost::asio::ip::tcp::endpoint ep( boost::asio::ip::tcp::v4(), iListen_port); // listen on 7700
boost::asio::ip::tcp::acceptor acc(service, ep);


std::vector<socket_ptr> gp_sock_list;
socket_ptr gp_sock;

void init_boost_tcp_server(int iListen_port);
void client_session(socket_ptr sock);

void clear_buf(){
//	list_total.Clear();
	memset(uch_total,0,MAX_BUF);
//	len_total = 0;
	str_total = "";
}
bool check_json_parttern(std::string str){
	int icount = str.length();
	int pair = 0;
	for(int i = 0 ; i < icount ; ++i){
		std::string tmp = str.substr(i,1);
		if( "{" ==  tmp){
			pair++;
		}else if( "}" == tmp ){
			pair--;
		}
		if(pair < 0 ){
			break;
		}
	}
	if(pair == 0){
		return true;
	}
	return false;
}
std::string check_json_over(){



	int left_size = cComm::PatternCount(str_total,"{");

	int right_size = cComm::PatternCount(str_total,"}");

	if( ( left_size > 0 ) || ( right_size > 0 ))
	{
		//std::cout<<"left_size:"<<left_size<<" right_size:"<<right_size<<std::endl;
		//std::cout<<"total string:"<<str_total<<std::endl;
	}
	//

	if( (left_size > 0) || (right_size > 0)){
		if(left_size == right_size){


			if(check_json_parttern(str_total)){
				return str_total;
			}else{
				//std::cout<<"json pattern err '{}'"<<std::endl;
				LOGS_DEBUG("appNavikitInterface_log")<<"json pattern err '{}'";
				return "";
			}


		}else{
			return "";
		}

	}else{
		return "";
	}

}
std::string transfer_data(std::string command){



	//std::cout<<"command:"<<command<<std::endl;
	std::string  result= "";

	if(command.length() < 1){

		return result;
	}

	int state = NavikitInterfaceCall(&handle,
		                             command,
									 result);


	return result;
}


void assept_session(){
	socket_ptr np_sock;
	while ( true) {
		
		np_sock = socket_ptr(new boost::asio::ip::tcp::socket(service));
		acc.accept(*np_sock);
		LOGS_DEBUG("appNavikitInterface_log")<<"assept_session,new client!!";
		if (gp_sock && gp_sock->is_open())
		{
			gp_sock->close();
		}
		gp_sock = np_sock;
		boost::thread( boost::bind(client_session, gp_sock));
	}
}

bool init_trans2(std::string str_init_name){
	//std::ifstream iff("appNavikitInterface.ini");
	str_init_name = cComm::Get_FileName(str_init_name) + ".ini";
	std::cout<<"init appNavikitinterface ini:"<<str_init_name<<std::endl;
	std::ifstream iff(str_init_name);
	std::string str;
	//std::string  strIP = "192.168.1.194";
	std::string  strIP = "COM1";
	int iPort(7700);

	if(iff.is_open()){

		std::getline(iff,str);

		if(str.size() > 0){
			strIP = str;
		}

		std::getline(iff,str);
		if(str.size() > 0){
			iPort = atoi(str.c_str());
		}
	}
	iListen_port = iPort;
	m_lReadBuffer.Init(MAX_BUF);
	boost::thread th( boost::bind(assept_session) );

	SLEEP(2000);

	return true;
}

bool getTCPData2(std::string &res){

	res = "";


	clear_buf();

	//	unsigned char* uch_data = new unsigned char[1024];
	//	memset(uch_data,0,1024);

	int len_data = 0;

	std::string str_send = "";

	int count = 50;

	while( count > 0 ){

		len_data = 0;
		m_lReadBuffer.Read(uch_total,len_data);
		
// 		if(len_data != 0){
// 			std::cout<<"rec data:"<<cComm::ByteToHexString(uch_total,len_data)<<std::endl;
// 		}
		std::string tmp = (char*)uch_total;

		str_total += tmp;

		LOGS_DEBUG("appNavikitInterface_log")<<"client rec:"<<str_total;

		memset(uch_total,0,MAX_BUF);

		str_send = check_json_over();

		if(str_send.size() > 0){
			//if(b_print_debug){
				//std::cout<<"clear buf!!!"<<std::endl;
				LOGS_DEBUG("appNavikitInterface_log")<<"clear buf!!!";
			//}
			clear_buf();
			res = str_send;
			return true;
		}else{
			count--;
			SLEEP(20);
		}
		//if(str_total.size()){

		//	std::cout<<"get data count:"<<count<<" str_total:"<<str_total.size()<<std::endl;

		//}
	

	}

	clear_buf();
	//std::cout<<"time out !!!"<<std::endl;
	LOGS_DEBUG("appNavikitInterface_log")<<"time out !!!";
	return false;

}

void sentTCPData2(std::string str_res){
	unsigned char* uch_res = (unsigned char*)str_res.c_str();
	int len_res = str_res.length();

	if(len_res < 1){
		return;
	}

	int total = 0;

	while (total < len_res) {
		int tmpLength = len_res - total > len_res ? len_res : len_res - total;

		size_t sz = gp_sock->write_some( boost::asio::buffer(uch_res + total,
			tmpLength));

		total += sz;
		//cout<<"sendData : length  = "<<tmpLength<<endl;
	}
	assert(total == len_res);

}
void shutdown(int sig)
{

	std::cout<<"shutdown child process"<<std::endl;

	gb_run = false;
	return;
}

void client_session(socket_ptr sock) {

	//gp_sock_list.push_back(sock);
	try
	{
		while ( sock->is_open()) {
			char data[512];
		
			size_t len = sock->read_some(boost::asio::buffer(data));
			if ( len > 0){
				m_lReadBuffer.Write((U8*)data,len);

			}
				//write(*sock, buffer("ok", 2));
		}
	}catch (std::exception &e)
	{
		std::cout<<"client_session" <<e.what()<<std::endl;
	}
}

int main(int argc, char *argv[])
{
	std::cout<<"Navikit Program Version: V1.3.5  "<<std::endl;
	b_print_debug = 0;
	if(argc > 1){
		b_print_debug = 1;
	}

	char    *host   = "127.0.0.1";

	std::string init_name = argv[0];

	log4cpp::Priority::Value priority = log4cpp::Priority::INFO;
	Config::getConfig("appNavikitInterface_log",priority);
	LOG.getLog("appNavikitInterface_log").setPriority(priority);  //LOGS_PRIORITY_INFO("planner_tray");

	//
	NavikitInterfaceCreate( host, 
		                   &handle);

	if(!init_trans2(init_name)){
		return -1;
	}

	if (NULL == handle)
	{
		printf("Create Navikit Interface Handle Failed!\n");
		return 0;
	}

	

	while(gb_run){


 		

		std::string str_send = "";
		if( !getTCPData2(str_send) ){
			
			//std::cout<<"no data receive:"<<std::endl;
			LOGS_DEBUG("appNavikitInterface_log")<<"no data receive:";
// 			if( ( i_max_err != 0 ) && ( i_err_count++ > i_max_err) )
// 			{
// 				std::cout<<"long time no data receive!"<<std::endl;
// 				b_conn_fault = false;
// 			}

			SLEEP(100);
			continue;
		}
		
		//if(b_print_debug){
			//std::cout<<"command:"<<str_send<<std::endl;
			LOGS_DEBUG("appNavikitInterface_log")<<"command:"<<str_send;
		//}

		std::string str_res = transfer_data(str_send);
		LOGS_DEBUG("appNavikitInterface_log")<<"return:"<<str_res;
		//std::string str_res = "res ok";

		//sentTCPData(str_res);
		sentTCPData2(str_res);
	}

	NavikitInterfaceRelease(&handle);

	return 0;
}
