/*
 * process_feedback.h
 *
 *  Created on: Jul 18, 2020
 *      Author: wangrui
 */

#ifndef _PROCESS_FEEDBACK_
#define _PROCESS_FEEDBACK_

#include <iostream>
#include <string>
#include <ctime>
#include <sys/time.h>
#include "RobotStruct.h"
#include <fstream>
#include "interprocess/shared_pool.hpp"

class process_feedback
{
public:
process_feedback(){
}

~process_feedback()
{
}

process_feedback operator<<(const std::string &info)
{
     std::cout<<"<<  info:"<<info<<std::endl;
}

static SProcess_Report generate_report(std::string process_name,std::string info)
{
		time_t tNowTime;
		time(&tNowTime);

		tm* tLocalTime = localtime(&tNowTime);

		struct timeval tv;
		gettimeofday(&tv, NULL);

		KHTimeStamp _timestamp;
		_timestamp.yy_ = tLocalTime->tm_year+1900 ;
		_timestamp.mm_ = tLocalTime->tm_mon+1;
		_timestamp.dd_ = tLocalTime->tm_mday;
		_timestamp.hour_ = tLocalTime->tm_hour;
		_timestamp.min_ = tLocalTime->tm_min;
		_timestamp.sec_ = tLocalTime->tm_sec;
		_timestamp.milsec = tv.tv_usec / 1000;


		SProcess_Report report;
		memset(report.process_name_,0,100);
		if(process_name.length()>199)
			memcpy(report.process_name_,process_name.c_str(),100);
		else
			memcpy(report.process_name_,process_name.c_str(),process_name.length());

		memset(report.description,0,200);
		if(info.length()>199)
			memcpy(report.description,info.c_str(),200);
		else
			memcpy(report.description,info.c_str(),info.length());

		report.time_ = _timestamp;

		return report;
}

static SProcess_Report generate_report(std::string process_name,std::stringstream info)
{
	return generate_report(process_name,info.str());
}

static std::string add_time_stamp(std::string info)
{
	time_t tNowTime;
	time(&tNowTime);

	tm* tmLocal = localtime(&tNowTime);

	struct timeval tv;
	gettimeofday(&tv, NULL);

	std::stringstream ss;
	ss<<"["<<tmLocal->tm_year+1900 <<"-"<<tmLocal->tm_mon+1<<"-"<<tmLocal->tm_mday<<"-"<<tmLocal->tm_hour
			<<"-"<<tmLocal->tm_min<<"-"<<tmLocal->tm_sec<<"]"<<info;
	return ss.str();
}

static std::string time_stamp()
{
	time_t tNowTime;
	time(&tNowTime);

	tm* tmLocal = localtime(&tNowTime);

	struct timeval tv;
	gettimeofday(&tv, NULL);

	std::stringstream ss;
	ss<<"["<<tmLocal->tm_year+1900 <<"-"<<tmLocal->tm_mon+1<<"-"<<tmLocal->tm_mday<<"-"<<tmLocal->tm_hour
			<<"-"<<tmLocal->tm_min<<"-"<<tmLocal->tm_sec<<"]";
	return ss.str();
}

static void save_logfile(std::string filename,std::stringstream &ss,int type =0)
{
	std::fstream fo;
	if(type == 0 )
		fo.open(filename.c_str(),std::ios::out);
	else if(type == 1)
		 fo.open(filename.c_str(),std::ios::out|std::ios::app);

	std::cout<<"save_logfile(),name:"<<filename<<std::endl;

	if(fo.is_open())
	{
		fo<<ss.str();
		fo.close();
	}
}


private:

  int index;
};



#endif
