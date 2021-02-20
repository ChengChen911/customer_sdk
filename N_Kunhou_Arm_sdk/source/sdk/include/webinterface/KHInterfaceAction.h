/***************************************************************************************************
* Copyright (C), 2015-2016 Suzhou Kunhou Automation Co.,Ltd
* File name:    KHInterfaceAction.h
* Description:
* Version:      0.0.1
* History: 
* (1) Author:   taibaidl
*     Date:     2016-06-03
*     Operate:  Create
*     Version:  0.0.1
* Others: 
***************************************************************************************************/
#ifndef _KH_INTERFACE_ACTION_H_
#define _KH_INTERFACE_ACTION_H_

#include <map>
#include <vector>
#include <boost/function.hpp>

#include "XmlRpc.h"
#include "KHTypes.h"
#include "KHCommon.h"


/***************************************************************************************************
* INTERFACE   
***************************************************************************************************/
class KHInterfaceAction
{
public:
	KHInterfaceAction();
	~KHInterfaceAction();

public:
	void Init(const char *host);

public:
	std::vector<STRING> GetAllFunction();

	bool CallBackFun(const STRING             &func_name, 
		             std::map<STRING, STRING> &input_data, 
		             STRING                   &out_data);

private:
	void bindFuns();

private:
	F32 calVectorTheta(F32 start_x, 
		               F32 start_y,
		               F32 end_x,
		               F32 end_y);

private:
	//
	bool reboot(std::map<STRING, STRING> &input_data, 
		        STRING                   &out_data);

	/************ page monitor **********************************/
	//
	bool startRobot(std::map<STRING, STRING> &input_data, 
		           STRING                   &out_data);

	//
	bool setRobotPos(std::map<STRING, STRING> &input_data, 
		             STRING                   &out_data);

	bool getRobotPos(std::map<STRING, STRING> &input_data,
					STRING                    &out_data);
	bool getLaser(std::map<STRING, STRING> &input_data,
					STRING                    &out_data);
	bool getGlobalPath(std::map<STRING, STRING> &input_data,
						STRING                    &out_data);

	bool getStatus(std::map<STRING, STRING> &input_data,
						STRING                    &out_data);

	bool setTargetPos(std::map<STRING, STRING> &input_data, 
		              STRING                   &out_data);

	//
	bool selectFixPath(std::map<STRING, STRING> &input_data, 
		               STRING                   &out_data);

	bool recodePath(std::map<STRING, STRING> &input_data,
		            STRING                   &out_data);

	bool savePath(std::map<STRING, STRING> &input_data, 
		          STRING                   &out_data);

	//
	bool runPath(std::map<STRING, STRING> &input_data, 
		         STRING                   &out_data);

	bool stopRobot(std::map<STRING, STRING> &input_data, 
		           STRING                   &out_data);

	//
	bool setPathFileState(std::map<STRING, STRING> &input_data,
		                  STRING                   &out_data);

	bool setRobotLaserFileState(std::map<STRING, STRING> &input_data,
		                        STRING                   &out_data);

	//
	bool getMapList(std::map<STRING, STRING> &input_data, 
		            STRING                   &out_data);

	bool setAmclMap(std::map<STRING, STRING> &input_data,
		            STRING                   &out_data);

	bool setNaviMap(std::map<STRING, STRING> &input_data,
		            STRING                   &out_data);

	/************ page mapping**********************************/
	bool recodeMap(std::map<STRING, STRING> &input_data,
		           STRING                   &out_data);

	bool stopRecodeMap(std::map<STRING, STRING> &input_data,
		               STRING                   &out_data);

	bool robotForwardRecode(std::map<STRING, STRING> &input_data,
		                    STRING                   &out_data);

	bool robotBackwardRecode(std::map<STRING, STRING> &input_data,
			                    STRING                   &out_data);

	bool robotLeftRecode(std::map<STRING, STRING> &input_data,
			                    STRING                   &out_data);

	bool robotRightRecode(std::map<STRING, STRING> &input_data,
				                    STRING                   &out_data);


	bool robotTurnLeftRecode(std::map<STRING, STRING> &input_data,
		                     STRING                   &out_data);

	bool robotTurnRightRecode(std::map<STRING, STRING> &input_data,
		                      STRING                   &out_data);

	bool robotPauseRecode(std::map<STRING, STRING> &input_data,
		                  STRING                   &out_data);

	bool getRecodeMaps(std::map<STRING, STRING> &input_data,
		               STRING                   &out_data);

	bool mapping(std::map<STRING, STRING> &input_data,
		         STRING                   &out_data);

	bool pause(std::map<STRING, STRING> &input_data,
			     STRING                   &out_data);
	bool getMapProcess(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);
	//+  20190621
	bool setForkUp(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);
	bool setForkDown(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);
	bool setForkPos(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool begin_Charge(std::map<STRING, STRING> &input_data,
								STRING                   &out_data);
	bool stop_Charge(std::map<STRING, STRING> &input_data,
								STRING                   &out_data);
	bool startlmk(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);
	bool stoplmk(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool getForkStatus(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool get_battery_level(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool get_charging_status(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool get_start_lmk_status(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool get_stop_lmk_status(std::map<STRING, STRING> &input_data,
							STRING                   &out_data);

	bool do_function_code(int sub_index,int fuc,int para);

	bool  get_sub_task_status(int sub_index,int fuc,int para,int& current_step,int& max_step);

	bool  dispatch_task(std::map<STRING, STRING> &input_data,
		             STRING                   &out_data);
	bool  cancel_task(std::map<STRING, STRING> &input_data,
            STRING                   &out_data);
	bool  get_task_state(std::map<STRING, STRING> &input_data,
		             STRING                   &out_data);
	bool  clear_task(std::map<STRING, STRING> &input_data,
            STRING                   &out_data);

	bool  get_location_confidence(std::map<STRING, STRING> &input_data,
            STRING                   &out_data);


	//+
	bool getplanPath(std::map<STRING, STRING> &input_data,
			            STRING                   &out_data);

	bool  get_hmi_operation(std::map<STRING, STRING> &input_data,
		             STRING                   &out_data);
	bool  clear_hmi_operation(std::map<STRING, STRING> &input_data,
            STRING                   &out_data);

private:
	bool setFixPath(const STRING  &user_path);

	bool getFileList(STRING &dir_path, 
		             STRING &out_data,
		             STRING label);

	bool setParam(const STRING  &param_name, 
		          const STRING  &value);

private:
	typedef boost::function< bool (std::map<STRING, STRING> &input_data, 
		                           STRING                   &out_data)> ActionCallback;

	std::map< STRING, KHInterfaceAction::ActionCallback > functions_;

private:
	XmlRpc::XmlRpcClient *action_rpc_client_;
	XmlRpc::XmlRpcClient *param_rpc_client_;

	float  laser_dx_;

	static const int SUB_TASK_TYPE  = 1;

	static const int FOR_TASK_INDEX_FORKPOS = 1;
	static const int FOR_TASK_INDEX_CHARGE = 2;
	static const int FOR_TASK_INDEX_START_LMK = 3;
	static const int FOR_TASK_INDEX_STOP_LMK = 4;
	static const int FOR_TASK_INDEX_FORKUP = 5;
	static const int FOR_TASK_INDEX_FORKDOWN = 6;

	static const int FUNC_FORKDOWN = 1;
	static const int FUNC_FORKUP = 2;
	static const int FUNC_FORKPOS = 3;
	static const int FUNC_CHARGE = 4;
	static const int FUNC_START_LMK = 22;
	static const int FUNC_STOP_LMK = 23;
};

#endif  //_KH_INTERFACE_ACTION_H_
