
#include <signal.h>


#include <iostream>


#include <string>
#include <cassert>

#include "Log4cppArm.hpp"

#include "Comm.h"
#include "Geometry.h"
#include "TimerDiff.h"

#include "RobotStruct.h"
#include "buffer_con.hpp"
#include "interprocess_core/reg_msg.h"
#include "interprocess/shared_data.hpp"
#include "interprocess/shared_pool.hpp"

#include "Acc.h"

#include "chassis/chassis_2wd_diff.h"
#include "chassis/chassis_2wd_diff_new.h"
#include "chassis/chassis_forklift.h"
#include "chassis/chassis_forklift_ex.h"
#include "chassis/chassis_forklift_kh.h"
#include "chassis/chassis_forklift_diff.h"
#include "chassis/chassis_forklift_diff_kt.h"
#include "chassis/chassis_forklift_yn.h"
#include "chassis/chassis_forklift_ld_analog.h"
#include "chassis/chassis_forklift_diff_huaxiao.h"
#include "chassis/bias_steer_wheel.h"
#include "chassis/chassis_bias_steer.h"
#include "chassis/chassis_4wd_muilt.h"
#include "chassis/chassis_omni_45.h"
#include "chassis/chassis_double_steer.h"
#include "chassis/chassis_tetrad_steer.h"
#include "chassis/chassis_amcl.h"
#include "chassis/chassis_speed.h"

#include "chassis/my_driver.h"
#include "chassis/zhongda_driver.h"
#include "chassis/zhaowei_driver.h"
#include "chassis/zhidi_driver.h"
#include "chassis/WinnerRbtDriver.h"
#include "chassis/chuangmeng_driver_multi.h"
#include "chassis/plc_driver.h"
//#include "chassis/huaxiao_driver.h"
#include "chassis/vision_driver.h"
#include "chassis/chuangmeng_driver.h"
#include "chassis/chuangmeng_protocl.h"
#include "chassis/oriental_driver.h"
#include "chassis/ketai_driver.h"
#include "chassis/anjing_driver.h"
#include "chassis/huaxiao_driver2.h"
#include "chassis/huaxiao_driver3.h"
#include "chassis/smj_driver.h"
#include "chassis/moons_CANplus.h"
#include "chassis/moons_driver.h"
#include "chassis/moons_canalyst.h"
#include "chassis/anjing_steering_driver.h"

#include "chassis/t20_half_driver.h"
#include "chassis/anjing_zhongli_driver.h"
#include "chassis/taren_small_driver.h"
#include "chassis/half_driver.h"
#include "chassis/aichong_driver.h"
#include "chassis/CopleyCanTest.h"
#include "chassis/jiateng_driver.h"
#include "chassis/yutou_driver.h"
#include "chassis/yuguan_driver.h"
//#include "Xml_serialization.h"
#include "chassis/anjing_2wd_driver.h"
#include "chassis/omni_45_driver.h"
#include "chassis/songling.h"
#include "chassis/robomodule_driver.h"

#include "chassis/zwd_doublesteer_driver.h"
#include "chassis/zwd_tetradsteer_driver.h"
#include "chassis/robomodule_mc.h"
#include "chassis/speed_driver.h"
#include "chassis/kinco_can_driver.h"
#include "chassis/muxing_driver.h"
#include "chassis/huizhuan_driver.h"
#include "chassis/amcl_driver.h"
#include "chassis/speed_driver.h"
#include "chassis/R14_can_driver.h"
#include "chassis/R14_can_driver_bj.h"
#include "chassis/R14_driver_bj_new.h"
#include "chassis/R20_can_driver.h"
#include "chassis/nanwang_driver.h"
#include "chassis/zowell_driver.h"
#include "chassis/bz2steer_driver.h"
#include "chassis/jinshi_driver.h"
#include "chassis/lejia_driver.h"
#include "chassis/l16_1183_driver.h"
#include "chassis/changdian_driver.h"
#include "chassis/etv120_driver.h"
#include "chassis/tongrui_driver.h"
#include "chassis/tongrui_rs485_driver.h"
#include "chassis/PhoenixPower_driver.h"
#include "chassis/nuoli_driver.h"
#include "chassis/KH_Kiva_driver.h"
#include "chassis/fusha_1168_driver.h"

#include "sim_driver.h"
#include "copley_driver.h"


#ifndef _WIN32
#include "chassis/can_socket.h"
#include "chassis/t20_can_driver.h"
#endif


bool brun = true;
//std::string chassis_type = "T20analog";
//std::string chassis_type = "chuangmeng60";
//std::string chassis_type = "anjing";
//std::string chassis_type = "huaxiaoforkdiff";
//std::string chassis_type = "anjing_steering_driver";
std::string chassis_type = "T20KH";
Chassis_base* bchassis = 0;
Driver_base* driver = 0;
boost::mutex mu_speed;
SSpeed g_speed;

cTimerDiff acc_dt;
F32 g_v_acc = 0.2;
F32 g_v_dec = 1;
F32 g_w_acc = 0.5;
F32 g_w_dec = 0.5;

void shutdown(int sig)
{
	std::cout<<"ctrl c shut down"<<std::endl;
	brun = false;
	shared_pool::destructor();
	SLEEP(500);

	exit(0);

	return;
}

void callback(const sclose &cl){


	if (cl.over)
	{
		std::cout<<"core shut down:"<<shared_pool::name()<<std::endl;
		shutdown(1);
	}

}
int manual_ctl = 0;
int GUI_ctl = 0;
void call_speed(const SSpeed &speed){

	F64 accdt = acc_dt.GetTime();
	accdt /= 1000000;
	acc_dt.Begin();
	//std::cout<<"accdt:"<<accdt<<std::endl;

	boost::mutex::scoped_lock lock(mu_speed);
	if(manual_ctl){
		--manual_ctl;
		std::cout<<"manunal control! no access auto control!"<<std::endl;
		return;
	}
	if(GUI_ctl){
		std::cout<<"GUI control! no access auto control!"<<std::endl;
		return;
	}
	//std::cout<<"bafore acc!!! set speed call back vx:"<<speed.vx_<<" w:"<<speed.vw_<<std::endl;
//	g_speed.vx_  = Acc::CalAcc(accdt,g_speed.vx_,g_v_acc,g_v_dec,speed.vx_);
//	g_speed.vy_  = Acc::CalAcc(accdt,g_speed.vy_,g_v_acc,g_v_dec,speed.vy_);
//	g_speed.vw_  = Acc::CalAcc(accdt,g_speed.vw_,g_w_acc,g_w_dec,speed.vw_);
	g_speed.vx_  = speed.vx_;
	g_speed.vy_  = speed.vy_;
	g_speed.vw_  = speed.vw_;
	//std::cout<<"after acc!!!! set speed call back vx:"<<g_speed.vx_<<" w:"<<g_speed.vw_<<std::endl;
	//
}
void call_remote_speed(const SSpeed &manunal_speed){
	boost::mutex::scoped_lock lock(mu_speed);
	manual_ctl = 20;
	g_speed.vx_  = manunal_speed.vx_;
	g_speed.vy_  = manunal_speed.vy_;
	g_speed.vw_  = manunal_speed.vw_;
	LOGS_DEBUG("chassis")<<"remote control set speed call back vx:"<<g_speed.vx_<<" w:"<<g_speed.vw_;
}
void call_joystick_speed(const SSpeed &joystick_speed){
	std::string speed = cComm::ConvertToString(joystick_speed.vx_);
	std::string angle = cComm::ConvertToString(joystick_speed.vw_);
	if(driver)
	{
		driver->setPara("js_speed",speed);
		driver->setPara("js_angle",angle);
	}
	//std::cout<<"joystick speed:"<<speed<<" angle:"<<angle<<std::endl;
}
void call_dio(const SDI &dio){

	//std::cout<<"set dio call back!"<<std::endl;
	if (driver)
	{
		driver->set_dio(dio);
	}

}
void call_back_forktask(const SFork_state & fork_state)
{
//	std::cout<<std::dec<<fork_state.fnc_code_<<std::endl;
	std::string fnc_code = cComm::ConvertToString(fork_state.fnc_code_);
	std::string para = cComm::ConvertToString(fork_state.para_);
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
		//driver->setPara("status",current_status);
	}
	std::cout<<"fork task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}


void call_back_forkuptask(const SFnc_Action_Download & fork_up_task)
{
	std::string sub_index = cComm::ConvertToString(fork_up_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_up_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("11" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_up_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_up_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"fork up task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"fork up task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}
void call_back_forkdowntask(const SFnc_Action_Download & fork_down_task)
{
	std::string sub_index = cComm::ConvertToString(fork_down_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_down_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("12" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_down_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_down_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"fork down task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"fork down task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_forkpostask(const SFnc_Action_Download & fork_pos_task)
{
	int err_code = 0;

	std::string sub_index = cComm::ConvertToString(fork_pos_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_pos_task.fnc_code_);
	std::string para;
	std::string precise;

	std::string para_pair;
	std::vector<std::string> vpara;
	cComm::SplitString(fork_pos_task.paras_,";",vpara);

	std::vector<std::string>::iterator it = vpara.begin();
	for (; it != vpara.end() ; ++it)
	{
		std::string &para_pair1 = *it;
		if (para_pair1.length())
		{
			std::vector<std::string> vpara1;
			cComm::SplitString(para_pair1,":",vpara1);
			if (vpara1.size() > 1)
			{
				if(vpara1[0] == "fork_height")
				{
					para = vpara1[1];
					para_pair = para_pair1;
				}else if(vpara1[0] == "fork_precise")
				{
					precise = vpara1[1];
//					para_pair = para_pair1;
				}
			}
		}
	}
	if("13" != fnc_code){
		err_code = 201;
	}
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if(0 != err_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_pos_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_pos_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = err_code;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"fork pos task err code:"<<fnc_code<<" para:"<<para_pair<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("precise",precise);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"fork pos task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_forkforwardtask(const SFnc_Action_Download & fork_forward_task)
{
	std::string sub_index = cComm::ConvertToString(fork_forward_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_forward_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("16" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_forward_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_forward_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"fork forward task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"fork forward task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_forkbacktask(const SFnc_Action_Download & fork_back_task)
{
	std::string sub_index = cComm::ConvertToString(fork_back_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_back_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("17" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_back_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_back_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"fork back task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"fork back task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_clamppulltask(const SFnc_Action_Download & clamp_pull_task)
{
	std::string sub_index = cComm::ConvertToString(clamp_pull_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(clamp_pull_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("14" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = clamp_pull_task.sub_index_;
		sfnc_action_upload.fnc_code_ = clamp_pull_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"clamp pull task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"clamp pull task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_clamppushtask(const SFnc_Action_Download & clamp_push_task)
{
	std::string sub_index = cComm::ConvertToString(clamp_push_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(clamp_push_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("15" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = clamp_push_task.sub_index_;
		sfnc_action_upload.fnc_code_ = clamp_push_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"clamp push task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"clamp push task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_clamppushlittletask(const SFnc_Action_Download & clamp_pushlittle_task)
{
	std::string sub_index = cComm::ConvertToString(clamp_pushlittle_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(clamp_pushlittle_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("27" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = clamp_pushlittle_task.sub_index_;
		sfnc_action_upload.fnc_code_ = clamp_pushlittle_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"clamp push task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"clamp push task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_climbuptask(const SFnc_Action_Download & climb_up_task)
{
	std::string sub_index = cComm::ConvertToString(climb_up_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(climb_up_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("22" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = climb_up_task.sub_index_;
		sfnc_action_upload.fnc_code_ = climb_up_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"climb up task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"climb up task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}
void call_back_climbdowntask(const SFnc_Action_Download & climb_down_task)
{
	std::string sub_index = cComm::ConvertToString(climb_down_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(climb_down_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if("23" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = climb_down_task.sub_index_;
		sfnc_action_upload.fnc_code_ = climb_down_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"climb down task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"climb down task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}
void call_back_climbrotatetask(const SFnc_Action_Download & climb_rotate_task)
{
	int err_code = 0;
	std::string sub_index = cComm::ConvertToString(climb_rotate_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(climb_rotate_task.fnc_code_);
	std::string para;

	std::string para_pair;
	std::vector<std::string> vpara;
	cComm::SplitString(climb_rotate_task.paras_,";",vpara);

	std::vector<std::string>::iterator it = vpara.begin();
	for (; it != vpara.end() ; ++it)
	{
		std::string &para_pair1 = *it;
		if (para_pair1.length())
		{
			std::vector<std::string> vpara1;
			cComm::SplitString(para_pair1,":",vpara1);
			if (vpara1.size() > 1)
			{
				if(vpara1[0] == "rotate_angle")
				{
					para = vpara1[1];
					para_pair = para_pair1;
					break;
				}
			}
		}
	}
	if("25" != fnc_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = climb_rotate_task.sub_index_;
		sfnc_action_upload.fnc_code_ = climb_rotate_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = 201;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"climb rotate task err code:"<<fnc_code<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"climb rotate task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_forkposclamptask(const SFnc_Action_Download & fork_posclamp_task)
{
	int err_code = 0;

	std::string sub_index = cComm::ConvertToString(fork_posclamp_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_posclamp_task.fnc_code_);
	std::string para;

	std::string para_pair;
	std::vector<std::string> vpara;
	cComm::SplitString(fork_posclamp_task.paras_,";",vpara);

	std::vector<std::string>::iterator it = vpara.begin();
	for (; it != vpara.end() ; ++it)
	{
		std::string &para_pair1 = *it;
		if (para_pair1.length())
		{
			std::vector<std::string> vpara1;
			cComm::SplitString(para_pair1,":",vpara1);
			if (vpara1.size() > 1)
			{
				if(vpara1[0] == "fork_height")
				{
					para = vpara1[1];
					para_pair = para_pair1;
					break;
				}
			}
		}
	}
	if("26" != fnc_code){
		err_code = 201;
	}
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;

	if(0 != err_code){
		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_posclamp_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_posclamp_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_REJECT;
		sfnc_action_upload.error_code_ = err_code;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"fork clamp task err code:"<<fnc_code<<" para:"<<para_pair<<std::endl;
		return;
	}
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"fork pos task, code:"<<fnc_code<<" para:"<<para<<std::endl;
}

void call_back_cancelforktask(const SFnc_Action_Download & fork_task)
{
	std::string sub_index = cComm::ConvertToString(fork_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_task.fnc_code_);
	std::string para = "1";
	//std::string current_status = cComm::ConvertToString(fork_state.current_status_);
//	std::cout<<"111"<<std::endl;
	if(driver)
	{
//		std::cout<<"222"<<std::endl;
		fnc_code = "0";
		driver->setPara("sub_index",sub_index);
		driver->setPara("para",para);
		driver->setPara("fnc_code",fnc_code);

		SFnc_Action_Upload sfnc_action_upload;
		sfnc_action_upload.sub_index_ = fork_task.sub_index_;
		sfnc_action_upload.fnc_code_ = fork_task.fnc_code_;
		sfnc_action_upload.task_state_ = Task_State::TASK_DONE;
		sfnc_action_upload.error_code_ = 0;
		shared_pool::Publish(shared_pool::name(),"update_action_status",sfnc_action_upload);
		std::cout<<"cancel fork task, code:"<<fnc_code<<" para:"<<para<<std::endl;
	}
}

void set_forward_para(const F32 &forward_para ){
	std::string forward_para_ = cComm::ConvertToString(forward_para);
	if(driver)
	{
		driver->setPara("forward_para",forward_para_);
	}
}

void chassis_control(const Srecoder & srecoder){
	std::string chassis_control = srecoder.cmd_file_;

	std::vector<std::string> control_para;
	cComm::SplitString(chassis_control,";",control_para);
	std::cout<<"chassis_control:    "<<chassis_control<<"  "<<control_para.size()<<std::endl;

	std::vector<std::string> control_type;
	cComm::SplitString(control_para[0],":",control_type);

	if(control_type.size() && "chassis_control" == control_type[0]){
		if("setspeed" == control_type[1] && control_para.size()==4 ){
			F32 vx = 0;
			std::vector<std::string> control_vx;
			cComm::SplitString(control_para[1],":",control_vx);
			cComm::ConvertToNum(vx,control_vx[1]);
			cComm::RangeIt(vx,(F32)-0.8,(F32)1.5);

			F32 vy = 0;
			std::vector<std::string> control_vy;
			cComm::SplitString(control_para[2],":",control_vy);
			cComm::ConvertToNum(vy,control_vy[1]);
			cComm::RangeIt(vy,(F32)-0.8,(F32)1.5);

			F32 vw = 0;
			std::vector<std::string> control_vw;
			cComm::SplitString(control_para[3],":",control_vw);
			cComm::ConvertToNum(vw,control_vw[1]);
			cComm::RangeIt(vw,(F32)-0.4,(F32)0.4);

			if(fabs(vx)>10e-6 || fabs(vy)>10e-6  || fabs(vw)>10e-6){
				GUI_ctl = 1;
			}else{
				GUI_ctl = 0;
			}
			{
				boost::mutex::scoped_lock lock(mu_speed);
				g_speed.vx_  = vx;
				g_speed.vy_  = vy;
				g_speed.vw_  = vw;
			}

			std::cout<<"chassis control set speed call back: "<<GUI_ctl<<" vx:"<<g_speed.vx_<<" w:"<<g_speed.vw_<<std::endl;
			LOGS_DEBUG("chassis")<<"chassis control set speed call back vx:"<<g_speed.vx_<<" w:"<<g_speed.vw_;
		}else if("reset_para" == control_type[1] && control_para.size()==1 ){

			std::string chassis_para = "dia:0.200;steer_max_angle:120;D1:0.8;L1:0.0;D2:-0.8;L2:0.0";
			Config::getConfig("chassis_para",chassis_para);
			std::cout<<chassis_para<<std::endl;
			if (bchassis){
				bchassis->init(chassis_para);
			}

			std::string chassis_com = "port:ttyUSB0;ratio:0.021";
			Config::getConfig("chassis_com",chassis_com);
			std::cout<<chassis_com<<std::endl;
			std::vector<std::string> d_para;
			cComm::SplitString(chassis_com,";",d_para);

			std::vector<std::string>::iterator d_it = d_para.begin();
			for (; d_it != d_para.end() ; ++d_it){
				std::string &d_para_pair = *d_it;
				if (d_para_pair.length()){
					std::vector<std::string> d_para2;
					cComm::SplitString(d_para_pair,":",d_para2);
					if (d_para2.size() > 1 && driver){
						driver->setPara(d_para2[0],d_para2[1]);
					}
				}
			}
			bchassis->resetOdom();
			std::cout<<"reset chassis odom!"<<std::endl;
		}
	}
}

/*mengli scram unlock*/
void Set_ML_Scram(const int &scram)
{
	std::string fnc_scram = cComm::ConvertToString(scram);
	if(driver)
	{
		driver->setPara("scram_unlock",fnc_scram);
	}
}

void Curtis_Reset(const SFnc_Action_Download & fork_curtisreset_task)
{
	std::string sub_index = cComm::ConvertToString(fork_curtisreset_task.sub_index_);
	std::string fnc_code = cComm::ConvertToString(fork_curtisreset_task.fnc_code_);
	if(driver)
	{
		driver->setPara("sub_index",sub_index);
		driver->setPara("fnc_code",fnc_code);
	}
	std::cout<<"main call Curtis_Reset: "<<fnc_code<<std::endl;
}

/*bozhong battery task start*/
void Get_SSub_Task(const SSub_task_fnc &current_task)
{
	if(current_task.function_code_ == 4)
	{
		std::string fnc_code = "4";

		if(driver)
		{
			driver->setPara("fnc_code",fnc_code);
		}
//		std::cout<<"Get_SSub_Task():"<<std::endl;
	}
}
void Get_Stop_Task(const SSub_task_fnc &current_task)
{
	std::string fnc_code = "5";
	if(driver)
	{
		driver->setPara("fnc_code",fnc_code);
	}
}
/*bozhong battery task end*/

void call_back_amcl( const SPos &amcl_pos ){

	if(driver)
	{
		driver->setAmclPos(amcl_pos);
	}
}
void init_shared_pool(char *argv[]){

	shared_pool::init(argv);
	std::cout<<"chassis init call back"<<std::endl;

	boost::function<void( const sclose &cl)> fnc_shutdown;
	fnc_shutdown = boost::bind(callback,_1);
	shared_pool::Subscribe(shared_pool::name(),"shutdown",fnc_shutdown);

	boost::function<void( const SSpeed &speed )> fnc_speed;
	fnc_speed = boost::bind(call_speed,_1);
	shared_pool::Subscribe(shared_pool::name(),"setspeed",fnc_speed);

	boost::function<void( const SSpeed &speed )> fnc_remote_speed;
	fnc_remote_speed = boost::bind(call_remote_speed,_1);
	shared_pool::Subscribe(shared_pool::name(),"remote_setspeed",fnc_remote_speed);

	boost::function<void( const SSpeed &speed )> fnc_joystick_speed;
	fnc_joystick_speed = boost::bind(call_joystick_speed,_1);
	shared_pool::Subscribe(shared_pool::name(),"joystick_setspeed",fnc_joystick_speed);

	boost::function<void( const SDI &dio )> fnc_io;
	fnc_io = boost::bind(call_dio,_1);
	shared_pool::Subscribe(shared_pool::name(),"gpio_di",fnc_io);

	if((chassis_type == "ketai") || (chassis_type == "tongri")){
	boost::function<void(  const SPos &amcl_pos )> fnc_amcl;
	fnc_amcl = boost::bind(&call_back_amcl,_1);
	shared_pool::Subscribe(shared_pool::name(),"amcl_pos",fnc_amcl);
	}

	boost::function<void(const SFork_state & fork_state)> fnc_forktask;
	fnc_forktask  = boost::bind(&call_back_forktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"set_fork_status",fnc_forktask);


	boost::function<void(const SFnc_Action_Download & fork_up_task)> fnc_forkuptask;
	fnc_forkuptask  = boost::bind(&call_back_forkuptask,_1);
	shared_pool::Subscribe(shared_pool::name(),"forkup_action",fnc_forkuptask);

	boost::function<void(const SFnc_Action_Download & fork_down_task)> fnc_forkdowntask;
	fnc_forkdowntask  = boost::bind(&call_back_forkdowntask,_1);
	shared_pool::Subscribe(shared_pool::name(),"forkdown_action",fnc_forkdowntask);

	boost::function<void(const SFnc_Action_Download & fork_pos_task)> fnc_forkpostask;
	fnc_forkpostask  = boost::bind(&call_back_forkpostask,_1);
	shared_pool::Subscribe(shared_pool::name(),"forkpos_action",fnc_forkpostask);

	boost::function<void(const SFnc_Action_Download & fork_forward_task)> fnc_forkforwardtask;
	fnc_forkforwardtask  = boost::bind(&call_back_forkforwardtask,_1);
	shared_pool::Subscribe(shared_pool::name(),"forkforward_action",fnc_forkforwardtask);

	boost::function<void(const SFnc_Action_Download & fork_back_task)> fnc_forkbacktask;
	fnc_forkbacktask  = boost::bind(&call_back_forkbacktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"forkback_action",fnc_forkbacktask);

	boost::function<void(const SFnc_Action_Download & clamp_pull_task)> fnc_clamppulltask;
	fnc_clamppulltask  = boost::bind(&call_back_clamppulltask,_1);
	shared_pool::Subscribe(shared_pool::name(),"clamppull_action",fnc_clamppulltask);

	boost::function<void(const SFnc_Action_Download & clamp_push_task)> fnc_clamppushtask;
	fnc_clamppushtask  = boost::bind(&call_back_clamppushtask,_1);
	shared_pool::Subscribe(shared_pool::name(),"clamppush_action",fnc_clamppushtask);

	boost::function<void(const SFnc_Action_Download & clamp_pushlittle_task)> fnc_clamppushlittletask;
	fnc_clamppushlittletask  = boost::bind(&call_back_clamppushlittletask,_1);
	shared_pool::Subscribe(shared_pool::name(),"clamppushlittle_action",fnc_clamppushlittletask);

	boost::function<void(const SFnc_Action_Download & climb_up_task)> fnc_climbuptask;
	fnc_climbuptask  = boost::bind(&call_back_climbuptask,_1);
	shared_pool::Subscribe(shared_pool::name(),"climbup_action",fnc_climbuptask);

	boost::function<void(const SFnc_Action_Download & climb_down_task)> fnc_climbdowntask;
	fnc_climbdowntask  = boost::bind(&call_back_climbdowntask,_1);
	shared_pool::Subscribe(shared_pool::name(),"climbdown_action",fnc_climbdowntask);

	boost::function<void(const SFnc_Action_Download & climb_rotate_task)> fnc_climbrotatetask;
	fnc_climbrotatetask  = boost::bind(&call_back_climbrotatetask,_1);
	shared_pool::Subscribe(shared_pool::name(),"climbrotate_action",fnc_climbrotatetask);

	boost::function<void(const SFnc_Action_Download & fork_posclamp_task)> fnc_forkposclamptask;
	fnc_forkposclamptask  = boost::bind(&call_back_forkposclamptask,_1);
	shared_pool::Subscribe(shared_pool::name(),"forkposclamp_action",fnc_forkposclamptask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelforkuptask;
	fnc_cancelforkuptask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_forkup_action",fnc_cancelforkuptask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelforkdowntask;
	fnc_cancelforkdowntask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_forkdown_action",fnc_cancelforkdowntask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelforkpostask;
	fnc_cancelforkpostask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_forkpos_action",fnc_cancelforkpostask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelforkforwardtask;
	fnc_cancelforkforwardtask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_forkforward_action",fnc_cancelforkforwardtask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelforkbacktask;
	fnc_cancelforkbacktask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_forkback_action",fnc_cancelforkbacktask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelclamppulltask;
	fnc_cancelclamppulltask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_clamppull_action",fnc_cancelclamppulltask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelclamppushtask;
	fnc_cancelclamppushtask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_clamppush_action",fnc_cancelclamppushtask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelclamppushlittletask;
	fnc_cancelclamppushlittletask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_clamppushlittle_action",fnc_cancelclamppushlittletask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelclimbuptask;
	fnc_cancelclimbuptask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_climbup_action",fnc_cancelclimbuptask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelclimbdowntask;
	fnc_cancelclimbdowntask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_climbdown_action",fnc_cancelclimbdowntask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelclimbrotatetask;
	fnc_cancelclimbrotatetask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_climbrotate_action",fnc_cancelclimbrotatetask);

	boost::function<void(const SFnc_Action_Download & fork_task)> fnc_cancelforkposclamptask;
	fnc_cancelforkposclamptask  = boost::bind(&call_back_cancelforktask,_1);
	shared_pool::Subscribe(shared_pool::name(),"cancel_forkposclamp_action",fnc_cancelforkposclamptask);

	boost::function<void(const SFnc_Action_Download & fork_curtisreset_task)> fnc_curtis_reset;
	fnc_curtis_reset  = boost::bind(&Curtis_Reset,_1);
	shared_pool::Subscribe(shared_pool::name(),"curtis_reset",fnc_curtis_reset);

	boost::function<void(const F32 & forward_para)> fnc_set_forward_para;
	fnc_set_forward_para  = boost::bind(&set_forward_para,_1);
	shared_pool::Subscribe(shared_pool::name(),"set_forward_para",fnc_set_forward_para);

	boost::function<void(const Srecoder & srecoder)> fnc_chassis_control;
	fnc_chassis_control  = boost::bind(&chassis_control,_1);
	shared_pool::Subscribe(shared_pool::name(),"chassis_control",fnc_chassis_control);

	boost::function<void(const int & scram)> fnc_scram;
	fnc_scram  = boost::bind(&Set_ML_Scram,_1);
	shared_pool::Subscribe(shared_pool::name(),"scram_unlock",fnc_scram);


	std::cout<<"chassis init call back over "<<std::endl;

}

void inti_chassis(Chassis_base* &base){


	std::string chassis_type = "jinshi";

	Config::getConfig("chassis_type",chassis_type);

	//copley std::string chassis_para = "dia:0.18;axle:0.560";
	//copley_can std::string chassis_para = "dia:0.220;axle:0.335"
	//kps std::string chassis_para = "dia:0.202;axle:0.385";
	//ketai1 std::string chassis_para = "dia:0.156;axle:0.530";
	//ketai2 std::string chassis_para = "adia:0.152;D:0.620;L:0.0;axle:0.309";

	//std::string chassis_para = "adia:0.23;fdia:0.08;D:2.368;L:0.0;axle:1.04";
	//chuangmeng std::string chassis_para = "adia:0.128;D:0.627;L:0.0;axle:0.339";
	//std::string chassis_para = "dia:0.126;axle:0.339";
	//yn std::string chassis_para = "adia:0.18;D:1.32;L:0.0;Pid_p:3.0";
	//anjing 2wd std::string chassis_para = "adia:0.255;D:1.57;L:0.0";
	//huaxiao2 std::string chassis_para = "adia:0.128;D:0.627;L:0.0;axle:0.339";
	//std::string chassis_para = "dia:0.15;axle:0.26";
	//jd std::string chassis_para = "adia:0.255;D:1.52;L:0.0"
	std::string chassis_para = "dia:0.200;steer_max_angle:120;D1:0.8;L1:0.0;D2:-0.8;L2:0.0";
	//std::string chassis_para = "dia:0.064;axle:0.165;";
	//std::string chassis_para = "dia:0.250;wheel_base:2.0;wheel_axle:0.58";
	//std::string chassis_para = "dia:0.250;steer_max_angle:120;D1:0.5;D2:-0.5;L1:-0.0;L2:0.0";//double_steer
	//std::string chassis_para = "dia:0.250;steer_max_angle:120;D1:0.85;D2:-0.85;D3:0.85;D4:-0.85;L1:0.8;L2:0.8;L3:-0.8;L4:-0.8";//t_steer

	Config::getConfig("chassis_para",chassis_para);
	//kps std::string chassis_com = "port:ttyUSB0;counts:60;reduction:4.3";
	//yn std::string chassis_com = "port:ttyUSB0;counts:24;reduction:9900-31.4";
	//jd std::string chassis_com = "port:ttyS3;counts:5900;reduction:1";
	//colpey_can std::string chassis_com = "port:CAN;counts:10000;reduction:20";
	std::string chassis_com = "IP:192.168.0.145;port:30001.30002.30003";
	//std::string chassis_com = "port:ttyUSB0;ratio:0.021";
	Config::getConfig("chassis_com",chassis_com);

	std::cout<<"chassis_type:"<<chassis_type<<std::endl;
	std::cout<<"chassis_para:"<<chassis_para<<std::endl;
	std::cout<<"chassis_com"<<chassis_com<<std::endl;

	Config::getConfig("chassis_vacc",g_v_acc);
	Config::getConfig("chassis_vdec",g_v_dec);
	Config::getConfig("chassis_wacc",g_w_acc);
	Config::getConfig("chassis_wdec",g_w_dec);

	std::cout<<"chassis_vacc:"<<g_v_acc<<std::endl;
	std::cout<<"chassis_vdec:"<<g_v_dec<<std::endl;
	std::cout<<"chassis_wacc"<<g_w_acc<<std::endl;
	std::cout<<"chassis_wdec"<<g_w_dec<<std::endl;

	if(chassis_type == "zhidi"){
		base =(Chassis_base*)new Chassis_forklift();
	}else if(chassis_type == "huaxiaoforkdiff"){
		base =(Chassis_base*)new Chassis_forklift_diff_huaxiao();
	}else if(chassis_type == "chuangmeng"){
		base =(Chassis_base*)new Chassis_forklift_diff();
	}else if(chassis_type == "chuangmeng60"){
		base =(Chassis_base*)new Chassis_forklift_yn();
	}else if(chassis_type == "T20analog"){
		base =(Chassis_base*)new Chassis_forklift_ld_analog();
	}else if(chassis_type == "oriental"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "ketai"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "anjing"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "anjing_steering_driver"){
		base =(Chassis_base*)new bias_steer_wheel();
	}else if(chassis_type == "anjing_2wd"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "daji"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "T20Hdriver"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "huaxiao3"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "moons_driver"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "moons_can"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "moons_canalyst"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "half_driver"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "T20KH"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "T50KH"){
		base =(Chassis_base*)new Chassis_bias_steer();
	}else if(chassis_type == "L16KH"){
		base =(Chassis_base*)new Chassis_bias_steer();
	}else if(chassis_type == "motatech"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "copley_can"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "kinco_can"){
		base =(Chassis_base*)new Chassis_2wd_diff();
	}else if(chassis_type == "yuguan"){
		base =(Chassis_base*)new Chassis_4wd_muilt();
	}else if(chassis_type == "omni"){
		base =(Chassis_base*)new Chassis_omni_45();
	}else if(chassis_type == "songling"){
		base =(Chassis_base*)new Chassis_omni_45();
	}else if(chassis_type == "robo"){
		base =(Chassis_base*)new Chassis_forklift_kh();
	}else if(chassis_type == "double_steer"){
		base =(Chassis_base*)new Chassis_double_steer();
	}else if(chassis_type == "tetrad_steer"){
		base =(Chassis_base*)new Chassis_tetrad_steer();
	}else if(chassis_type == "ruijie"){
		base =(Chassis_base*)new Chassis_speed();
	}else if(chassis_type == "huizhuan"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "zowell"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "jieshide"){
		base =(Chassis_base*)new Chassis_speed();
	}else if(chassis_type == "tongri"){
		base =(Chassis_base*)new Chassis_amcl();
	}else if(chassis_type == "amcl"){
		base =(Chassis_base*)new Chassis_amcl();
	}else if(chassis_type == "R14KH"){
		base =(Chassis_base*)new Chassis_bias_steer();
	}else if(chassis_type == "R14KH_CLAMP"){
		base =(Chassis_base*)new Chassis_bias_steer();
	}else if(chassis_type == "nanwang"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "BZs2"){
		base =(Chassis_base*)new Chassis_double_steer();
	}else if(chassis_type == "jinshi"){
		base =(Chassis_base*)new Chassis_speed();
	}else if(chassis_type == "jingzhe"){
		base =(Chassis_base*)new Chassis_speed();
	}else if(chassis_type == "R20KH"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "lejia"){
		base =(Chassis_base*)new Chassis_2wd_diff_new();
	}else if(chassis_type == "L16KH_1183"){
		base =(Chassis_base*)new Chassis_bias_steer();
	}else if(chassis_type == "changdian"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "ETV120KH"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "tongrui"){
		base =(Chassis_base*)new Chassis_2wd_diff_new();
	}else if(chassis_type == "tongrui_485"){
		base =(Chassis_base*)new Chassis_2wd_diff_new();
	}else if(chassis_type == "PhoenixPower"){
		base =(Chassis_base*)new Chassis_double_steer();
	}else if(chassis_type == "nuoli"){
		base =(Chassis_base*)new Chassis_forklift_ex();
	}else if(chassis_type == "KH_Kiva"){
		base =(Chassis_base*)new Chassis_2wd_diff_new();
	}else if(chassis_type == "L16fusha"){
		base =(Chassis_base*)new Chassis_bias_steer();
	}else{
		base =(Chassis_base*)new Chassis_2wd_diff();
	}
	

	if ((base) && ( base->init( chassis_para )))
	{
		if(chassis_type == "copley"){
			driver = (Driver_base*)new copley_driver();
		}else if(chassis_type == "kps"){
			driver = (Driver_base*)new My_driver();
		}else if(chassis_type == "zhongda"){
			driver = (Driver_base*)new zhongda_driver();
		}else if(chassis_type == "zhaowei"){
			driver = (Driver_base*)new zhaowei_driver();
		}else if(chassis_type == "zhidi"){
			driver = (Driver_base*)new zhidi_driver();
		}else if(chassis_type == "WinnerRbt"){
			driver = (Driver_base*)new WinnerRbtDriver();
		}else if(chassis_type == "huaxiaoforkdiff"){
			driver = (Driver_base*)new huaxiao_driver2();
		}else if(chassis_type == "huaxiao3"){
			driver = (Driver_base*)new huaxiao_driver3();
		}else if(chassis_type == "vision"){
			driver = (Driver_base*)new vision::vision_driver();
		}else if(chassis_type == "chuangmeng"){
			driver = (Driver_base*)new chuangmeng_driver();
		}else if(chassis_type == "T20analog"){
			driver = (Driver_base*)new plc_driver();
		}else if(chassis_type == "chuangmeng60"){
			driver = (Driver_base*)new chuangmeng_driver_multi();
		}else if(chassis_type == "oriental"){
			driver = (Driver_base*)new oriental_driver();
		}else if(chassis_type == "ketai"){
			driver = (Driver_base*)new ketai::ketai_driver();
		}else if(chassis_type == "anjing"){
			driver = (Driver_base*)new anjing::anjing_driver();
		}else if(chassis_type == "anjing_2wd"){
			driver = (Driver_base*)new anjing_2wd_driver();
		}else if(chassis_type == "anjing_steering_driver"){
			driver = (Driver_base*)new anjing_steering_driver();
		}else if(chassis_type == "daji"){
			driver = (Driver_base*)new smj_driver();
		}else if(chassis_type == "T20Hdriver"){
			driver = (Driver_base*)new t20_half_driver();
		}else if(chassis_type == "moons_can"){
			driver = (Driver_base*)new moons_CANplus();
		}else if(chassis_type == "moons_canalyst"){
			driver = (Driver_base*)new moons_canalyst();
		}else if(chassis_type == "moons_driver"){
			driver = (Driver_base*)new moons_driver();
		}else if(chassis_type == "half_driver"){
			driver = (Driver_base*)new half_driver();
		}else if(chassis_type == "T20KH"){
			driver = (Driver_base*)new t20_can_driver();
		}else if(chassis_type == "T50KH"){
			driver = (Driver_base*)new t20_can_driver();
		}else if(chassis_type == "L16KH"){
			driver = (Driver_base*)new t20_can_driver();
		}else if(chassis_type == "motatech"){
			driver = (Driver_base*)new aichong_driver();
		}else if(chassis_type == "copley_can"){
			driver = (Driver_base*)new copley_CANplus();
		}else if(chassis_type == "kinco_can"){
			driver = (Driver_base*)new kinco_can_driver();
		}else if(chassis_type == "jiateng"){
			driver = (Driver_base*)new jiateng_driver();
		}else if(chassis_type == "yuguan"){
			driver = (Driver_base*)new yuguan_driver();
		}else if(chassis_type == "omni"){
			driver = (Driver_base*)new robomodule_mc();
		}else if(chassis_type == "songling"){
			driver = (Driver_base*)new songling_driver();
		}else if(chassis_type == "robo"){
			driver = (Driver_base*)new robomodule_driver();
		}else if(chassis_type == "tetrad_steer"){
			driver = (Driver_base*)new zwd_tetradsteer();
		}else if(chassis_type == "double_steer"){
			driver = (Driver_base*)new zwd_doublesteer();
		}else if(chassis_type == "ruijie"){
			driver = (Driver_base*)new speed_driver();
		}else if(chassis_type == "muxing"){
			driver = (Driver_base*)new muxing_driver();
		}else if(chassis_type == "huizhuan"){
			driver = (Driver_base*)new huizhuan_driver();
		}else if(chassis_type == "jieshide"){
			driver = (Driver_base*)new speed_driver();
		}else if(chassis_type == "tongri"){
			driver = (Driver_base*)new amcl_driver();
		}else if(chassis_type == "amcl"){
			driver = (Driver_base*)new amcl_driver();
		}else if(chassis_type == "R14KH"){
			driver = (Driver_base*)new r14_can_driver_bj();
//		}else if(chassis_type == "R14KH_CLAMP"){
//			driver = (Driver_base*)new r14_can_driver_bj();
		}else if(chassis_type == "R14KH_CLAMP"){
			driver = (Driver_base*)new r14_driver_bj_new();
		}else if(chassis_type == "nanwang"){
			driver = (Driver_base*)new nanwang_driver();
		}else if(chassis_type == "zowell"){
			driver = (Driver_base*)new zowell_driver();
		}else if(chassis_type == "BZs2"){
			driver = (Driver_base*)new bz2steer_driver();
		}else if(chassis_type == "jinshi"){
			driver = (Driver_base*)new jinshi_driver();
		}else if(chassis_type == "jingzhe"){
			driver = (Driver_base*)new speed_driver();
		}else if(chassis_type == "R20KH"){
			driver = (Driver_base*)new r20_can_driver();
		}else if(chassis_type == "lejia"){
			driver = (Driver_base*)new lejia_driver();
		}else if(chassis_type == "L16KH_1183"){
			driver = (Driver_base*)new l16_1183_driver();
		}else if(chassis_type == "changdian"){
			driver = (Driver_base*)new changdian_driver();
		}else if(chassis_type == "ETV120KH"){
			driver = (Driver_base*)new etv120_driver();
		}else if(chassis_type == "tongrui"){
			driver = (Driver_base*)new tongrui_driver();
		}else if(chassis_type == "tongrui_485"){
			driver = (Driver_base*)new tongrui_rs485_driver();
		}else if(chassis_type == "PhoenixPower"){
			driver = (Driver_base*)new PhoenixPower_driver();
		}else if(chassis_type == "nuoli"){
			driver = (Driver_base*)new nuoli_driver();
		}else if(chassis_type == "KH_Kiva"){
			driver = (Driver_base*)new KH_Kiva_driver();
		}else if(chassis_type == "L16fusha"){
			driver = (Driver_base*)new fusha_1168_driver();
		}else{
			driver = (Driver_base*)new sim_driver();
		}
		std::cout<<"init driver type:"<<chassis_type<<std::endl;

#ifdef _WIN32
		//if( (driver) && ( driver->init("port:COM1;counts:30;reduction:30")))
		if( (driver) && ( driver->init("port:COM1;ratio:0.003393")))
#else
		if( (driver) && ( driver->init( chassis_com )))
#endif
		{

			base->initDriver(driver);
			base->run(41);
		}
		else{
			std::cout<<"driver init err "<<chassis_com<<" !!!!!!"<<std::endl;
			SLEEP(2000);
			exit(0);
		}

	}else{
		std::cout<<"device init err!!"<<std::endl;

	}
}
void init_log(){

	log4cpp::Priority::Value priority = log4cpp::Priority::INFO;
	Config::getConfig("chassis_priority",priority);
	std::cout<<"chassis_priority:"<<int(priority)<<std::endl;
	LOG.getLog("chassis").setPriority(log4cpp::Priority::Value(priority));  //LOGS_PRIORITY_INFO("planner_tray");

	//
	LOGS_INFO("chassis")<<"chassis start";
}

int main(int argc, char *argv[])
{
	std::cout<<"Navikit Program Version: V1.4.9"<<std::endl;
	::signal(SIGINT, shutdown);

	init_log();

	//init shared pool : shutdown call back 


	if(argc > 0){

		if(argc > 1){
			init_shared_pool(argv);
		}
		inti_chassis(bchassis);
		assert(bchassis);

		bchassis->setSpeed(0.0,0.0,0.0);
		SLEEP(100);
	}


	cTimerDiff dt;

	F32 x = 0.0;
	F32 y = 0.0;
	F32 a = 0.0;
	SOdomSpeed odom_speed;

	while(brun){
		dt.Begin();
		
		F32 ox,oy,oa;
		F32 ovx,ovy,ow;
		{
			boost::mutex::scoped_lock lock(mu_speed);
			x = g_speed.vx_;
			y = g_speed.vy_;
			a = g_speed.vw_;
 			//x = 0.0;
//			std::cout<<"STARTTTTT"<<std::endl;
//			std::cin>>x;
//			std::cin>>y;
//			std::cin>>a;
//			std::cout<<"OKKKKKKKK"<<std::endl;
 			//a = 0.1;
		}
		bchassis->setSpeed(x,y,a);

		bchassis->getOdom(ox,oy,oa);

		bchassis->getSpeed(ovx,ovy,ow);

		odom_speed.x_ = ox;
		odom_speed.y_ = oy;
		odom_speed.th_ = oa;
		odom_speed.vx_ = ovx;
		odom_speed.vy_ = ovy;
		odom_speed.vw_ = ow;

		if(argc > 1){
			shared_pool::Publish(shared_pool::name(),"odom",odom_speed);
			//std::cout<<"x="<<ox<<"  y="<<oy<<"  w="<<oa<<std::endl;
		}
		dt.ms_loop(40);
	    dt.End();
		std::stringstream ss;
		//ss<<" cx:"<<ovx<<" cw:"<<ow<<std::endl;
		//ss<<"chassis do time dt:"<<(dt.GetTime() / 1e6)<<" sx:"<<x<<" sa:"<<a<<" cx:"<<ovx<<" cw:"<<ow<<" odox:"<<ox<<" odoy:"<<oy<<" odoa:"<<oa;
		ss<<" sx:"<<x<<" sa:"<<a<<" odox:"<<ox<<" odoy:"<<oy<<" odoa:"<<oa;
		//std::cout<<ss.str()<<std::endl;

		LOGS_DEBUG("chassis")<<ss.str();

	}

	if(bchassis){

		bchassis->stop();

		SDelete(bchassis);
	}

	return 0;
}
