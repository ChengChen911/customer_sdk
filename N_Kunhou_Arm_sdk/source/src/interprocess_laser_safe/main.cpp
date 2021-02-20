/*
 * main.cpp
 *  Describe :  interprocess_laser_safe  main.cpp
 *  Created on: Dec 21, 2017
 *      Author: neo
 */


#include <signal.h>


#include <iostream>
#include <fstream>

#include <string>
#include <cassert>
#include "TimerDiff.h"

#include "interprocess_core/reg_msg.h"
#include "interprocess/shared_data.hpp"
#include "interprocess/shared_pool.hpp"

#include "mapserver/MapServer.h"

#include "robot/angle_calibration.h"
#include "polygon_ex.h"
#include "Obstacle_Detector.h"

using namespace NS_Laser_Safe;


bool brun = true;
int show_lasersafe = 1;
bool use_manual_frame = true; //false: auto ; true : manual

U32 obstacle_min_ = 10;      //框内光点 阀值
F32 rotation_angle_ = 0;   //弧线时框体旋转角度
U32 send_count = 3;         //检测结果变化时，写入prohibitmap次数
int tim310_send_count =3;

Obstacle_Detector obstacle_finder;
Obstacle_Status last_pause = Obstacle_Status::NO_OBSTACLE;   //记录上一次障碍物检测结果

Obstacle_Status last_310_pause = Obstacle_Status::NO_OBSTACLE;   //记录上一次310障碍物检测结果

bool last_collision = false;

int used_laser_ = 0;
bool use_laser_amcl = false;
bool use_laser_tf1 = false;
bool use_laser_tf2 = false;
bool use_photosensor = false;

SLaser_para amcl_para;
SLaser_para tf1_para;
SLaser_para tf2_para;

Laser_safe amcl_;
Laser_safe tf1_;
Laser_safe tf2_;

SLaserSafe_Info amcl_info_;
SLaserSafe_Info tf1_info_;
SLaserSafe_Info tf2_info_;


///****///
//特定功能：    导航激光遮挡保护
//描述：   设置 一个框，框内scan足够多时认为导航激光被遮挡，停车
bool use_laserscan_protector_ = false;
Laser_safe  laser_stop_;
bool laser_safe_scan_stop  = false;
int unstop_ = 0;
std::string str_safe_shape_straight_stop = "2.5:-0.4;2.5:0.4;-0.4:0.4;-0.4:-0.4;2.5:-0.4";
///end///


polygon_ex amcl_polygon_ex_;  //计算激光 min_range
polygon_ex tf1_polygon_ex_;  //计算激光 min_range
polygon_ex tf2_polygon_ex_;  //计算激光 min_range

bool b_first_run_amcl = true;
bool b_first_run_tf1 = true;
bool b_first_run_tf2 = true;

bool b_first_init_amcl = true;
bool b_first_init_tf1 = true;
bool b_first_init_tf2 = true;
int inited_laser_cnt = 0;

SLaser get_amcl_laser_data;
SLaser get_tf1_laser_data;
SLaser get_tf2_laser_data;

boost::mutex mu_setspeed;
boost::mutex mu_dio;
boost::mutex mu_laser_amcl;
boost::mutex mu_laser_tf1;
boost::mutex mu_laser_tf2;
boost::mutex mu_srun;


SSpeed get_speed;
SDI Sdi;

//laser_safe_shape Parameters
F32 x_redu1 = 0.8,x_stop = 0.4,y_redu1 = 0.2,y_stop = 0.05;
F32 x_redu2 = 0.6,x_buff = 0.6,y_redu2 = 0.2,y_buff = 0.1;

Shape_xy robot_shape;
//tim 310
NS_Laser_Safe::SHAPE_DIR cur_dir_ ;
int tim310_res = 0;
bool use_tim_310 = false;
boost::mutex mu_tim310;
bool use_collisionbar = false;

int dynamic_area_code = 0;

//手动输入框体顶点坐标, exp robot_shape: "1.62:-0.5;1.62:0.5;-0.2:0.5;-0.2:-0.5;1.62:-0.5;"
std::string test_shapestring = "1.0:-0.5;1.0:-0.1;-1.0:0.5;-1.0:0.1;1.0:-0.5";

void set_pause_continue( Obstacle_Status &bflag);
void amcl_laser_first(const SLaser &used_laser_data, Laser_safe &laser_safe, SLaser_para &laser_para);



void get_laser_para(const std::string pub_str,SLaser_para &para){

	para.laser_ip_ = "0.0.0.0";
	para.reverse_ = false;
	para.laser_start_angle_ = -135;
	para.laser_range_max_ = 50.0;
	para.laser_range_min_ = 0.03;
	para.laser_dx_ = 0;
	para.laser_dy_ = 0;

	if( pub_str == LASER_TF_1_STR ){
		Config::getConfig("laser_tf1_ip",para.laser_ip_);
		Config::getConfig("laser_tf1_reverse",para.reverse_);
		Config::getConfig("laser_tf1_start_angle",para.laser_start_angle_);
		Config::getConfig("laser_tf1_range_max",para.laser_range_max_);
		Config::getConfig("laser_tf1_range_min",para.laser_range_min_);
		Config::getConfig("laser_tf1_dx",para.laser_dx_);
		Config::getConfig("laser_tf1_dy",para.laser_dy_);
	}else if( pub_str == LASER_TF_2_STR ){
		Config::getConfig("laser_tf2_ip",para.laser_ip_);
		Config::getConfig("laser_tf2_reverse",para.reverse_);
		Config::getConfig("laser_tf2_start_angle",para.laser_start_angle_);
		Config::getConfig("laser_tf2_range_max",para.laser_range_max_);
		Config::getConfig("laser_tf2_range_min",para.laser_range_min_);
		Config::getConfig("laser_tf2_dx",para.laser_dx_);
		Config::getConfig("laser_tf2_dy",para.laser_dy_);
	}else{
		Config::getConfig("laser_ip",para.laser_ip_);
		Config::getConfig("laser_reverse",para.reverse_);
		Config::getConfig("laser_start_angle",para.laser_start_angle_);
		Config::getConfig("laser_range_max",para.laser_range_max_);
		Config::getConfig("laser_range_min",para.laser_range_min_);
		Config::getConfig("laser_dx",para.laser_dx_);
		Config::getConfig("laser_dy",para.laser_dy_);
	}
	std::cout<<"laser name :  >>>  "<<pub_str<<std::endl;
	std::cout<<"laser ip:"<<para.laser_ip_<<std::endl;
	std::cout<<"laser reverse_:"<<para.reverse_<<std::endl;
	std::cout<<"laser_start_angle_:"<<para.laser_start_angle_<<std::endl;
	std::cout<<"laser_range_max_:"<<para.laser_range_max_<<std::endl;
	std::cout<<"laser_range_min_:"<<para.laser_range_min_<<std::endl;
	std::cout<<"laser_dx_:"<<para.laser_dx_<<std::endl;
	std::cout<<"laser_dy_:"<<para.laser_dy_<<std::endl;

}

void get_robot_shape(U8 i, std::string value1 , std::string value2)
{
	switch(i){
		case 1:
			cComm::ConvertToNum(robot_shape.x1_,value1);
			cComm::ConvertToNum(robot_shape.y1_,value2);
			//cComm::RangeIt(x1,F32(0.0),F32(1.5));
			std::cout<<"x1:"<<robot_shape.x1_<<"  y1:"<<robot_shape.y1_<<std::endl;
			break;
		case 2:
			cComm::ConvertToNum(robot_shape.x2_,value1);
			cComm::ConvertToNum(robot_shape.y2_,value2);
			//cComm::RangeIt(x1,F32(0.0),F32(1.5));
			std::cout<<"x2:"<<robot_shape.x2_<<"  y2:"<<robot_shape.y2_<<std::endl;
			break;
		case 3:
			cComm::ConvertToNum(robot_shape.x3_,value1);
			cComm::ConvertToNum(robot_shape.y3_,value2);
			//cComm::RangeIt(x1,F32(0.0),F32(1.5));
			std::cout<<"x3:"<<robot_shape.x3_<<"  y3:"<<robot_shape.y3_<<std::endl;
			break;
		case 4:
			cComm::ConvertToNum(robot_shape.x4_,value1);
			cComm::ConvertToNum(robot_shape.y4_,value2);
			//cComm::RangeIt(x1,F32(0.0),F32(1.5));
			std::cout<<"x4:"<<robot_shape.x4_<<"  y4:"<<robot_shape.y4_<<std::endl;
			break;
	}
}

//  --从网页中的距离值，计算所有避障框体的定点坐标
void get_laser_shape_dis(std::string name , std::string value)
{
	if(name == "x_redu1"){
		cComm::ConvertToNum(x_redu1,value);
		cComm::RangeIt(x_redu1,F32(0.0),F32(3));
		std::cout<<"x_redu1:"<<x_redu1<<std::endl;
	}else if(name == "x_redu2"){
		cComm::ConvertToNum(x_redu2,value);
		cComm::RangeIt(x_redu2,F32(0.0),F32(3));
		std::cout<<"x_redu2:"<<x_redu2<<std::endl;
	}else if(name == "x_buff"){
		cComm::ConvertToNum(x_buff,value);
		cComm::RangeIt(x_buff,F32(0.0),F32(3));
		std::cout<<"x_buff:"<<x_buff<<std::endl;
	}else if(name == "x_stop"){
		cComm::ConvertToNum(x_stop,value);
		cComm::RangeIt(x_stop,F32(0.0),F32(3));
		std::cout<<"x_stop:"<<x_stop<<std::endl;
	}else if(name == "y_redu1"){
		cComm::ConvertToNum(y_redu1,value);
		cComm::RangeIt(y_redu1,F32(0.0),F32(3));
		std::cout<<"y_redu1:"<<y_redu1<<std::endl;
	}else if(name == "y_redu2"){
		cComm::ConvertToNum(y_redu2,value);
		cComm::RangeIt(y_redu2,F32(0.0),F32(3));
		std::cout<<"y_redu2:"<<y_redu2<<std::endl;
	}else if(name == "y_stop"){
		cComm::ConvertToNum(y_stop,value);
		cComm::RangeIt(y_stop,F32(0.0),F32(3));
		std::cout<<"y_stop:"<<y_stop<<std::endl;
	}else if(name == "y_buff"){
		cComm::ConvertToNum(y_buff,value);
		cComm::RangeIt(y_buff,F32(0.0),F32(3));
		std::cout<<"y_buff:"<<y_buff<<std::endl;
	}else if(name == "rotation_angle"){
		cComm::ConvertToNum(rotation_angle_,value);
		cComm::RangeIt(rotation_angle_,F32(0.0),F32(90));
		std::cout<<"rotation_angle:"<<rotation_angle_<<std::endl;
	}else{
		std::cout<<"err name:"<<name<<" para:"<<value<<std::endl;
	}
}

void string_to_direction(std::string locate,Direction& dir)
{
	if(locate=="back"){
		dir  = Direction::BACK;
		std::cout<<">> back"<<std::endl;
	}else if(locate =="left"){
		dir  = Direction::LEFT;
		std::cout<<">> left"<<std::endl;
	}else if(locate =="right"){
		dir  = Direction::RIGHT;
		std::cout<<">> right"<<std::endl;
	}else if(locate =="front"){
		dir  = Direction::FRONT;
		std::cout<<">>front"<<std::endl;
	}
}
void get_laser_location(std::string name , std::string locate)
{
	if (name  == "laser")
	{
		used_laser_++;
		use_laser_amcl = true;
		Direction direct;
		string_to_direction(locate,direct);
		amcl_.laser_location_ = direct;
		std::cout<<"laser inf used: amcl laser"<<std::endl;
	}else if(name == LASER_TF_1_STR){
		used_laser_++;
		use_laser_tf1 = true;
		Direction direct;
		string_to_direction(locate,direct);
		tf1_.laser_location_ = direct;
		std::cout<<"laser inf used: tf1 laser,dir: "<<direct<<std::endl;
	}else if(name == LASER_TF_2_STR){
		used_laser_++;
		use_laser_tf2 = true;
		Direction direct;
		string_to_direction(locate,direct);
		tf2_.laser_location_ = direct;
		std::cout<<"laser inf used: tf2 laser dir: "<<direct<<std::endl;
	}else if(name == "photosensor"){
		if(locate == "true"){
			use_photosensor = true;
			obstacle_finder.usephotosensor(true);
			std::cout<<"use photosensor"<<std::endl;
		}

	}else if(name == "scan_protect"){
			use_laserscan_protector_ = true;
			std::cout<<"use use_laserscan_protector_"<<std::endl;
	}else if(name == "tim310"){
		if(locate == "true"){
			use_tim_310 = true;
			std::cout<<"use tim310!!"<<std::endl;
		}

	}else if(name == "use_collisionbar"){
		if( locate== "true"){
			use_collisionbar = true;
			std::cout<<"use_collisionbar!!"<<std::endl;
		}

	}

}

void getConfigPara()
{
	std::cout<<"Start Config Para!***"<<std::endl;

	std::string robot_shape = "1.2:-0.25;1.2:0.25;-1.2:0.25;-1.2:-0.25;1.2:-0.25";
	Config::getConfig("robot_shape",robot_shape);
	std::vector<std::string> robot_sh;
	cComm::SplitString(robot_shape,";",robot_sh);
	std::vector<std::string>::iterator it_r = robot_sh.begin();
	U8 i=1;
	for ( ; it_r != robot_sh.end() ; ++it_r)
	{
		std::string &para_pair = *it_r;
		if (para_pair.length())
		{
			std::vector<std::string> vpara2;
			cComm::SplitString(para_pair,":",vpara2);
			if (vpara2.size() > 1)
			{
				get_robot_shape(i, vpara2[0], vpara2[1]);
			}
		}
		i++;
	}

	std::string laser_shape = "x_redu1:0.8;x_redu2:0.6;x_buff:0.6;x_stop:0.5;y_redu1:0.0;y_redu2:0.0;y_buff:0.0;y_stop:0.0;rotation_angle:0";
	Config::getConfig("laser_shape_distance",laser_shape);
	std::vector<std::string> laser_sh;
	cComm::SplitString(laser_shape,";",laser_sh);
	std::vector<std::string>::iterator it_l = laser_sh.begin();
	for ( ; it_l != laser_sh.end() ; ++it_l)
	{
		std::string &para_pair = *it_l;
		if (para_pair.length())
		{
			std::vector<std::string> vpara2;
			cComm::SplitString(para_pair,":",vpara2);
			if (vpara2.size() > 1)
			{
				get_laser_shape_dis(vpara2[0],vpara2[1]);
			}
		}
	}

	std::string str_laser_inf_msg = "laser";
	Config::getConfig("laser_inf_msg",str_laser_inf_msg);
	std::cout<<"laser_inf_msg:"<<str_laser_inf_msg<<std::endl;
	std::vector<std::string> vmsg;
	cComm::SplitString(str_laser_inf_msg,";",vmsg);
	std::vector<std::string>::iterator it = vmsg.begin();
	for ( ; it != vmsg.end() ; ++it)
	{
		std::string &para_pair = *it;
		if (para_pair.length())
		{
			std::vector<std::string> vpara2;
			cComm::SplitString(para_pair,":",vpara2);
			if (vpara2.size() > 1)
			{
				get_laser_location(vpara2[0],vpara2[1]);
			}
		}
	}

	get_laser_para("laser",amcl_para);
	get_laser_para(LASER_TF_1_STR,tf1_para);
	get_laser_para(LASER_TF_2_STR,tf2_para);

	Config::getConfig("obstacle_min",obstacle_min_);
	obstacle_finder.setPara(obstacle_min_);

	Config::getConfig("show_lasersafe",show_lasersafe);
	//----- Test  ----  ////
	//------------------////

	std::cout<<"Finish Config Para!***"<<std::endl;
}

void shutdown(int sig)
{
	std::cout<<"ctrl c shut down"<<std::endl;
	brun = false;
	shared_pool::destructor();
	SLEEP(500);

	return;
}

void update_amcl_laser(const SLaser &used_laser_data){
	boost::mutex::scoped_lock lock(mu_laser_amcl);
	if (b_first_run_amcl)
	{
		get_amcl_laser_data = used_laser_data;
		b_first_run_amcl = false;
		std::cout<<"call back first run ,init amcl_laser range."<<std::endl;
		if(use_laserscan_protector_){
			amcl_laser_first(used_laser_data,laser_stop_,amcl_para);
		}

	}else{
		amcl_.laser_data_ = used_laser_data;
		if(use_laserscan_protector_){
			laser_stop_.laser_data_ = used_laser_data;
		}

	}
}
void update_tf1_laser(const SLaser &used_laser_data){
	boost::mutex::scoped_lock lock(mu_laser_tf1);
	if (b_first_run_tf1)
	{
		get_tf1_laser_data = used_laser_data;
		b_first_run_tf1 = false;
		std::cout<<"call back first run ,init tf1 range."<<std::endl;
	}else{

	tf1_.laser_data_ = used_laser_data;

	}
}
void update_tf2_laser(const SLaser &used_laser_data){
	boost::mutex::scoped_lock lock(mu_laser_tf2);
	if(b_first_run_tf2 )
	{
		get_tf2_laser_data = used_laser_data;
		b_first_run_tf2 = false;
		std::cout<<"call back first run ,init tf2 range."<<std::endl;
	}else{
		tf2_.laser_data_ = used_laser_data;
	}

}

void callback(const sclose &cl){

	if (cl.over)
	{
		std::cout<<"core shut down:"<<shared_pool::name()<<std::endl;
		shutdown(1);
	}

}

void callSpeed(const SSpeed &speed)
{
	boost::mutex::scoped_lock lock(mu_setspeed);
	get_speed = speed;
}

void calldio(const SDI &sdi)
{
	boost::mutex::scoped_lock lock(mu_dio);
	Sdi = sdi;
}
void update_amcl_entropy(const SAMCL_RES &res){
//	std::cout<<"res.entropy_:"<<res.entropy_<<std::endl;
	F32 f_safe_entropy = 1000;
	Config::getConfig("safe_entropy",f_safe_entropy);
//	std::cout<<"f_safe_entropy:"<<f_safe_entropy<<std::endl;

	if( res.entropy_ > f_safe_entropy ){
		F32 vx = 0;
		F32 vw = 0;
		{
			boost::mutex::scoped_lock lock(mu_setspeed);
			vx = get_speed.vx_;
			vw = get_speed.vw_;
		}
		if( (fabs(vx) > 0.001) || (fabs(vw) > 0.001)){

			U8 data[] = {1};
			U32 len = 1;

//			shared_pool::Shared_Pipe_Push( PROHIBIT_MAP ,"layer1",data,len);
//			std::cout<<"update_amcl_entropy set pause!"<<std::endl;
//			Obstacle_Status status_comb = Obstacle_Status::OBSTACLE_STOP;
//			set_pause_continue(status_comb);
		}

	}
}
void calltim310(const int &val)
{
	boost::mutex::scoped_lock lock(mu_tim310);
	tim310_res = val;
	//std::cout<<" callback  value :"<<tim310_res<<std::endl;
}

void calldynamicfnc(const dynamic_area_fnc &dynamicfnc)
{
	//std::cout<<"call dynamic fnc ,val:"<<(int)dynamicfnc.function_code_<<std::endl;
	if(dynamicfnc.function_code_ == 1){
		dynamic_area_code = 1;
	}else if(dynamicfnc.function_code_ == 0)
		dynamic_area_code = 0;
	;
}

void init_shared_pool(char *argv[]){

	shared_pool::init(argv);
	boost::function<void( const sclose &cl)> fnc_shutdown;
	fnc_shutdown = boost::bind(callback,_1);
	shared_pool::Subscribe(shared_pool::name(),"shutdown",fnc_shutdown);

	boost::function<void( const SSpeed &speed )> fnc_SetSpeed;
	fnc_SetSpeed = boost::bind(callSpeed,_1);
	shared_pool::Subscribe(shared_pool::name(),"setspeed",fnc_SetSpeed);

	boost::function<void( const SDI &sdi )> fnc_SDI;
	fnc_SDI = boost::bind(calldio,_1);
	shared_pool::Subscribe(shared_pool::name(),"gpio_di",fnc_SDI);

	boost::function<void( const SRunStatus &runstate )> fnc_runstate;
	fnc_runstate = boost::bind(&Obstacle_Detector::UpdateRunStatus,&obstacle_finder,_1);
	shared_pool::Subscribe(shared_pool::name(),"run_status",fnc_runstate);

	//subscribe laser amcl data
	if (use_laser_amcl)
	{
		boost::function<void( const SLaser &laser_data )> fnc_amcl;
		fnc_amcl = boost::bind(update_amcl_laser,_1);
		shared_pool::Subscribe(shared_pool::name(),"laser",fnc_amcl);
	}

	//subscribe laser tf1 data
	if(use_laser_tf1){
		boost::function<void( const SLaser &laser_data )> fnc_tf1;
		fnc_tf1 = boost::bind(update_tf1_laser,_1);
		shared_pool::Subscribe(shared_pool::name(),LASER_TF_1_STR,fnc_tf1);
	}

	//subscribe laser tf2 data
	if(use_laser_tf2){
		boost::function<void( const SLaser &laser_data )> fnc_tf2;
		fnc_tf2 = boost::bind(update_tf2_laser,_1);
		shared_pool::Subscribe(shared_pool::name(),LASER_TF_2_STR,fnc_tf2);
	}

	boost::function<void( const SAMCL_RES &pos )> fnc_amcl_res;
	fnc_amcl_res = boost::bind(update_amcl_entropy,_1);
	shared_pool::Subscribe(shared_pool::name(),"amcl_entropy",fnc_amcl_res);

	boost::function<void( const int &sdi )> fnc_tim310;
	fnc_tim310 = boost::bind(calltim310,_1);
	shared_pool::Subscribe(shared_pool::name(),"tim310_value",fnc_tim310);

	boost::function<void( const dynamic_area_fnc &speed )> fnc_dynamic_fnc;
	fnc_dynamic_fnc = boost::bind(calldynamicfnc,_1);
	shared_pool::Subscribe(shared_pool::name(),"set_dynamic_lasersafe",fnc_dynamic_fnc);
}

bool check_status(Obstacle_Status st,SLaserSafe_Info amcl,SLaserSafe_Info tf1,SLaserSafe_Info tf2,SLaserSafe_Info& last_info){

	Obstacle_Status s1 = (Obstacle_Status)amcl.check_result_;
	Obstacle_Status s2 = (Obstacle_Status)tf1.check_result_;
	Obstacle_Status s3 = (Obstacle_Status)tf2.check_result_;

	if((st==s1)&&use_laser_amcl){
		last_info = amcl_info_;
		//std::cout<<"pub info  = amcl_info"<<std::endl;
		return true;
	}
	if((st==s2)&&use_laser_tf1){
		last_info = tf1_info_;
		//std::cout<<"pub info  = tf1_info,id: "<<tf1_info_.laser_id_<<std::endl;
		return true;
	}
	if((st==s3)&&use_laser_tf2){
		last_info = tf2_info_;
		//std::cout<<"pub info  = tf2_infoid: "<<tf2_info_.laser_id_<<std::endl;
		return true;
	}
	//std::cout<<"st: "<<st<<" s1: "<<s1<<" s2:"<<s2<<" s3: "<<s3<<std::endl;
	//std::cout<<"  aa pub info  = tf1_info,id: "<<tf1_info_.laser_id_<<std::endl;
	last_info = tf1_info_;
	//std::cout<<"pub info  = tf1_info     else"<<std::endl;
	return false;

}
bool check_status(Obstacle_Status st,Obstacle_Status s1,Obstacle_Status s2,Obstacle_Status s3,SLaserSafe_Info& last_info){
	last_info = amcl_info_;

	if((st==s1)){
		return true;
	}
	if(st==s2){
		last_info = tf1_info_;
		return true;
	}
	if(st==s3){
		last_info = tf2_info_;
		return true;
	}
	//std::cout<<"st: "<<st<<" s1: "<<s1<<" s2:"<<s2<<" s3: "<<s3<<std::endl;
	return false;

}

void set_pause_continue( Obstacle_Status &bflag){

	if ( last_pause != bflag )
	{
		send_count = 3;
		if(last_pause == Obstacle_Status::OBSTACLE_STOP){
			if(unstop_>2){
				unstop_=0;
				last_pause = bflag;
			}else{
				bflag = last_pause;
			}
			unstop_ ++;
		}else{
			last_pause = bflag;
		}
	}

	if ( send_count > 0)
	{
		send_count--;
	}else
	{
		return;
	}

	U8 data[] = {5};
	U32 len = 1;

	data[0] = bflag;

	std::cout<<"laser_safe set_status: "<<(int)data[0]<<std::endl;
	shared_pool::Shared_Pipe_Push( PROHIBIT_MAP ,"layer1",data,len);
	if(bflag == Obstacle_Status::OBSTACLE_STOP){
		SLEEP(500);
	}
//	Srecoder pause;
//	memset(pause.cmd_file_,0,200);
//	memcpy(pause.cmd_file_,str.c_str(),str.length() );
//	shared_pool::Publish(shared_pool::name(),"pause",pause);

}

void set_310_pause_continue( Obstacle_Status &bflag){
//std::cout<<"last_310_pause:"<<last_310_pause<<" bflag:"<<(int)bflag<<std::endl;
	if ( last_310_pause != bflag )
	{
		tim310_send_count  = 3;
		if(last_310_pause == Obstacle_Status::OBSTACLE_STOP){
			if(unstop_>2){
				unstop_=0;
				last_310_pause = bflag;
			}else{
				bflag = last_310_pause;
			}
			unstop_ ++;
		}else{
			last_310_pause = bflag;
		}
	}

	if ( tim310_send_count  > 0)
	{
		tim310_send_count--;
	}else
	{
		return;
	}

	U8 data[] = {5};
	U32 len = 1;

	data[0] = bflag;

	std::cout<<"laser_safe 310 set_status: "<<(int)data[0]<<std::endl;
	shared_pool::Shared_Pipe_Push( PROHIBIT_MAP ,"layer1",data,len);
	if(bflag == Obstacle_Status::OBSTACLE_STOP){
		SLEEP(500);
	}
}


int get_id_value(int id,SDI din)
{

	int val = 0;
	for(int i=0;i<din.used_;i++)
	{
		if(id == din.id_[i]){
			val = din.di_[i];
			return val;
		}
	}

	return val;
}

void check_collision_stop()
{
	//return ;
	boost::mutex::scoped_lock lock(mu_dio);
	//int collision_di = Sdi.di_[229-222];
	int collision_di = get_id_value(11,Sdi); // kejia agv 12
	//std::cout<<" din 11 :"<<collision_di<<std::endl;
	if(collision_di == 1){
		last_collision = false;
		return;
	}

	if(last_collision == true)    //写过prohibit map，避免重复写
		return ;

	U8 data[] = {8};
	U32 len = 1;
	std::cout<<" din 11 :"<<collision_di<<std::endl;
//	std::cout<<"collision   set_status: "<<(int)data[0]<<std::endl;
	shared_pool::Shared_Pipe_Push( PROHIBIT_MAP ,"layer1",data,len);
	SLEEP(300);
	last_collision = true;
}

int main(int argc, char *argv[])
{
	std::cout<<"Navikit Program Version: V1.4.5"<<std::endl;
	bool b_test = false;

	::signal(SIGINT, shutdown);

	getConfigPara();

	//init shared pool
	if(argc > 1){

		init_shared_pool(argv);

	}
	//SLEEP(100);

	cTimerDiff dt;

	Obstacle_Status status_comb = Obstacle_Status::NO_OBSTACLE;

	Obstacle_Status status_amcl = Obstacle_Status::NO_OBSTACLE;
	Obstacle_Status status_tf1 = Obstacle_Status::NO_OBSTACLE;
	Obstacle_Status status_tf2 = Obstacle_Status::NO_OBSTACLE;

	SLaser active_frame;
	SLaserSafe_Info pub_info;
	U8 active_frameid = 0;

	amcl_info_.laser_id_ = 0;
	amcl_info_.check_result_ = (U8)Obstacle_Status::NO_OBSTACLE;
	amcl_info_.active_frame_id_ = 0;
	tf1_info_.laser_id_ = 1;
	tf1_info_.check_result_ = (U8)Obstacle_Status::NO_OBSTACLE;
	tf1_info_.active_frame_id_ = 0;
	tf2_info_.laser_id_ = 2;
	tf2_info_.check_result_ = (U8)Obstacle_Status::NO_OBSTACLE;
	tf2_info_.active_frame_id_ = 0;
	if(use_manual_frame){
		obstacle_finder.LoadFrames();
		std::cout<<"***** use Manual frame !!!******"<<std::endl;
	}else{
		obstacle_finder.setFrames(robot_shape,x_redu1,x_redu2,x_buff,x_stop,y_redu1,y_redu2,y_buff,y_stop,rotation_angle_);
		std::cout<<"***** use Auto  frame !!!******"<<std::endl;
	}

	bool init_lasershape = false;
	int cnt = 0;

	//frame  test
//	get_tf1_laser_data.resolution_ = Deg2Rad(0.5);
//	get_tf1_laser_data.start_angle_ =Deg2Rad(-45) ;
//	get_tf1_laser_data.range_max_ = 20;
//	obstacle_finder.init_lasersafe(get_tf1_laser_data,tf1_,tf1_para);
	///end//
	SLaserSafe_Frames frame;
	shared_pool::Publish(shared_pool::name(),"tim310_dir",(int)NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT);
	while(brun){
		if(init_lasershape== false)
		{
			{
				boost::mutex::scoped_lock lock(mu_laser_amcl);
				if(b_first_run_amcl == false &&b_first_init_amcl ==true){
					obstacle_finder.init_lasersafe(get_amcl_laser_data,amcl_,amcl_para,frame);
					frame.laser_id_ = 0;
					inited_laser_cnt++;
					b_first_init_amcl = false;
					shared_pool::Publish(shared_pool::name(),"display_frames",frame);
					cnt++;
				}
			}

			{
				boost::mutex::scoped_lock lock(mu_laser_tf1);
				if(b_first_run_tf1 == false &&b_first_init_tf1 ==true){
					obstacle_finder.init_lasersafe(get_tf1_laser_data,tf1_,tf1_para,frame);
					frame.laser_id_ = 1;
					inited_laser_cnt++;
					b_first_init_tf1 = false;
					shared_pool::Publish(shared_pool::name(),"display_frames",frame);
					cnt++;
				}
			}
			{
				boost::mutex::scoped_lock lock(mu_laser_tf2);
				if(b_first_run_tf2 == false &&b_first_init_tf2 ==true){
					//std::cout<<" tf2 para,  x: "<<tf2_para.laser_dx_<<" y: "<<tf2_para.laser_dy_<<std::endl;
					obstacle_finder.init_lasersafe(get_tf2_laser_data,tf2_,tf2_para,frame);
					frame.laser_id_ = 2;
					inited_laser_cnt++;
					b_first_init_tf2 = false;
					shared_pool::Publish(shared_pool::name(),"display_frames",frame);
					cnt++;
				}
			}
			if(inited_laser_cnt>=used_laser_){
				init_lasershape = true;
				std::cout<<"*********Initialize Done ,** used laser count: "<<cnt<<std::endl;
			}
			SLEEP(100);

		}
		else
		{
				F32 vx,vy,vw;
				{
					boost::mutex::scoped_lock lock(mu_setspeed);
					vx = get_speed.vx_;
					vy = get_speed.vy_;
					vw = get_speed.vw_;
				}

				if(use_photosensor){
					boost::mutex::scoped_lock lock(mu_dio);
					obstacle_finder.SetDI(Sdi);
				}
				if ((use_laser_amcl)&&(b_first_run_amcl == false))
				{
					boost::mutex::scoped_lock lock(mu_laser_amcl);
					obstacle_finder.check_Obstacle(vx,vy,vw,amcl_,status_amcl,active_frameid);
					amcl_info_.check_result_ = status_amcl;
					amcl_info_.active_frame_id_= active_frameid;
					amcl_info_.laser_scan_ = amcl_.laser_data_;
					//std::cout<<">>>>>Amcl status<<<: "<<(int)status_amcl<<std::endl;

				}
				if ((use_laser_tf1)&&(b_first_run_tf1==false))
				{
					boost::mutex::scoped_lock lock(mu_laser_tf1);
					obstacle_finder.check_Obstacle(vx,vy,vw,tf1_,status_tf1,active_frameid);
					tf1_info_.check_result_ = status_tf1;
					tf1_info_.active_frame_id_ = active_frameid;
					tf1_info_.laser_scan_ = tf1_.laser_data_;
					//std::cout<<">>>>>Tf1 status<<<<<<<<: "<<(int)status_tf1<<std::endl;
				}
				if ((use_laser_tf2)&&(b_first_run_tf2==false))
				{
					boost::mutex::scoped_lock lock(mu_laser_tf2);
					obstacle_finder.check_Obstacle(vx,vy,vw,tf2_,status_tf2,active_frameid);
					tf2_info_.check_result_ = status_tf2;
					tf2_info_.active_frame_id_ = active_frameid;
					tf2_info_.laser_scan_ = tf2_.laser_data_;
					//std::cout<<"Tf2 status: "<<(int)status_tf2<<std::endl;
				}

				if(check_status(Obstacle_Status::OBSTACLE_STOP,amcl_info_,tf1_info_,tf2_info_,pub_info)){
					status_comb = Obstacle_Status::OBSTACLE_STOP;
				}else if(check_status(Obstacle_Status::OBSTACLE_REDUCE2,amcl_info_,tf1_info_,tf2_info_,pub_info)){
					status_comb = Obstacle_Status::OBSTACLE_REDUCE2;
				}else if(check_status(Obstacle_Status::OBSTACLE_REDUCE1,amcl_info_,tf1_info_,tf2_info_,pub_info)){
					status_comb = Obstacle_Status::OBSTACLE_REDUCE1;
				}else{
					status_comb = Obstacle_Status::NO_OBSTACLE;
					//std::cout<<"status_comb no obstacle status"<<status_tf2<<std::endl;
				}
				if(use_collisionbar)
				  check_collision_stop();

				if( use_tim_310){
					//pub(dir ) cur_dir_ = 8;
					obstacle_finder.tim310_preProcessSpeed(vx,vy,vw,cur_dir_);
					bool photosensor_res = false;
					if(use_photosensor&&(cur_dir_>2)&&(cur_dir_<7)){
						//std::cout<<"cur_dir_:"<<(int)cur_dir_<<std::endl;
						boost::mutex::scoped_lock lock(mu_dio);
						photosensor_res = obstacle_finder.check_data(Sdi);
					}

					shared_pool::Publish(shared_pool::name(),"tim310_dir",(int)cur_dir_);
					//shared_pool::Publish(shared_pool::name(),"tim310_dir",0);
					int res = 0;
					{
						boost::mutex::scoped_lock lock(mu_tim310);
						res = tim310_res;
						//std::cout<<" res  value :"<<tim310_res<<std::endl;
					}
					//std::cout<<"dynamic_area_code :"<<dynamic_area_code<<std::endl;
					Obstacle_Status  tim310_obs_res ;
					if(cur_dir_ == NS_Laser_Safe::SHAPE_DIR::UNKNOWN){
						tim310_obs_res = Obstacle_Status::NO_OBSTACLE;
					}else if(res == 3 ||photosensor_res){//小检测框有物体，有障碍物
						if(dynamic_area_code ==0 )
							tim310_obs_res = Obstacle_Status::OBSTACLE_STOP;
					}else if(res == 2){//中检测框有物体
						tim310_obs_res = Obstacle_Status::OBSTACLE_REDUCE2;
					}else if(res == 1){ //大检测框有物体
						tim310_obs_res = Obstacle_Status::OBSTACLE_REDUCE1;
					}else{
						tim310_obs_res = Obstacle_Status::NO_OBSTACLE;
					}

					set_310_pause_continue(tim310_obs_res);
					if(argc > 1){
						SLaserSafe_Info info;
						info.check_result_ = tim310_obs_res;
						shared_pool::Publish(shared_pool::name(),"update_lasersafe",info);
					}

				}else{
					//std::cout<<"set pause: "<<status_comb<<std::endl;
					set_pause_continue(status_comb);
				}
				if(use_laser_amcl)  //aimosheng
					set_pause_continue(status_comb);

				if(argc > 1){
					if(show_lasersafe){
						//printf("ID :  %d \n",pub_info.laser_id_);
						shared_pool::Publish(shared_pool::name(),"update_lasersafe",pub_info);
					}
				}

#if 1
				//for laserscan only
				if(use_laserscan_protector_){
					NS_Laser_Safe::Obstacle_Status  special_status;
					int tatal_scan = 0;
					int set = 50;
					Config::getConfig("laserscan_threshold",set);
					Obstacle_Detector::s_check_Obstacle(laser_stop_,special_status,set,tatal_scan);
					//std::cout<<"stop_scan :"<<set<<std::endl;
					if(special_status == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP){
						if(laser_safe_scan_stop == false){
							U8 data[] = {1};
							U32 len = 1;
							shared_pool::Shared_Pipe_Push( PROHIBIT_MAP ,"layer1",data,len);
							std::cout<<"special stop  set pause!tatal_scan:  "<<tatal_scan<<std::endl;
							laser_safe_scan_stop  = true;
						}

					}else if((laser_safe_scan_stop  == true)&&(tatal_scan< set-20)){
						U8 data[] = {0};
						U32 len = 1;
						shared_pool::Shared_Pipe_Push( PROHIBIT_MAP ,"layer1",data,len);
						laser_safe_scan_stop = false;
						std::cout<<"special stop  set continue! tatal_scan:"<<tatal_scan<<std::endl;
					}
				}
				/////end///////
#endif

		}

		dt.ms_loop(50);   //20ms loop , 50hz
	}

	return 0;
}
#if 1
void amcl_laser_first(const SLaser &used_laser_data, Laser_safe &laser_safe, SLaser_para &laser_para)
{
	laser_safe.laser_range_straight_stop_ = used_laser_data;

	std::cout<<"Laser Shape polygon with laser dx:"<<laser_para.laser_dx_<<" dy:"<<laser_para.laser_dy_<<std::endl;

	amcl_polygon_ex_.init_shape(str_safe_shape_straight_stop);
	amcl_polygon_ex_.get_range(laser_safe.laser_range_straight_stop_,laser_para.laser_dx_,laser_para.laser_dy_);


}
#endif
