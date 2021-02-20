/*
 * Obstacle_Detector.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: neo
 */

#include "Obstacle_Detector.h"

#include "my_tf.h"
#include <fstream>
#include "Comm.h"
#define LASER_SAFE_FRAMES "lasersafe_frames.txt"


Obstacle_Detector::Obstacle_Detector():	rotation_angle_(20),obstacle_min_(5),use_photosensor_(false),planner_is_idel_(true)
{
	initfnc();
	b_test_ = false;
	current_dir_ =  NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT;
	last_dir_ = current_dir_;
	obstacle_min_ = 10;
	obstacle_warn_ = 0;
	tim310_last_dir_ = NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT;;
}
Obstacle_Detector::~Obstacle_Detector()
{


}
void Obstacle_Detector::setPara(F32 obstacle_min)
{
    if(obstacle_min>0){
    	obstacle_min_ = obstacle_min;
    }
	std::cout<<"obstacle_min_: "<<obstacle_min_<<std::endl;
}
bool Obstacle_Detector::check_Obstacle(const F32 &vx,const F32 &vy, const F32 &vw,
										const NS_Laser_Safe::Laser_safe&laser_safe_data,
										NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{
	//SLaser active_frame;
	active_frameid = 0;

	preProcessSpeed(vx,vy,vw, current_dir_);   //vx vy 判断AGV方向

	//step1:  planner is idel  ?
	{
		boost::mutex::scoped_lock lock(mu_run_state);
		//if(0){
		if(planner_is_idel_ == true){
			//std::cout<<" planner_is_idel_,  Do not check ！！！"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
			//active_frame = laser_safe_data.laser_range_straight_stop_;
			current_state_ = cs;
			last_state_ =  current_state_;
			return true;
		}
	}
	//step2:  current_direction  is unkown ?
	if(current_dir_ == NS_Laser_Safe::SHAPE_DIR::UNKNOWN){
		//std::cout<<" direction unknown ,  Do not check ！！！"<<std::endl;
		cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
		//active_frame = laser_safe_data.laser_range_straight_stop_;
		current_state_ = cs;
		last_state_ =  current_state_;
		return true;
	}

	//step3:  compare motion_direction with laser_location ?
	bool same_direction = false;
	if(current_dir_<3){
		if(laser_safe_data.laser_location_ == Direction::FRONT){
			same_direction = true;
		}
	}else if(current_dir_>2&&current_dir_<6){
		if(laser_safe_data.laser_location_ == Direction::BACK){
			same_direction = true;
		}
	}else if(current_dir_ ==NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT){
		//if(laser_safe_info.laser_location_ == Direction::LEFT){
			same_direction = true;
		//}
	}else if(current_dir_ ==NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT){
		//if(laser_safe_info.laser_location_ == Direction::RIGHT){
			same_direction = true;
		//}
	}


	same_direction = true;
	//step4:  check laser_data ?
	if(same_direction){

		if(obstacle_warn_>0)
		{
			std::cout<<"obstacle_warn_ ,current_dir_: "<<current_dir_<<std::endl;
			obstacle_warn_--;
		}
		    //current_dir_ = NS_Laser_Safe::SHAPE_DIR::UNKNOWN; // bug test
		Obstacle_Detector::CALL checkCall;

		if(current_dir_>=0 && current_dir_ <9)
		{
			try
			{
				 checkCall = check_Calls_[current_dir_];
			}
			catch (std::exception& e)
			{
				std::cout<<"checkCall assign exception:"<<e.what()<<std::endl;
			};

			try
			{
				checkCall(laser_safe_data,cs,active_frameid);
			}
			catch (std::exception& e)
			{
				std::cout<<"checkCall execute exception:"<<e.what()<<std::endl;
			};
		}
		else if(current_dir_ == NS_Laser_Safe::SHAPE_DIR::FAST_FRONT)
		{
			try
			{
				 checkCall = check_Calls_[NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT];
			}
			catch (std::exception& e)
			{
				std::cout<<"checkCall FAST_FRONT assign exception:"<<e.what()<<std::endl;
			};

			try
			{
				checkCall(laser_safe_data,cs,active_frameid);
			}
			catch (std::exception& e)
			{
				std::cout<<"checkCall FAST_FRONT execute exception:"<<e.what()<<std::endl;
			};
		}



//		Obstacle_Detector::CALL checkCall = check_Calls_[current_dir_];
//		checkCall(laser_safe_data,cs,active_frameid);
		//std::cout<<"Check result : "<<int(cs)<<std::endl;
		current_state_ = cs;
		last_state_ =  current_state_;

		if( cs == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
		{
			//save_laser(laser_safe_data.laser_data_,current_dir_);
			obstacle_warn_ = 10;
		}

		return true;
	}else{
		//std::cout<<"motion direction diff with  laser_location  !!! Do not check"<<std::endl;
		cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
		active_frameid = 0;
		current_state_ = cs;
		last_state_ =  current_state_;
	}

	//step5:  use_photosensor?
	if((use_photosensor_ == true)&&(current_dir_>2)&&(current_dir_<6)){
		if(check_data(SDi_)){
			std::cout<<"di find obstacle!"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			return true;
		}
	}

}

void Obstacle_Detector::initfnc()
{
	Obstacle_Detector::CALL fnc;

	// FRONT_STRAIGHT
	fnc = boost::bind(&Obstacle_Detector::check_FrontStraight, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT] = fnc;
	// FRONT_LEFT
	fnc = boost::bind(&Obstacle_Detector::check_FrontLeft, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::FRONT_LEFT] = fnc;
	// FRONT_RIGHT
	fnc = boost::bind(&Obstacle_Detector::check_FrontRight, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::FRONT_RIGHT] = fnc;
	// BACK_STRAIGHT
	fnc = boost::bind(&Obstacle_Detector::check_BackStraight, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::BACK_STRAIGHT] = fnc;
	//BACK_LEFT
	fnc = boost::bind(&Obstacle_Detector::check_BackLeft, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::BACK_LEFT] = fnc;
	// BACK_RIGHT
	fnc = boost::bind(&Obstacle_Detector::check_BackRight, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::BACK_RIGHT] = fnc;
	// LEFT_STRAIGHT
	fnc = boost::bind(&Obstacle_Detector::check_LeftStraight, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT] = fnc;
	// RIGHT_STRAIGHT
	fnc = boost::bind(&Obstacle_Detector::check_RightStraight, this, _1, _2,_3);
	check_Calls_[NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT] = fnc;

}

void Obstacle_Detector::UpdateRunStatus(const SRunStatus&runstate )
{
	boost::mutex::scoped_lock lock(mu_run_state);
	if(runstate.status_ == 0){
		planner_is_idel_ = true;
		current_dir_ = NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT;
		last_dir_ = current_dir_;
		tim310_last_dir_ =  NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT;;
	}else{
		planner_is_idel_ = false;
		//std::cout<<"callback  not idel!"<<std::endl;
	}
}
void Obstacle_Detector::check_FrontStraight(const NS_Laser_Safe::Laser_safe&laser_safe_info,
											NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{//std::cout<<"check frontstraight "<<std::endl;

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_straight_stop_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_straight_redu1_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_straight_redu2_;
	SLaser laser_range_buff = laser_safe_info.laser_range_straight_buff_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{//std::cout<<"last find obstacle stop,,,check frontstraight buffer"<<std::endl;
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"find obstacle in  frontstraight buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 2;
			return ;
		}
		//std::cout<<"check frontstraight buffer res: "<<int(cs)<<std::endl;
	}

		//完整检测
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			//std::cout<<"obstacle in  frontstraight stop"<<std::endl;
			active_frameid = 3;
			return ;
		}

		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			//std::cout<<"obstacle in  frontstraight redu2"<<std::endl;
			active_frameid = 1;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 0;
				//std::cout<<"obstacle in  frontstraight redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 0;
				//active_frame = laser_safe_info.laser_range_right_redu1_;

			}

		}

	//std::cout<<"check frontstraight over res: "<<int(cs)<<std::endl;
}

void Obstacle_Detector::check_FrontLeft(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{
	//std::cout<<"check frontleft "<<std::endl;
	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_left_stop_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_left_redu1_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_left_redu2_;
	SLaser laser_range_buff = laser_safe_info.laser_range_left_buff_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{//std::cout<<"last find obstacle stop,check frontleftbuffer"<<std::endl;
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  frontleft buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 6;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  frontleft stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 7;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 5;
			//std::cout<<"obstacle in  frontleft redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 4;
				//std::cout<<"obstacle in  frontleft redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 4;
			}

		}
	}
	//std::cout<<"check frontleft over res: "<<int(cs)<<std::endl;
}
void Obstacle_Detector::check_FrontRight(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_right_stop_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_right_redu1_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_right_redu2_;
	SLaser laser_range_buff = laser_safe_info.laser_range_right_buff_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  frontright buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 10;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  frontright stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 11;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 9;
			//std::cout<<"obstacle in  frontright redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 8;
				//std::cout<<"obstacle in  frontright redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 8;
			}

		}
	}
	//std::cout<<"check frontright over res: "<<int(cs)<<std::endl;
}
void Obstacle_Detector::check_BackStraight(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_straight_stop_r_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_straight_redu1_r_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_straight_redu2_r_;
	SLaser laser_range_buff = laser_safe_info.laser_range_straight_buff_r_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  backstraight buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 14;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  backstraight stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 15;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 13;
			//std::cout<<"obstacle in  backstraight redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 12;
				//std::cout<<"obstacle in  backstraight redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 12;
			}

		}
	}

	//std::cout<<"check backstraight over res: "<<int(cs)<<std::endl;
}
void Obstacle_Detector::check_BackLeft(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_left_stop_r_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_left_redu1_r_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_left_redu2_r_;
	SLaser laser_range_buff = laser_safe_info.laser_range_left_buff_r_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  backleft buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 18;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  backleft stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 19;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 17;
			//std::cout<<"obstacle in  backleft redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 16;
				//std::cout<<"obstacle in  backleft redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 16;
			}

		}
	}
	//std::cout<<"check backleft over res: "<<int(cs)<<std::endl;
}
void Obstacle_Detector::check_BackRight(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_right_stop_r_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_right_redu1_r_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_right_redu2_r_;
	SLaser laser_range_buff = laser_safe_info.laser_range_right_buff_r_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  backright buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 22;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  backright stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 23;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 21;
			//std::cout<<"obstacle in  backright redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 20;
				//std::cout<<"obstacle in  backright redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 20;
			}

		}
	}
	//std::cout<<"check backright over res: "<<int(cs)<<std::endl;
}

void Obstacle_Detector::check_LeftStraight(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{
	//TODO:

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_left_straight_stop_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_left_straight_redu1_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_left_straight_redu2_;
	SLaser laser_range_buff = laser_safe_info.laser_range_left_straight_buff_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  backright buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 26;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  backright stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 27;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 25;
			//std::cout<<"obstacle in  backright redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 24;
				//std::cout<<"obstacle in  backright redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 24;
			}

		}
	}
}
void Obstacle_Detector::check_RightStraight(const NS_Laser_Safe::Laser_safe&laser_safe_info,
						NS_Laser_Safe::Obstacle_Status &cs, U8& active_frameid)
{
	//TODO:

	SLaser laser_data = laser_safe_info.laser_data_;
	SLaser laser_range_stop = laser_safe_info.laser_range_right_straight_stop_;
	SLaser laser_range_redu1 = laser_safe_info.laser_range_right_straight_redu1_;
	SLaser laser_range_redu2 = laser_safe_info.laser_range_right_straight_redu2_;
	SLaser laser_range_buff = laser_safe_info.laser_range_right_straight_buff_;

	//上次停止框障碍物
	if(last_state_ == NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP)
	{
		if(check_data(laser_data,laser_range_buff)){
			//std::cout<<"obstacle in  backright buffer "<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 30;
			return ;
		}
	}

	//完整检测
	{
		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop)){
			//std::cout<<"find obstacle in  backright stop"<<std::endl;
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			active_frameid = 31;
			return ;
		}
		//检测减速2框 有无障碍物
		if(check_data(laser_data,laser_range_redu2)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE2;
			active_frameid = 29;
			//std::cout<<"obstacle in  backright redu2"<<std::endl;
			return;
		}else{
			//检测减速1框 有无障碍物
			if(check_data(laser_data,laser_range_redu1)){
				cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_REDUCE1;
				active_frameid = 28;
				//std::cout<<"obstacle in  backright redu1"<<std::endl;
			}else{
				cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
				active_frameid = 28;
			}

		}
	}
}

void Obstacle_Detector::preProcessSpeed(const F32 &vx,const F32 &vy, const F32 &vw,NS_Laser_Safe::SHAPE_DIR &dir)
{

	if(vx>0.05){
		dir = NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT;
		if(vw>0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::FRONT_LEFT;
		}else if(vw<-0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::FRONT_RIGHT;
		}
		last_dir_ = dir;
	}else if(vx<-0.05){
		dir = NS_Laser_Safe::SHAPE_DIR::BACK_STRAIGHT;
		if(vw>0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::BACK_LEFT;
		}else if(vw<-0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::BACK_RIGHT;
		}
		last_dir_ = dir;
	}else if(vy>0.05){
		dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
		if(vw>0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
		}else if(vw<-0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
		}
		last_dir_ = dir;
	}else if(vy<-0.05){
		dir = NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT;
		if(vw>0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT;
		}else if(vw<-0.1){
			dir = NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT;
		}
		last_dir_ = dir;
	}else if(fabs(vw)>0.05){
		dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
		last_dir_ = dir;
		std::cout<<"  now   rotation ,vw:"<<vw<<" vx:"<<vx<<std::endl;
	}else{
		dir = last_dir_;
	}
	//std::cout<<"current dir: "<<int(dir)<<std::endl;
}

bool Obstacle_Detector::check_data( SLaser &laser_data, SLaser &laser_range,const F32 angle){

	int obstacle = 0;

	int current_index = 0;

	//std::cout<<"laser_data_.resolution_:"<<laser_data_.resolution_<<" laser_data_.range_min_:"<<laser_data_.range_min_ <<" laser_data_.range_max_:"<<laser_data_.range_max_<<std::endl;
	int diff_index = angle  / laser_data.resolution_;
	//std::cout<<"current angle:"<<angle<<" diff_index:"<<diff_index<<std::endl;

	for (int i = 0  ; i < LASER_COUNT ; ++i )
	{

		current_index = i + diff_index;
		if(current_index >= LASER_COUNT)
			continue;

		if( (current_index < 0) || (current_index >= LASER_COUNT)){
			continue;
		}

		if( laser_data.data_[current_index] < laser_data.range_min_){
			//std::cout<<" laser:"<<laser_data_.data_[current_index]<<" min:"<<laser_data_.range_min_<<std::endl;
			continue;
		}
		if( laser_data.data_[current_index] > laser_data.range_max_){
			continue;
		}

		if (  laser_data.seg_[current_index]   )
		{

			if( laser_data.data_[current_index] < laser_range.data_[i] ){
				obstacle++;
				F32 angle = laser_data.start_angle_ + laser_data.resolution_*i;
				angle = Rad2Deg(angle);
				//std::cout<<"obstacle:"<<obstacle<<" angle:"<<angle<<" dis:"<<laser_data.data_[current_index];
				//std::cout<<" min_dis: "<<laser_range.data_[i]<<std::endl;
			}
		}

	}

	if ( obstacle > obstacle_min_ )
	{
		return true;
	}

	return false;
}

bool Obstacle_Detector::check_data(const SDI di)
{
	S32 di_2=0,di_3=0;
//	di_2 = di.di_[2];  //old
//	di_3 = di.di_[1];

	di_2 = get_id_value(9,di);
	di_3 = get_id_value(10,di);
//	std::cout<<"di2 : "<<di_2<<" di3: "<<di_3<<std::endl;
	if(di_3==0 || di_2==0){
		return true;
	}
	else{
		return false;
	}
}

int Obstacle_Detector::get_id_value(int id,SDI din)
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
void Obstacle_Detector::SetDI(const SDI di)
{
	//std::cout<<"set di"<<std::endl;
	SDi_ = di;
}

NS_Laser_Safe::Obstacle_Status Obstacle_Detector::GetStatus()
{
	return current_state_;
}
void Obstacle_Detector::usephotosensor(bool use)
{
	use_photosensor_ = use;
}


void Obstacle_Detector::init_lasersafe(SLaser &used_laser_data, NS_Laser_Safe::Laser_safe &laser_safe, SLaser_para &laser_para,SLaserSafe_Frames&frames)
{
	laser_safe.laser_range_straight_redu1_ = used_laser_data;
	laser_safe.laser_range_straight_redu2_ = used_laser_data;
	laser_safe.laser_range_straight_buff_ = used_laser_data;
	laser_safe.laser_range_straight_stop_ = used_laser_data;
	laser_safe.laser_range_left_redu1_ = used_laser_data;
	laser_safe.laser_range_left_redu2_ = used_laser_data;
	laser_safe.laser_range_left_buff_ = used_laser_data;
	laser_safe.laser_range_left_stop_ = used_laser_data;
	laser_safe.laser_range_right_redu1_ = used_laser_data;
	laser_safe.laser_range_right_redu2_ = used_laser_data;
	laser_safe.laser_range_right_buff_ = used_laser_data;
	laser_safe.laser_range_right_stop_ = used_laser_data;

	laser_safe.laser_range_straight_redu1_r_ = used_laser_data;
	laser_safe.laser_range_straight_redu2_r_ = used_laser_data;
	laser_safe.laser_range_straight_buff_r_ = used_laser_data;
	laser_safe.laser_range_straight_stop_r_ = used_laser_data;
	laser_safe.laser_range_left_redu1_r_ = used_laser_data;
	laser_safe.laser_range_left_redu2_r_ = used_laser_data;
	laser_safe.laser_range_left_buff_r_ = used_laser_data;
	laser_safe.laser_range_left_stop_r_ = used_laser_data;
	laser_safe.laser_range_right_redu1_r_ = used_laser_data;
	laser_safe.laser_range_right_redu2_r_ = used_laser_data;
	laser_safe.laser_range_right_buff_r_ = used_laser_data;
	laser_safe.laser_range_right_stop_r_ = used_laser_data;

	laser_safe.laser_range_left_straight_redu1_ = used_laser_data;
	laser_safe.laser_range_left_straight_redu2_ = used_laser_data;
	laser_safe.laser_range_left_straight_buff_ = used_laser_data;
	laser_safe.laser_range_left_straight_stop_ = used_laser_data;
	laser_safe.laser_range_right_straight_redu1_ = used_laser_data;
	laser_safe.laser_range_right_straight_redu2_ = used_laser_data;
	laser_safe.laser_range_right_straight_buff_ = used_laser_data;
	laser_safe.laser_range_right_straight_stop_ = used_laser_data;

	std::cout<<"Laser Shape polygon with laser dx:"<<laser_para.laser_dx_<<" dy:"<<laser_para.laser_dy_<<std::endl;

	lasersafe_init_shape(frames_.front_straight_redu1_);
	lasersafe_get_range(laser_safe.laser_range_straight_redu1_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_straight_redu2_);
	lasersafe_get_range(laser_safe.laser_range_straight_redu2_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_straight_buff_);
	lasersafe_get_range(laser_safe.laser_range_straight_buff_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_straight_stop_);
	lasersafe_get_range(laser_safe.laser_range_straight_stop_,laser_para.laser_dx_,laser_para.laser_dy_);

	lasersafe_init_shape(frames_.front_left_redu1_);
	lasersafe_get_range(laser_safe.laser_range_left_redu1_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_left_redu2_);
	lasersafe_get_range(laser_safe.laser_range_left_redu2_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_left_buff_);
	lasersafe_get_range(laser_safe.laser_range_left_buff_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_left_stop_);
	lasersafe_get_range(laser_safe.laser_range_left_stop_,laser_para.laser_dx_,laser_para.laser_dy_);

	lasersafe_init_shape(frames_.front_right_redu1_);
	lasersafe_get_range(laser_safe.laser_range_right_redu1_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_right_redu2_);
	lasersafe_get_range(laser_safe.laser_range_right_redu2_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_right_buff_);
	lasersafe_get_range(laser_safe.laser_range_right_buff_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.front_right_stop_);
	lasersafe_get_range(laser_safe.laser_range_right_stop_,laser_para.laser_dx_,laser_para.laser_dy_);


	lasersafe_init_shape(frames_.back_straight_redu1_);
	lasersafe_get_range(laser_safe.laser_range_straight_redu1_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_straight_redu2_);
	lasersafe_get_range(laser_safe.laser_range_straight_redu2_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_straight_buff_);
	lasersafe_get_range(laser_safe.laser_range_straight_buff_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_straight_stop_);
	lasersafe_get_range(laser_safe.laser_range_straight_stop_r_,laser_para.laser_dx_,laser_para.laser_dy_);

	lasersafe_init_shape(frames_.back_left_redu1_);
	lasersafe_get_range(laser_safe.laser_range_left_redu1_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_left_redu2_);
	lasersafe_get_range(laser_safe.laser_range_left_redu2_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_left_buff_);
	lasersafe_get_range(laser_safe.laser_range_left_buff_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_left_stop_);
	lasersafe_get_range(laser_safe.laser_range_left_stop_r_,laser_para.laser_dx_,laser_para.laser_dy_);

	lasersafe_init_shape(frames_.back_right_redu1_);
	lasersafe_get_range(laser_safe.laser_range_right_redu1_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_right_redu2_);
	lasersafe_get_range(laser_safe.laser_range_right_redu2_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_right_buff_);
	lasersafe_get_range(laser_safe.laser_range_right_buff_r_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.back_right_stop_);
	lasersafe_get_range(laser_safe.laser_range_right_stop_r_,laser_para.laser_dx_,laser_para.laser_dy_);

	lasersafe_init_shape(frames_.left_straight_redu1_);
	lasersafe_get_range(laser_safe.laser_range_left_straight_redu1_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.left_straight_redu2_);
	lasersafe_get_range(laser_safe.laser_range_left_straight_redu2_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.left_straight_buff_);
	lasersafe_get_range(laser_safe.laser_range_left_straight_buff_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.left_straight_stop_);
	lasersafe_get_range(laser_safe.laser_range_left_straight_stop_,laser_para.laser_dx_,laser_para.laser_dy_);

	lasersafe_init_shape(frames_.right_straight_redu1_);
	lasersafe_get_range(laser_safe.laser_range_right_straight_redu1_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.right_straight_redu2_);
	lasersafe_get_range(laser_safe.laser_range_right_straight_redu2_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.right_straight_buff_);
	lasersafe_get_range(laser_safe.laser_range_right_straight_buff_,laser_para.laser_dx_,laser_para.laser_dy_);
	lasersafe_init_shape(frames_.right_straight_stop_);
	lasersafe_get_range(laser_safe.laser_range_right_straight_stop_,laser_para.laser_dx_,laser_para.laser_dy_);

	std::cout<<"*** Laser safe  calculate  min_range Over!***"<<std::endl;

	find_showframes(laser_safe,frames,laser_para.laser_dx_,laser_para.laser_dy_);

}


void Obstacle_Detector::lasersafe_init_shape( const std::string &str_shape )
{
//std::cout<<"shape string :  "<<str_shape<<std::endl;
	std::vector<Sxy> shape;

	std::vector<std::string> vres;
	cComm::SplitString(str_shape,";",vres);
	std::vector<std::string>::iterator it = vres.begin();
	for ( ; it !=  vres.end() ; ++it)
	{
		std::string str = *it;
		std::vector<std::string> vpos;
		cComm::SplitString(str,":",vpos);

		if (vpos.size() > 1)
		{
			Sxy xy;
			cComm::ConvertToNum(xy.x_,vpos[0]);
			cComm::ConvertToNum(xy.y_,vpos[1]);
			shape.push_back(xy);
		}
	}
	lasersafe_init_shape(shape);
}
void Obstacle_Detector::lasersafe_init_shape( const std::vector<Sxy> &shape )
{
	//std::cout<<"begin init polygon para pos size:"<<shape.size()<<std::endl;
	if (shape.size() < 4)
	{
		Sxy xy;
		xy.x_ = 0.5;
		xy.y_ = 0.4;
		shape_list_.push_back(xy);

		xy.x_ = -0.3;
		xy.y_ = 0.4;
		shape_list_.push_back(xy);

		xy.x_ = -0.3;
		xy.y_ = -0.4;
		shape_list_.push_back(xy);

		xy.x_ = 0.5;
		xy.y_ = -0.4;
		shape_list_.push_back(xy);

		xy.x_ = 0.5;
		xy.y_ = 0.4;
		shape_list_.push_back(xy);
	}else
	{
		shape_list_ = shape;
	}

	std::vector<Sxy> shape_new;

	//recalculate the shape
	//1 find robot head in which shape range
	//2 redefine the shape_list,make sure the robot head is the first shape point
	//  find robot head ray line intersection with the shape range
	//  add first shape point to the list
	//  add last shape point to the list

	std::vector<Sxy>::iterator it1 = shape_list_.begin();
	std::vector<Sxy>::iterator it2 = shape_list_.begin();
	it2++;


	//printf shape
	it1 = shape_list_.begin();
	//std::cout<<" ******* One  Shape in baselink******frame_cnt_ : "<<frame_cnt_<<std::endl;
	frame_cnt_++;
	for ( ; it1 != shape_list_.end() ; ++it1 )
	{
		//std::cout<<"  x: "<<it1->x_<<"  y: "<<it1->y_<<std::endl;
	}

//	std::cout<<"******Sxy  shapelist in baselink !******"<<std::endl;
}

void Obstacle_Detector::lasersafe_get_range(SLaser& min_range,F32 dx,F32 dy){
	//clear all data
	memset(min_range.data_,0,sizeof(F32)*LASER_COUNT);
	//set base laser to base odom
	TF_SERVER.set_base_laser_in_base_link(dx,dy,0);


	//先判断激光是否在避障shape 四边形内
	bool is_inside = check_inside(shape_list_,dx,dy);
	if(!is_inside){
	 	for( int i = 0 ; i < LASER_COUNT ; ++i){
	 		min_range.data_[i] =  0;
		}
	 	std::cout<<"laser is  not inside, set 0 to  all range !!!!! "<<std::endl;

	 	return ;
	}

	//1 transfer shape list to laser base
	//2 find every ray of laser beems intersection to the shape
	std::vector<Sxy> shape_laser_;
	std::vector<Sxy>::iterator it = shape_list_.begin();
	for ( ; it != shape_list_.end();  ++it)
	{
		VecPosition v(it->x_,it->y_);
		TF_SERVER.tf_base_link_to_base_laser(v);
		Sxy xy;
		xy.x_ = v.getX();
		xy.y_ = v.getY();
		shape_laser_.push_back(xy);
		//std::cout<<"base laser, x:  "<< v.getX()<<"  y:   "<<v.getY()<<std::endl;
		//std::cout<<" distance in baselaser:  "<< sqrt(v.getX()*v.getX()+v.getY()*v.getY())<<std::endl;
	}
	//std::cout<<"******Sxy  shapelist in baselaser !******"<<std::endl;
	//out1.close();
	shape_list_ = shape_laser_;

	F32 start_a = Rad2Deg(min_range.start_angle_);
	F32 diff_a = Rad2Deg(min_range.resolution_);
	F32 angle = 0 ;
	F32 last_range = 0;

	//std::cout<<"start_a : "<<start_a<<std::endl;
	for ( int i = 0 ; i < LASER_COUNT ; ++i )
	{
		angle = start_a + diff_a*i;

		VecPosition ray(2.0,angle,POLAR);

		Line seg_line;
		Line ray_line = Line::makeLineFromPositionAndAngle(VecPosition(0,0),ray.getDirection());

		VecPosition shape_vertex;
		VecPosition shape_v1;
		VecPosition shape_v2;
		bool check = false;

		if (lasersafe_getCrossLine2(seg_line,ray,shape_v1,shape_v2,shape_vertex,check))
		{
			VecPosition intersection;
			ray_line.getIntersection(seg_line,intersection);
			F32 range = intersection.getMagnitude();

			if(range >shape_vertex.getMagnitude()){
				range = shape_vertex.getMagnitude();
				//std::cout<<" angle : "<<angle<<" out of range ! fix ,range: "<<range<<std::endl;
			}
			min_range.data_[i] = range;
			last_range = min_range.data_[i];
			//std::cout<<"i:"<<i<<" get crossline,last_range: "<<last_range<<"  range:"<<range<<std::endl;
		}
		else
		{
			//min_range.data_[i]  = last_range;
			min_range.data_[i] = shape_vertex.getMagnitude();
			//std::cout<<"i: "<<i<<"  can't get crossline so data_[i]:"<<min_range.data_[i]<<std::endl;
		}
		//std::cout<<"max  range: "<<shape_vertex.getMagnitude()<< "    x: "<<shape_vertex.getX()<<" y: "<<shape_vertex.getY()<<" extracheck: "<<check<<std::endl;
		//std::cout<<"i: "<<i<<" angle : "<<angle<<" range:"<<min_range.data_[i]<<std::endl;
	}

	//fix err
	last_range = 0;
 	for( int i = 0 ; i < LASER_COUNT ; ++i){
 		//std::cout<<"i: "<<i<<" range:"<<min_range.data_[i]<<std::endl;
 		if( fabs(min_range.data_[i]) < 1e-6){
 			min_range.data_[i] = last_range;
 		}
		if ( min_range.data_[i] < min_range.range_min_){

			F32 angle  = angle = start_a + diff_a*i;

			std::cout<<"err filter:"<<angle<<"  range:"<<min_range.data_[i]<<std::endl;

			min_range.data_[i] = min_range.range_min_;
		}
		last_range = min_range.data_[i];
		//std::cout<<"i: "<<i<<" range:"<<min_range.data_[i]<<std::endl;
	}

 	//output shape in base_link
 	//outputshape(min_range,dx,dy);

 	//std::cout<<"********Get range end!***********"<<std::endl;
}

bool Obstacle_Detector::lasersafe_getCrossLine2( Line &ln,VecPosition ray ,VecPosition& shape_vertex1,VecPosition& shape_vertex2,VecPosition&close_shape_vertex,bool& check)
{
	std::vector<Sxy>::iterator seg_begin = shape_list_.end();
	std::vector<Sxy>::iterator seg_end = shape_list_.end();

	if (lasersafe_find_ray_in_seg(seg_begin,seg_end,ray,close_shape_vertex,check))
	{
		VecPosition vb(seg_begin->x_,seg_begin->y_);
		VecPosition ve(seg_end->x_,seg_end->y_);

		ln = Line::makeLineFromTwoPoints(vb,ve);

		shape_vertex1 = vb;
		shape_vertex2 = ve;

		if(check){
			getclosevertex(ln,vb,ve,ray,close_shape_vertex);
		}


		return true;
	}

//    std::cout<<"can't find ray_in seg"<<std::endl;
	return false;
}

bool Obstacle_Detector::lasersafe_find_ray_in_seg(std::vector<Sxy>::iterator &it_begin,std::vector<Sxy>::iterator &it_end,const VecPosition& ray,VecPosition& close_vertex,bool& check)
{

	it_begin = shape_list_.begin();
	it_end = shape_list_.begin();
	it_end++;

	int r = 1;
	for ( ; it_end != shape_list_.end(); ++it_end)
	{
		VecPosition v1(it_begin->x_,it_begin->y_);
		VecPosition v2(it_end->x_,it_end->y_);

		F32 a1 = VecPosition::IntersectionAngle(v1,ray);
		F32 a2 = VecPosition::IntersectionAngle(v2,ray);
		//std::cout<<"a1 : "<<a1<<"  a2 :  "<<a2<<" r: "<<r<<std::endl;
		r++;

		F32 angle1 = 0.0;
		F32 angle2 = 0.0;
		if((180-fabs(a1))<45){
			angle1 = fabs(180-fabs(a1));
		}else{
			angle1 = fabs(a1);
		}
		if((180-fabs(a2))<45){
			angle2 = fabs(180-fabs(a2));
		}else{
			angle2 = fabs(a2);
		}

		if(angle1>angle2){
			close_vertex = v2;
			if(angle2<10){
				check = true;
			}else{
				check = false;
			}

		}
		else{
			close_vertex = v1;
			if(angle1<10){
				check = true;
			}else{
				check = false;
			}
		}
		if((a1 < 0) && (a2 > 0)){
			return true;
		}
		it_begin++;
	}

	//如果没有得到正确的线段，认为夹角最小的顶点是距离极值
	F32 interangle1 = 0.0;
	F32 interangle2 = 0.0;
	F32 interangle3 = 0.0;
	F32 interangle4 = 0.0;

	it_begin = shape_list_.begin();
	VecPosition v1(it_begin->x_,it_begin->y_);
	interangle1 = fabs(VecPosition::IntersectionAngle(v1,ray));

	it_begin =  shape_list_.begin()+1;
	VecPosition v2(it_begin->x_,it_begin->y_);
	interangle2 = fabs(VecPosition::IntersectionAngle(v2,ray));

	it_begin =  shape_list_.begin()+2;
	VecPosition v3(it_begin->x_,it_begin->y_);
	interangle3 = fabs(VecPosition::IntersectionAngle(v3,ray));
	it_begin =  shape_list_.begin()+3;
	VecPosition v4(it_begin->x_,it_begin->y_);
	interangle4 = fabs(VecPosition::IntersectionAngle(v4,ray));


	int v_in = 1;
	find_mina(interangle1,interangle2,interangle3,interangle4,v_in);
	switch(v_in){
		case 1:
			close_vertex = v1;
			break;
		case 2:
			close_vertex = v2;
			break;
		case 3:
			close_vertex = v3;
			break;
		case 4:
			close_vertex = v4;
			break;
		default:
			close_vertex = v1;
	}
	return false;
}

void Obstacle_Detector::outputshape(SLaser shape,F32 laser_dx,F32 laser_dy)
{
	F32 diff_a =  shape.resolution_;
	F32 start_angle  =  shape.start_angle_;

	//std::ofstream out("3.txt");
	test_out.open("1left_shapes.txt");

	if(test_out!=NULL){
		for(int i = 0;i<LASER_COUNT;i++){
			float range = shape.data_[i];
			float angle = start_angle + diff_a*i;

			test_out<<"out  i: "<<i<<" range: "<<range<<" angle:"<<angle<<std::endl;

			//test_out<<"  "<<dx<<"    "<<angle<<"   \n";
		}
		//test_out<<"****** "<<"x y in baselink"<<" ***********  \n";
		for(int i = 0;i<LASER_COUNT;i++){
			float range = shape.data_[i];
			float angle = start_angle + diff_a*i;
			float x_= range*cos(angle)+laser_dx;
			float y_= range*sin(angle)+laser_dy;
			//std::cout<<"out  i: "<<i<<" x: "<<x_<<" y: "<<y_<<" angle:"<<angle<<"  range:"<<range<<std::endl;
			//test_out<<" range: "<<range<<" angle:"<<angle<<" laser_dx:"<<laser_dx<<"   \n";
			test_out<<x_<<"  "<<y_<<"   \n";
			//test_out<<"  "<<dx<<"    "<<angle<<"   \n";
		}

		//test_out<<"****** "<<"output  end "<<" ***********  \n";
	}

	test_out.close();
	//out.close();
	return ;
}

void Obstacle_Detector::shapexy_to_vector(const Shape_xy shapexy,std::vector<Sxy> &shape,F32 x_diff,F32 y_diff,bool front/*true:left, false: right*/)
{
	shape.clear();
	Sxy xy;
	if(front == false){   //right
		xy.x_ = shapexy.x1_ + y_diff;
		xy.y_ = shapexy.y1_ - x_diff;
		shape.push_back(xy);
		xy.x_ = shapexy.x2_ ;
		xy.y_ = shapexy.y2_ ;
		shape.push_back(xy);
		xy.x_ = shapexy.x3_;
		xy.y_ = shapexy.y3_;
		shape.push_back(xy);
		xy.x_ = shapexy.x4_ - y_diff;;
		xy.y_ = shapexy.y4_ - x_diff;
		shape.push_back(xy);
		xy.x_ = shapexy.x1_ + y_diff;
		xy.y_ = shapexy.y1_ - x_diff;
		shape.push_back(xy);

	}else{     				//left
		xy.x_ = shapexy.x1_;
		xy.y_ = shapexy.y1_;
		shape.push_back(xy);
		xy.x_ = shapexy.x2_ + y_diff;
		xy.y_ = shapexy.y2_ + x_diff;
		shape.push_back(xy);
		xy.x_ = shapexy.x3_ - y_diff;
		xy.y_ = shapexy.y3_ + x_diff;
		shape.push_back(xy);
		xy.x_ = shapexy.x4_ ;
		xy.y_ = shapexy.y4_ ;
		shape.push_back(xy);
		xy.x_ = shapexy.x1_;
		xy.y_ = shapexy.y1_;
		shape.push_back(xy);
	}
}

void Obstacle_Detector::shapexy_to_vector(const Shape_xy shapexy,std::vector<Sxy> &shape,F32 x_diff,F32 y_diff,F32 angle,bool front/*true:front, false: back*/)
{
	shape.clear();
	F32 rotate_angle = Deg2Rad(angle);
	Sxy xy;
	//turn_x=(t_x-o_x)*cos(ang)-(t_y-o_y)*sin(ang)+o_x;
	//turn_y=(t_x-o_x)*sin(ang)+(t_y-o_y)*cos(ang)+o_y;
	if(front == true){        //front 1 2
		F32 new_x = shapexy.x1_ + x_diff;
		F32 new_y = shapexy.y1_ - y_diff;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x2_ + x_diff;
		new_y = shapexy.y2_ + y_diff;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x3_;
		new_y = shapexy.y3_;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x4_;
		new_y = shapexy.y4_;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x1_ + x_diff;
		new_y = shapexy.y1_ - y_diff;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

	}else{                    //back 3 4
		F32 new_x = shapexy.x1_;
		F32 new_y = shapexy.y1_;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x2_ ;
		new_y = shapexy.y2_ ;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x3_ - x_diff;
		new_y = shapexy.y3_ + y_diff;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x4_ - x_diff;
		new_y = shapexy.y4_ - y_diff;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);

		new_x = shapexy.x1_;
		new_y = shapexy.y1_ ;
		xy.x_ = new_x*cos(rotate_angle) - new_y*sin(rotate_angle);
		xy.y_ = new_x*sin(rotate_angle) + new_y*cos(rotate_angle);
		shape.push_back(xy);
	}
}
void Obstacle_Detector::setFrames(Shape_xy robot_shape,F32 r1_x,F32 r2_x,F32 b_x,F32 s_x,F32 r1_y,F32 r2_y,F32 b_y,F32 s_y,F32 rotate_angle)
{//std::cout<<"r1_x: "<<r1_x<<" r2_x:"<<r2_x<<" b_x:"<<b_x<<" s_x:"<<s_x<<"r1_y: "<<r1_y<<" r2_y:"<<r2_y<<" b_y:"<<b_y<<" s_y:"<<s_y<<std::endl;
	std::vector<Sxy>  oneshape;

	//preProcess value
	r1_x = fabs(r1_x);
	r2_x = fabs(r2_x);
	b_x = fabs(b_x);
	s_x = fabs(s_x);
	r1_y = fabs(r1_y);
	r2_y = fabs(r2_y);
	b_y = fabs(b_y);
	s_y = fabs(s_y);
	//rotate_angle = abs(rotate_angle);
	//straight fornt
	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,0.0,true);
	frames_.front_straight_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,0.0,true);
	frames_.front_straight_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,0.0,true);
	frames_.front_straight_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,0.0,true);
	frames_.front_straight_stop_ = oneshape;
    //straight back
	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,0.0,false);
	frames_.back_straight_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,0.0,false);
	frames_.back_straight_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,0.0,false);
	frames_.back_straight_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,0.0,false);
	frames_.back_straight_stop_ = oneshape;

	//straight left
	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,true);
	frames_.left_straight_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,true);
	frames_.left_straight_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,true);
	frames_.left_straight_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,true);
	frames_.left_straight_stop_ = oneshape;
    //straight right
	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,false);
	frames_.right_straight_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,false);
	frames_.right_straight_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,false);
	frames_.right_straight_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,false);
	frames_.right_straight_stop_ = oneshape;

	//left rotate frame
	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,rotate_angle,true);
	frames_.front_left_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,rotate_angle,true);
	frames_.front_left_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,rotate_angle,true);
	frames_.front_left_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,rotate_angle,true);
	frames_.front_left_stop_ = oneshape;

	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,rotate_angle,false);
	frames_.back_left_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,rotate_angle,false);
	frames_.back_left_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,rotate_angle,false);
	frames_.back_left_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,rotate_angle,false);
	frames_.back_left_stop_ = oneshape;
//
	//right rotate frame
	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,-rotate_angle,true);
	frames_.front_right_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,-rotate_angle,true);
	frames_.front_right_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,-rotate_angle,true);
	frames_.front_right_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,-rotate_angle,true);
	frames_.front_right_stop_ = oneshape;

	shapexy_to_vector(robot_shape,oneshape,r1_x,r1_y,-rotate_angle,false);
	frames_.back_right_redu1_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,r2_x,r2_y,-rotate_angle,false);
	frames_.back_right_redu2_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,b_x,b_y,-rotate_angle,false);
	frames_.back_right_buff_ = oneshape;
	shapexy_to_vector(robot_shape,oneshape,s_x,s_y,-rotate_angle,false);
	frames_.back_right_stop_ = oneshape;

	return ;
}

bool Obstacle_Detector::LoadFrames()
{

	std::map<int,std::string>  index_frame_map;
	std::map<std::string, SubNode> ini_map;

	//step1: readfile & push into map
	load_ini(index_frame_map);
	std::cout<<"111111  get   index_frame_map,size:"<<index_frame_map.size()<<std::endl;

	//step2:  convert to vector<Sxy>
	std::map<int,std::vector<Sxy>> map_Sxys;
	int n = 0;
	std::map<int,std::string>::iterator it_map = index_frame_map.begin();
	for(;it_map!= index_frame_map.end();it_map++){
		std::string str;
		str = it_map->second;
		//std::cout<<"string : "<<str<<std::endl;
		std::vector<Sxy> fm;
		stringtoSxy(str,fm);
		map_Sxys[n] = fm;
		n++;
	}

	//step3: push into frames_
	if(map_Sxys.size()==16){

		frames_.front_straight_redu1_  = map_Sxys[0];
		frames_.front_straight_redu2_ = change_VecSxy(map_Sxys[0],-0.2,0);
		frames_.front_straight_buff_ = change_VecSxy(map_Sxys[8],0.15,0);
		frames_.front_straight_stop_ = map_Sxys[8];

		frames_.front_left_redu1_   = map_Sxys[1];
		frames_.front_left_redu2_   = change_VecSxy(map_Sxys[1],-0.2,0);
		frames_.front_left_buff_    = change_VecSxy(map_Sxys[9],0.15,0);
		frames_.front_left_stop_    = map_Sxys[9];

		frames_.front_right_redu1_   = map_Sxys[2];
		frames_.front_right_redu2_   = change_VecSxy(map_Sxys[2],-0.2,0);
		frames_.front_right_buff_   = change_VecSxy(map_Sxys[10],0.15,0);
		frames_.front_right_stop_   = map_Sxys[10];


		frames_.back_straight_redu1_ = map_Sxys[3];
		frames_.back_straight_redu2_ =change_VecSxy(map_Sxys[3],-0.2,0);
		frames_.back_straight_buff_ =change_VecSxy(map_Sxys[11],0.15,0);
		frames_.back_straight_stop_ = map_Sxys[11];

		frames_.back_left_redu1_ = map_Sxys[4];
		frames_.back_left_redu2_ =change_VecSxy(map_Sxys[4],-0.2,0);
		frames_.back_left_buff_ =change_VecSxy(map_Sxys[12],0.15,0);
		frames_.back_left_stop_ = map_Sxys[12];

		frames_.back_right_redu1_ = map_Sxys[5];
		frames_.back_right_redu2_ =change_VecSxy(map_Sxys[5],-0.2,0);
		frames_.back_right_buff_ =change_VecSxy(map_Sxys[13],0.15,0);
		frames_.back_right_stop_ = map_Sxys[13];

		frames_.left_straight_redu1_ = map_Sxys[6];
		frames_.left_straight_redu2_ =change_VecSxy(map_Sxys[6],0,-0.2);
		frames_.left_straight_buff_ =change_VecSxy(map_Sxys[14],0,0.15);
		frames_.left_straight_stop_ = map_Sxys[14];

		frames_.right_straight_redu1_ = map_Sxys[7];
		frames_.right_straight_redu2_ =change_VecSxy(map_Sxys[7],0,-0.2);
		frames_.right_straight_buff_ =change_VecSxy(map_Sxys[15],0,0.15);
		frames_.right_straight_stop_ = map_Sxys[15];

		return true;
	}

	return false;
}
void Obstacle_Detector::stringtoSxy(std::string shape_string,std::vector<Sxy> &shape)
{

	std::vector<std::string> vres;
	cComm::SplitString(shape_string,";",vres);
	std::vector<std::string>::iterator it = vres.begin();
	for ( ; it !=  vres.end() ; ++it)
	{
		std::string str = *it;
		std::vector<std::string> vpos;
		cComm::SplitString(str,":",vpos);

		if (vpos.size() > 1)
		{
			Sxy xy;
			cComm::ConvertToNum(xy.x_,vpos[0]);
			cComm::ConvertToNum(xy.y_,vpos[1]);
			shape.push_back(xy);
		}
	}

}

void Obstacle_Detector::load_ini(std::map<int,std::string>& frames)
{
	frames.clear();

	std::string strline = "";
	std::string current_index = "";
	int current = 0;
	std::string frame;

	std::ifstream infile;
	std::string ini_name(LASER_SAFE_FRAMES);
	infile.open(ini_name.c_str());
	if(infile.is_open()){
		while (getline(infile,strline))
		{
			int left_pos = 0 ;
			int right_pos = 0;

			if ( (strline.npos != (left_pos = strline.find("[")))
				&& (strline.npos != (right_pos = strline.find("]"))))
			{
				std::string dirname = strline.substr(left_pos+1,right_pos-1);
				dirname = trimString(dirname);
				//std::cout<<"dir name: "<<dirname<<std::endl;
				current_index = dirname;
			}
			else
			{
				cComm::ConvertToNum(current,current_index);
				{
					frame = trimString(strline);
					if (frame.size()>0)
					{
						frames[current]=frame;
						//std::cout<<"index: "<<current<<" frame: "<<frame<<std::endl;
					}
				}
			}
		}
		infile.close();
	}
	else{
		std::cout<<"open file error!!!!!!!"<<std::endl;
	}
}
std::string &Obstacle_Detector::trimString(std::string &str)
{
// 	int pos = 0;
//
// 	while(str.npos != (pos = str.find(" ")))
// 	{
// 		str = str.replace(pos, pos + 1, "");
// 	}

	int index = 0;
	if( !str.empty())
	{
		while( (index = str.find(' ',index)) != std::string::npos)
		{
			str.erase(index,1);
		}
	}

	return str;
}

std::vector<Sxy> Obstacle_Detector::change_VecSxy( std::vector<Sxy> s,float dx,float dy)
{
	if(s.size()>4){
		std::vector<Sxy> frame;
		Sxy op;
		op.x_ = s[0].x_+dx;   //0
		op.y_ = s[0].y_+dy;
		frame.push_back(op);
		op.x_ = s[1].x_+dx;   //1
		op.y_ = s[1].y_+dy;
		frame.push_back(op);
		op.x_ = s[2].x_+dx;   //2
		op.y_ = s[2].y_+dy;
		frame.push_back(op);
		op.x_ = s[3].x_+dx;   //3
		op.y_ = s[3].y_+dy;
		frame.push_back(op);
		op.x_ = s[4].x_+dx;   //4
		op.y_ = s[4].y_+dy;
		frame.push_back(op);

		return frame;

	}else{
		return s;
	}



}

 void Obstacle_Detector::s_check_Obstacle(const NS_Laser_Safe::Laser_safe&laser_safe_info,
		                    NS_Laser_Safe::Obstacle_Status &cs,int set,int &total)
 {
		SLaser laser_data = laser_safe_info.laser_data_;
		SLaser laser_range_stop = laser_safe_info.laser_range_straight_stop_;

		//检测制动框 有无障碍物
		if(check_data(laser_data,laser_range_stop,set,total)){
			cs = NS_Laser_Safe::Obstacle_Status::OBSTACLE_STOP;
			return ;
		}else{
			cs = NS_Laser_Safe::Obstacle_Status::NO_OBSTACLE;
			return ;
		}
 }

 bool Obstacle_Detector::check_data(SLaser &laser_data, SLaser &laser_range,int set,int& total)
 {

		int obstacle = 0;

		int current_index = 0;

		//std::cout<<"laser_data_.resolution_:"<<laser_data_.resolution_<<" laser_data_.range_min_:"<<laser_data_.range_min_ <<" laser_data_.range_max_:"<<laser_data_.range_max_<<std::endl;
		int diff_index = 0  / laser_data.resolution_;
		//std::cout<<"current angle:"<<angle<<" diff_index:"<<diff_index<<std::endl;

		for (int i = 0  ; i < LASER_COUNT ; ++i )
		{

			current_index = i + diff_index;

			if( (current_index < 0) || (current_index >= LASER_COUNT)){
				continue;
			}

			if( laser_data.data_[current_index] < laser_data.range_min_){
				//std::cout<<" laser:"<<laser_data_.data_[current_index]<<" min:"<<laser_data_.range_min_<<std::endl;
				continue;
			}
			if( laser_data.data_[current_index] > laser_data.range_max_){
				continue;
			}

			if (  laser_data.seg_[current_index]   )
			{

				if( laser_data.data_[current_index] < laser_range.data_[i] ){
					obstacle++;
					F32 angle = laser_data.start_angle_ + laser_data.resolution_*i;
					angle = Rad2Deg(angle);
					//std::cout<<"obstacle:"<<obstacle<<" angle:"<<angle<<" dis:"<<laser_data.data_[current_index];
					//std::cout<<" min_dis: "<<laser_range.data_[i]<<std::endl;
				}
			}

		}
		total = obstacle;
		if ( obstacle > set )
		{

			return true;
		}

		return false;

 }

 void Obstacle_Detector::find_mina(F32 a1,F32 a2,F32 a3, F32 a4,int& ind)
 {
	 ind = 1;
	 if((a1<a2)&&(a1<a2)&&(a1<a2)){
		 ind = 1;
	 }

	 if((a2<a1)&&(a2<a3)&&(a2<a4)){
		 ind = 2;
	 }
	 if((a3<a1)&&(a3<a2)&&(a3<a4)){
		 ind = 3;
	 }
	 if((a4<a1)&&(a4<a2)&&(a4<a3)){
		 ind = 4;
	 }

	 return ;
 }
 void Obstacle_Detector::getclosevertex(Line seg_ln,VecPosition vb,VecPosition ve,VecPosition ray,VecPosition&close_shape_vertex)
 {
		Line ray_line = Line::makeLineFromPositionAndAngle(VecPosition(0,0),ray.getDirection());

		VecPosition intersection;
		ray_line.getIntersection(seg_ln,intersection);

		F32  dis1 = ray.getDistanceTo(vb);
		F32  dis2 = ray.getDistanceTo(ve);

//		std::cout<<"dis1: "<<dis1<<"   dis2: "<<dis2<<std::endl;
//		std::cout<<"ray: "<<ray.getX()<<"  "<<ray.getY()<<std::endl;
//		std::cout<<"vb: "<<vb.getX()<<"  "<<vb.getY()<<std::endl;
//		std::cout<<"ve: "<<ve.getX()<<"  "<<ve.getY()<<std::endl;
		if(fabs(dis1)<fabs(dis2)){
			close_shape_vertex = vb;

		}else{
			close_shape_vertex = ve;

		}

 }

 bool Obstacle_Detector::check_inside(std::vector<Sxy> shape,F32 P_x,F32 P_y)
 {
	 if(shape.size()<4){
		 return false;
	 }
	Sxy  A = shape[0];
	Sxy  B = shape[1];
	Sxy  C = shape[2];
	Sxy  D = shape[3];
	float a = (B.x_ - A.x_)*(P_y - A.y_) - (B.y_ - A.y_)*(P_x - A.x_);
	float b = (C.x_ - B.x_)*(P_y - B.y_) - (C.y_ - B.y_)*(P_x - B.x_);
	float c = (D.x_ - C.x_)*(P_y - C.y_) - (D.y_ - C.y_)*(P_x - C.x_);
	float d = (A.x_ - D.x_)*(P_y - D.y_) - (A.y_ - D.y_)*(P_x - D.x_);

	//std::cout<<"a: "<<a<<" b: "<<b<<" c: "<<c<<" d: "<<d<<std::endl;
	// a b c d 有值接近与0，说明点在边上
	if(fabs(a)<0.05 || fabs(b)<0.05 || fabs(c)<0.05 || fabs(d)<0.05){
		return false;
	}
	 //a b c d 同号 说明点在四边形内 ;
	if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0) ) {
		return true;
	}

	return false;
 }


 void Obstacle_Detector::find_showframes( NS_Laser_Safe::Laser_safe &laser_safe,SLaserSafe_Frames & frames,F32 dx,F32 dy)
 {
	 frames.laser_dx_ = dx;
	 frames.laser_dy_ = dy;
	 //FS
	 frames.frame_[0] =laser_safe.laser_range_straight_redu1_;
	 frames.frame_[1] =laser_safe.laser_range_straight_redu2_;
	 frames.frame_[2] =laser_safe.laser_range_straight_buff_;
	 frames.frame_[3] =laser_safe.laser_range_straight_stop_;
	 //FL
	 frames.frame_[4] =laser_safe.laser_range_left_redu1_;
	 frames.frame_[5] =laser_safe.laser_range_left_redu2_;
	 frames.frame_[6] =laser_safe.laser_range_left_buff_;
	 frames.frame_[7] =laser_safe.laser_range_left_stop_;
	 //FR
	 frames.frame_[8] =laser_safe.laser_range_right_redu1_;
	 frames.frame_[9] =laser_safe.laser_range_right_redu2_;
	 frames.frame_[10] =laser_safe.laser_range_right_buff_;
	 frames.frame_[11] =laser_safe.laser_range_right_stop_;
	 //BS
	 frames.frame_[12] =laser_safe.laser_range_straight_redu1_r_;
	 frames.frame_[13] =laser_safe.laser_range_straight_redu2_r_;
	 frames.frame_[14] =laser_safe.laser_range_straight_buff_r_;
	 frames.frame_[15] =laser_safe.laser_range_straight_stop_r_;
	 //BL
	 frames.frame_[16] =laser_safe.laser_range_left_redu1_r_;
	 frames.frame_[17] =laser_safe.laser_range_left_redu2_r_;
	 frames.frame_[18] =laser_safe.laser_range_left_buff_r_;
	 frames.frame_[19] =laser_safe.laser_range_left_stop_r_;
	 //BR
	 frames.frame_[20] =laser_safe.laser_range_right_redu1_r_;
	 frames.frame_[21] =laser_safe.laser_range_right_redu2_r_;
	 frames.frame_[22] =laser_safe.laser_range_right_buff_r_;
	 frames.frame_[23] =laser_safe.laser_range_right_stop_r_;
	 //LS
	 frames.frame_[24] =laser_safe.laser_range_left_straight_redu1_;
	 frames.frame_[25] =laser_safe.laser_range_left_straight_redu2_;
	 frames.frame_[26] =laser_safe.laser_range_left_straight_buff_;
	 frames.frame_[27] =laser_safe.laser_range_left_straight_stop_;
	 //RS
	 frames.frame_[28] =laser_safe.laser_range_right_straight_redu1_;
	 frames.frame_[29] =laser_safe.laser_range_right_straight_redu2_;
	 frames.frame_[30] =laser_safe.laser_range_right_straight_buff_;
	 frames.frame_[31] =laser_safe.laser_range_right_straight_stop_;




 }

 NS_Laser_Safe::SHAPE_DIR Obstacle_Detector::get_dir()
 {

	 return current_dir_;
 }


 void Obstacle_Detector::tim310_preProcessSpeed(const F32 &vx,const F32 &vy,const F32 &vw,NS_Laser_Safe::SHAPE_DIR &dir)
 {

		if(vx>0.05){
			dir = NS_Laser_Safe::SHAPE_DIR::FRONT_STRAIGHT;
			if(vx>0.4){
				dir = NS_Laser_Safe::SHAPE_DIR::FAST_FRONT;
			}
			if(vw>0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::FRONT_LEFT;
			}else if(vw<-0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::FRONT_RIGHT;
			}
			tim310_last_dir_ = dir;
		}else if(vx<-0.05){
			dir = NS_Laser_Safe::SHAPE_DIR::BACK_STRAIGHT;
			if(vw>0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::BACK_LEFT;
			}else if(vw<-0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::BACK_RIGHT;
			}
			tim310_last_dir_ = dir;
		}else if(vy>0.05){
			dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
			if(vw>0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
			}else if(vw<-0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
			}
			tim310_last_dir_ = dir;
		}else if(vy<-0.05){
			dir = NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT;
			if(vw>0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT;
			}else if(vw<-0.1){
				dir = NS_Laser_Safe::SHAPE_DIR::RIGHT_STRAIGHT;
			}
			tim310_last_dir_ = dir;
		}else if(fabs(vw)>0.05){
			dir = NS_Laser_Safe::SHAPE_DIR::LEFT_STRAIGHT;
			tim310_last_dir_ = dir;
			std::cout<<"  now   rotation ,vw:"<<vw<<" vx:"<<vx<<std::endl;
		}else{
			dir = tim310_last_dir_;
		}
		//std::cout<<"current dir: "<<int(dir)<<std::endl;


 }

 void Obstacle_Detector::save_laser(const SLaser &laser_data,NS_Laser_Safe::SHAPE_DIR dir)
 {
 	std::fstream fo;
 	fo.open("Obstacle_laserdata.txt",std::ios::out | std::ios::app);
 	std::cout<<"save obstacle laser file:Obstacle_laserdata.txt"<<std::endl;

 	if(fo.is_open())
 	{
 		fo<<(int)dir<<"  ";
 		for( int i = 0 ; i < LASER_COUNT ; ++i){
 			fo<<laser_data.data_[i]<<" ";
 		}
 		fo<<std::endl;
 		fo.close();
 	}

 }


