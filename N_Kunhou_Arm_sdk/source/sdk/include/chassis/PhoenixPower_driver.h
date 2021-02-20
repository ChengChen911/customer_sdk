/*
 * PhoenixPower_driver.h
 *
 *  Created on: Aug 01, 2020
 *      Author: SudekiMing
 */

#ifndef  _CHASSIS_PHOENIXPOWER_DRIVER_
#define _CHASSIS_PHOENIXPOWER_DRIVER_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "driver_base.h"
#include "t20_dio.h"
#include "chassis/canalyst2.h"

#define FRONT_WALKING    	0x01
#define FRONT_STEERING    	0x01
#define BACK_WALKING		0x02
#define BACK_STEERING		0x02

#define MOTOR_FORWARD		0x00
#define MOTOR_REVERSES		0x01

#define MOTOR_WALKING		0x01
#define MOTOR_STEERING		0x02

#define ENABLE    					0x01
#define DISENABLE		    	0x00

#define	Limiter( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )	// amplitude limiting

class cTransferDevice;

class PhoenixPower_driver : Driver_base
{
public:
	PhoenixPower_driver();
	~PhoenixPower_driver();

protected:

private:
	std::string port_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	F32 reduction_ratio_steer_;
	U32 counts_steer_;
	U32 one_wheel_counts_steer_;

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);
	void close_driver(void);
	bool open_transfer_device(void);
	bool init_driver(void);

	void setRPM(int id,F32 v);
	void setAngle(int id,F32 rad_angle);
	F32 getSpeed(int id);
	F32 getAngle(int id);
	F32 getDiffAngle(int id);

private:
	bool first_odo_;
	canalyst2 canalyst2_;

	cTimerDiff dt_;
	cTimerDiff left_dt_;
	cTimerDiff  Time_dt_;

	S16 real_velocity[2];
	S16 real_angle[2];
	S32 real_position[2];

	S32 FrontWalking_Pos;
	S32 BackWalking_Pos;

	S32 Last_FrontWalking_Pos;
	S32 Last_BackWalking_Pos;

	S32 Incr_FrontWalking_Pos;
	S32 Incr_BackWalking_Pos;

	F32 Current_FrontWalking_Vel;
	F32 Current_BackWalking_Vel;

	F32 Current_FrontSteering_Angle;
	F32 Current_BackSteering_Angle;

	U8 Scram_;

	bool Filter_FrontFlag_;
	bool Filter_BackFlag_;

	F32 Actual_FrontAngle;
	F32 Target_FrontAngle;
	F32 Actual_BackAngle;
	F32 Target_BackAngle;

	bool ID1_AnglePackage_JudgeFlag_;
	bool ID2_AnglePackage_JudgeFlag_;
	bool ID1_PosPackage_JudgeFlag_;
	bool ID2_PosPackage_JudgeFlag_;

	F32 Last_FrontSteering_Angle;
	F32 Last_BackSteering_Angle;
	S32 Last_Incr_FrontWalking_Pos;
	S32 Last_Incr_BackWalking_Pos;


private:
	S32 get_din_value(int id,SDI  din);
	F32 GetDiffPos(int id );
	void init_PhoenixPower_para(void);
	void PhoenixPower_DRV_SetParam(U8 Driver_ID,U8  Message_ID,S16 para);
	void PhoenixPower_DRV_SetVelocity(U8 Driver_ID,S16 Motor_Velocity);
	void PhoenixPower_DRV_SetAngle(U8 Driver_ID,S16 Motor_Angle);
	void PhoenixPower_DRV_GetVelocity(U8 Driver_ID);
	void PhoenixPower_DRV_GetAngle(U8 Driver_ID);
	void PhoenixPower_DRV_GetPos(U8 Driver_ID);

};

#endif	// _CHASSIS_PHOENIXPOWER_DRIVER_
