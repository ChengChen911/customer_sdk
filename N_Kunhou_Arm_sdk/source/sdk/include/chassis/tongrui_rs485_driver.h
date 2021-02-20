/*
 * tongrui_rs485_driver.h
 *
 *  Created on: July 20, 2020
 *      Author: SudekiMing
 */

#ifndef  _CHASSIS_TONGRUI_RS485_DRIVER_
#define _CHASSIS_TONGRUI_RS485_DRIVER_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "driver_base.h"
#include "t20_dio.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class tongrui_rs485_driver : Driver_base
{
public:
	tongrui_rs485_driver();
	~tongrui_rs485_driver();

protected:

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);

	bool init_driver(void);
	bool open_transfer_device(void);
	void close_driver(void);

	void setRPM(int id,F32 v);
	F32 getSpeed(int id);
	F32 getDiffAngle(int id);

private:
	void setfunction(void);

private:

	cTransferDevice* pTransDevice_;
	std::string port_;

	F32 reduction_;
	U32 counts_;
	U32 jack_speed_;

	F32 left_set_rpm_;
	F32 right_set_rpm_;
	F32 left_cur_rpm_;
	F32 right_cur_rpm_;

	cTimerDiff dt_test_;
	cTimerDiff dt_;
	F32 left_dangle_;
	F32 right_dangle_;
	bool first_dangle_;

	S32 left_pos_;
	S32 left_pos_last_;
	S32 left_pos_dete_;
	S32 right_pos_;
	S32 right_pos_last_;
	S32 right_pos_dete_;

	cTimerDiff  Time_dt_;

private:
	bool init_updown();
	void SendData( U8* s_data,U16 len);
	void ReadData( U8* r_data,int &len,int need,int timeout);
	void SYNTRON_DRV_SetParam(U8 Driver_ID,S16 para);
	void SYNTRON_DRV_VelocityMode(U8 Driver_ID,S16 Temp_Velocity);
	void SYNTRON_DRV_GetVelocity(U8 Driver_ID);
	void SYNTRON_DRV_GetPos(U8 Driver_ID);

private:
	boost::mutex mu_action;
	S32 sub_index_;
	S32 fnc_code_;

	bool hasTask;
	bool isWorking;
	bool is_top_;
	bool is_bottom_;
	U8 din_id_top_;
	U8 din_id_bottom_;
	S32 fnc_time_;

	bool init_climb_;
	bool first_init_;

	S32 climb_reset_up_;
	S32 climb_reset_down_;
	S32 set_enc_top_;
	S32 set_enc_bottom_;
	S32 cur_enc_;
	U16 updown_time_;
	U8 set_enc_top_data_[4];
	U8 set_enc_bottom_data_[4];

	U8 dio_fork_status_;
};

#endif	// _CHASSIS_TONGRUI_RS485_DRIVER_
