/*
 * tongrui_driver.h
 *
 *  Created on: July 20, 2020
 *      Author: SudekiMing
 */

#ifndef  _CHASSIS_TONGRUI_DRIVER_
#define _CHASSIS_TONGRUI_DRIVER_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "driver_base.h"
#include "t20_dio.h"
#include "chassis/canalyst2.h"

#define LEN_MAX   			    102400
#define ID_LEFT_WHEEL    	0x0501
#define ID_RIGHT_WHEEL    	0x0502
#define ID_CLIMB		    		0x0503

#define RID_LEFT_WHEEL    	0x0601
#define RID_RIGHT_WHEEL   0x0602
#define RID_CLIMB		    	0x0603

#define ID_SOURCE		   		0x0000


class cTransferDevice;

class tongrui_driver : Driver_base
{
public:
	tongrui_driver();
	~tongrui_driver();

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
	void SYNTRON_DRV_GetPara(void);
	void parse_left(U8* Data8);
	void parse_right(U8* Data8);
	void parse_climb(U8* Data8);

	bool b_run_;
	canalyst2 canalyst2_;
	F32 reduction_;
	U32 counts_;

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

	boost::mutex mu_speed;

private:
	bool init_updown();
	void CAN_SendData(U16 id1, U8 id2, U8 funcode, U16 regi, U8 *s_data, S16 num);

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

#endif	// _CHASSIS_TONGRUI_DRIVER_
