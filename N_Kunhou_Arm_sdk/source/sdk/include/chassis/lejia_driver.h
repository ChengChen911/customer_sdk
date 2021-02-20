#ifndef _CHASSIS_LEJIA_DRIVER_
#define _CHASSIS_LEJIA_DRIVER_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "driver_base.h"
#include "t20_dio.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class lejia_driver : Driver_base
{
public:
	lejia_driver();
	~lejia_driver();

protected:

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);

	bool init_driver();
	bool open_transfer_device();
	void close_driver();

	void setRPM(int id,F32 v);
	F32 getSpeed(int id);

	F32 getDiffAngle(int id);

private:
	void setfunction();

private:
	void th_read();
	void parse_left(U8* Data8);
	void parse_right(U8* Data8);
	void parse_climb(U8* Data8);

	bool b_run_;
	boost::mutex mu_canalyst_;
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

private:
	bool init_updown();
	void SendCANData(U16 id1, U8 id2, U8 funcode, U16 regi, U8 *s_data, S16 num);
	void SendData( U8* s_data,U16 len );
	void ReadData(  U8* r_data,int &len,const int &need  );

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

#endif//_CHASSIS_LEJIA_DRIVER_
