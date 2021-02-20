// Created by shibangyao on 2019/1/10.
#ifndef NANWANG_DRIVER_H_
#define NANWANG_DRIVER_H_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "chassis/driver_base.h"
//#include "t20_dio.h"
#include "chassis/usb_can_ld.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class nanwang_driver : Driver_base
{
public:
	nanwang_driver();
	~nanwang_driver();

protected:

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);
	bool init_driver();
	void close_driver();
	bool open_transfer_device();

	void setRPM(int id,F32 v);
	F32 getSpeed(int id);
	void setAngle(int id,F32 rad_angle);
	F32 getAngle(int id);
	F32 getDiffAngle(int id);

private:
	cTransferDevice* pTransDevice_;
	std::string port_;
	bool sigle_fork_;
	bool up_task_;

	bool b_run_;
	//speed and angle
	S16 SetSpeed_;
	F32 CurSpeed_;
	F32 reduction_ratio_;
	//my test
	bool my_test_;



	F32 SetAngle_;
	F32 CurAngle_;
	S32 Set_Enc_pos_;
	S32 Cur_Enc_pos_;
	F32 Enc_resolution_;
	F32 steer_reduction_ratio_;
	S32 angle_zero_pos_;
	F32 angle_zero_;

	//diffangle
	cTimerDiff dt_;
	F32 dangle_;

	//fork hight
	bool fork_reach_;

	F32 SetForkHight_;
	F32 CurForkHight_;
	F32 LastCurForkHight_;
	F32 time_;

	U32 fnc_code_;
	S32 dio_fork_status_;
	F32 ZeroHight_;

	bool hasTask;
	bool fork_init;
	bool turn_init;
	//test
	F32 fork_need_speed_;
	bool quick_stop_;
	bool up_trip_;
	bool down_trip_;
	bool kinco_error_;

	bool start_t_;
	bool end_t_;
	cTimerDiff d_ti_;

	S16 tim0;
	F32 forward_dx_;
	F32 set_forward_para_;

	S16 fork_pid_time;
	F32 ForkSpeedMax_up_;
	F32 ForkSpeedMax_down_;
	//fork pid para
	F32 Fork_U_P_;
	F32 Fork_U_I_;
	F32 Fork_U_D_;
	F32 Fork_D_P_;
	F32 Fork_D_I_;
	F32 Fork_D_D_;

	F32 Fork_P_P_;
	F32 Fork_P_I_;
	F32 Fork_P_D_;
	F32 Fork_S_P_;
	F32 Fork_S_I_;
	F32 Fork_S_D_;

	//fork pos pid

	F32 ForkHightErr_;
	F32 ForkHightErrSum_;
	F32 LastForkHightErr_;

	//fork speed pid
	F32 SetForkSpeed_;
	F32 ForkCurSpeed_;
	F32 LastForkSpeed_;
	F32 ForkSpeedErr_;
	F32 LastForkSpeedErr_;
	F32 ForkSpeedErrSum_;

//	S32 speed_now_;
//	S32 speed_last_;
//	S32 speed_err_max_;
//	S32 ForkspeedErr_;
//	S32 ForkspeedErr_sum_;
//	S32 LastForkspeedErr_;
	cTimerDiff speed_dt_;

	bool pick_up_;
	S16 ForkSpeedOut_;

private:
	void test_init();

	void th_read();

	void read_speed(U8* Data8);
	void read_angle(U8* Data8);
	void read_hight(U8* Data8);

	void send_speed(S16 &speed);
	void send_angle(S32 &angle);
	void send_forkspeed(F32 forkspeed);

	S32 Fork_pid();
	F32 Fork_pos_pid(S16 sethight);
	F32 Fork_speed_pid(F32 setspeed);

	void CANopen_Send_PDO(U32 cobid, U8 *buf,int num);
	void SendData( U8* s_data,U16 len );
	void ReadData(  U8* r_data,int &len,const int &need  );

	void task_init();
	void steer_init();
	void kinco_init();
	void Set_Fork_pid_para();

private://usb can
	usb_can_ld usb_ld_;

	typedef enum{ON_BOARD,USB_LD,USB_BOX} CAN_TYPE;
	CAN_TYPE can_data_type_;
	std::string can_port_;

private://canalyst2
	canalyst2 canalyst2_;

	F32 js_speed_;
	F32 js_angle_;

private:
	enum My_status{
		IDLE_ = 0,
		FORWARD_,
		BACK_,
		UP_FORK_,
		DOWN_FORK_,
		FINISH_,
		SLEEP_
	}my_status_;
	typedef struct _Fork_Pid
	{
		F32 max_speed_up;
		F32 max_speed_down;
		F32 P_up_speed;
		F32 I_up_speed;
		F32 D_up_speed;
		F32 P_up_pos;
		F32 I_up_pos;
		F32 D_up_pos;
		F32 P_down_pos;
		F32 I_down_pos;
		F32 D_down_pos;
	}Fork_Pid;

	Fork_Pid fork_pid_;

	boost::mutex hee_mu_;
};






#endif /* NANWANG_DRIVER_H_ */
