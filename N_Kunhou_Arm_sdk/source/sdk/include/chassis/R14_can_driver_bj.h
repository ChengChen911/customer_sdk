#ifndef _CHASSIS_R14BJ_CAN_DRIVER_CHENG_
#define _CHASSIS_R14BJ_CAN_DRIVER_CHENG_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "driver_base.h"
#include "t20_dio.h"
#include "chassis/t20_k_rpm.h"
#include "chassis/t20_k_angle.h"
#include "chassis/l16_k_fork.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class r14_can_driver_bj : Driver_base
{
public:
	r14_can_driver_bj();
	~r14_can_driver_bj();

	void setAngleOri(int id,F32 rad_angle);
	void setRPMOri(int id,F32 v);
	void setfork(U32 fun_code_ri, S32 fork_hight_ri);

	F32 Speed_PID(F32 Set_rpm, F32 Cur_rpm);
	F32 Angle_PID(F32 Set_Angle, F32 Cur_Angle);
protected:


private://virtual customized :TODO
	void setPara(std::string name ,std::string value);

	bool init_driver();
	bool open_transfer_device();
	bool open_transfer_device_clamp();
	void close_driver();
	void close_driver_clamp();
	std::string port_;
	std::string port_clamp_;

	F32 reduction_ratio_;


	void setRPM(int id,F32 v);
	F32 getSpeed(int id);

	F32 getDiffAngle(int id);

	void setAngle(int id,F32 rad_angle);

	F32 getAngle(int id);

private:

	cTimerDiff dt_;
	F32 dangle_;

//	F32 c_action_wheel_speed_;
//	F32 c_action_wheel_angle_;

private:
	bool b_run_;
	void th_read();

//	boost::mutex mu_feedback_;
//	ST20_FeedBack feedback_data_;
//	CSBuffer<F32,1> list_speed_;
	SBuffer<F32> list_speed_;
	boost::mutex mu_feedback_;
	F32 wheel_angle_deg_;
	F32 wheel_speed_rpm_;
	int alive_count_;

	void parse_201_angle(U8* Data8);
	void parse_221_speed(U8* Data8);
	void parse_222_park(U8* Data8);
	void parse_225_244_fault(U8* Data8);

	void Reset_Fork(S16 reset);
	U8 Reset_State();

private:
	cTransferDevice* pTransDevice_;
	cTransferDevice* pTransDevice_clamp_;

	void SendData( U8* s_data,U16 len );
	void ReadData(  U8* r_data,int &len,const int &need  );
	void SendData_clamp( U8* s_data,U16 len );
	void ReadData_clamp(  U8* r_data,int &len,const int &need  );
	bool ModBus_Read(U8* read_data, const int &len);

	void Set_Fork_pid_para();
	void init_wheel_pid();

	void Creat_taskfnc(U8 clamp_state);
	void Send_Task_PLC();
	void publish_task_status();

	void th_setangle();
	F32 manual_ang;
	boost::mutex mu_ang;

	modbus_10_data half_MB_10_;
	modbus_03_data half_MB_03_;

	U8 Send_10_[64];
	U8 Send_03_[8];
	U8 num_10_;

private:

	S16 SetAangleSpeed_;
	S16 SetSpeed_;
	S16 SetUpDown_;

	t20_k_rpm k_para;
	t20_k_angle k_angle_para;
	l16_k_fork k_fork_para;
	F32 angle_zero_;
//Speed_PID
    F32 set_rpm_;
    F32 rpm_Kp_;
    F32 rpm_Ki_;
    F32 rpm_Kd_;
    F32 rpm_Ki_max_;
    F32 rpm_last_error_;
    F32 rpm_current_error_;
    F32 rpm_err_sum_;
//Angle_PID
    F32 Angle_Kp_;
    F32 Angle_Ki_;
    F32 Angle_Kd_;
    F32 Ki_max_;
    F32 Angle_err_;
    F32 Angle_err_last_;
    F32 Angle_err_sum_;

    F32 AngleSpeed_k_;
    F32 AngleSpeed_b_;

	U32 fnc_code_;
	U32 fnc_code_fork_;
	U32 fnc_code_clamp_;
	S32 fork_para_;
	S16 fork_cur_pos_;
	U8 fork_status_;
	U8 clamp_status_;
	U16 clamp_set_reg_;
	U16 clamp_get_reg_;
	bool clamp_clear_;

	boost::mutex mu_action;
	boost::mutex mu_st;
	bool hasTask;
	bool hasTask_fork;
	bool hasTask_clamp;
	bool hasTask_fork_clamp;

	F32 js_speed_;
	F32 js_angle_;

	U8 park_status_;
	U8 park_set_;

	U8 fault_state_;//0:fault  1:no fault
	U8 reset_state_;//0:no reset  1:reset finish  2:reseting

	cTimerDiff test_dt_;

	std::string can_port_;

private://canalyst2
	canalyst2 canalyst2_;

private:
	typedef struct _Fork_Pid
	{
		S16 max_speed_up;
		S16 max_speed_down;
		S16 P_up_speed;
		S16 I_up_speed;
		S16 D_up_speed;
		S16 P_up_pos;
		S16 I_up_pos;
		S16 D_up_pos;
		S16 P_down_pos;
		S16 I_down_pos;
		S16 D_down_pos;
	}Fork_Pid;

	Fork_Pid fork_pid_;


};
#endif//_CHASSIS_R14BJ_CAN_DRIVER_CHENG_
