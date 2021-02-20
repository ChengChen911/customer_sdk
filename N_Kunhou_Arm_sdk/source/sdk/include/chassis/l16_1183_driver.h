#ifndef _CHASSIS_L16_1183_DRIVER_
#define _CHASSIS_L16_1183_DRIVER_

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "driver_base.h"
#include "t20_dio.h"
#include "chassis/t20_k_rpm.h"
#include "chassis/t20_k_angle.h"
#include "chassis/l16_k_fork.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class l16_1183_driver : Driver_base
{
public:
	l16_1183_driver();
	~l16_1183_driver();

	void setAngleOri(int id,F32 rad_angle);
	void setRPMOri(int id,F32 v);
	S32 ReadL16Fork(void);
	void setfork(U32 fun_code_ri, S32 fork_hight_ri);

	F32 Speed_PID(F32 Set_rpm, F32 Cur_rpm);
	F32 Angle_PID(F32 Set_Angle, F32 Cur_Angle);
protected:


private://virtual customized :TODO
	void setPara(std::string name ,std::string value);

	bool init_driver();
	bool open_transfer_device();
	void close_driver();
	std::string port_;

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

	void parse_181(U8* Data8);
	void parse_183(U8* Data8);
	void parse_38x_fault(U8* Data8);

	void parse_187_RSpeed(U8* Data8);
	void parse_287_FSpeed(U8* Data8);

private:
	U8 fault_state_;//0:no fault  1:fault
	U8 reset_state_;//0:no reset  1:reset finish  2:reseting

	boost::mutex mu_forkliftstatus_;
	U8 ControlMode_;//0:auto 1:manual
	U8 Scram_;//0:no stop 1:stop
	U8 NoCanData_;//0:normal 1:nodata
	U8 ForkLiftErr_;//0:normal 1:reset
	U8 Com_dio_;//0:ok  1:no

	void Reset_Fork(S16 reset);
	U8 Reset_State();
private:
	cTransferDevice* pTransDevice_;

	void ReadTruckStatus(void);
	void SendData( U8* s_data,U16 len );
	void ReadData(  U8* r_data,int &len,const int &need  );
	bool ModBus_Read(U8* read_data, const int &len);

	void Set_Fork_pid_para();
	void Set_Kuebler_Type();
	void init_wheel_pid();

	void th_setangle();
	F32 ang;
	F32 rpmmm;
	boost::mutex mu_ang;

	modbus_10_data half_MB_10_;
	modbus_03_data half_MB_03_;

	U8 Send_10_[64];
	U8 Send_03_[8];
	U8 num_10_;

private:
	S32 Fork_Hight_;

	S16 SetAangle_;
	S16 SetSpeed_;
	S16 SetUpDown_;

	U16 SetManualRSpeed_;
	U16 SetManualFSpeed_;

	bool is_cmd_mode_;//bit0
	//bool fork_status_;//bit1
	bool is_forward_;//bit2
	bool is_tiller_;
	bool is_guard_;
	bool is_pedal_;
	bool is_stand_;

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
    F32 Set_angle_last_;
    F32 Angle_pid_;


	U8 dio_fork_status_;
	S32 fork_status_;
	S32 sub_index_;
	S32 fnc_code_;
	S32 para_;
	boost::mutex mu_action;
	boost::mutex mu_st;
	boost::mutex mu_manual;
	bool precise_;
	bool hasTask;
	bool isWorking;
	int  ct_;

	cTimerDiff test_dt_;

private://usb can
	typedef enum{ON_BOARD,USB_LD,USB_BOX} CAN_TYPE;
	CAN_TYPE can_data_type_;
	std::string can_port_;

private://canalyst2
	canalyst2 canalyst2_;

private:
	typedef struct _Kuebler
	{
		S16 type;
		S16 mode;
		S16 offset;
	}Kuebler;

	S16 Kuebler_Type_;//1-5:modbus; 6-10:canopen
	/*****************************
	1:D8.4B0.0200.F86E.6112.S058
	2:D8.1106.F86E.6112
	3:
	6:D8.4B1.0300.F82A.2123
	*****************************/
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

private:
	F32 js_speed_;
	F32 js_angle_;

};



#endif//_CHASSIS_L16_1183_DRIVER_
