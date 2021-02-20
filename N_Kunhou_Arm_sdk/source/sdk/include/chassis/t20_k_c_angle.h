#ifndef _CHASSIS_T20_K_C_ANGLE_
#define _CHASSIS_T20_K_C_ANGLE_

#include <boost/thread.hpp>
#include <map>

#include "TimerDiff.h"
#include "Comm/buffer_con.hpp"
#include "MyDefine.h"
#include "TransferDevice.h"

class rino_gyro{

public:

	rino_gyro();
	~rino_gyro();

	void load_para();
	void save_para();

	F32 trans_angle( F32 angle );

	bool calibration(F32 &k_angle , F32 &gyro_angle, const F32 &set_angle, const F32 &current_angle ,const F32 &p_step);

	void set_k_para(const F32 &angle,const F32 &k_para);

	bool init(F32 &angle);
	void stop_read();
	void get_gyro(F32 &vel, F32 &acc, F32 &angle);

private:
	void open_device();
	void close_device();
	void Angle_clean();

	void ReadData(  U8* r_data,int &len,const int &need  );
	void SendData( U8* s_data,U16 len );

	void th_read();
	void state_mathine(U8 data);

private:
	typedef enum{IDLE,MARK,LEN,ADR,CMD,VEL_1,VEL_2,VEL_3,ACC_1,ACC_2,ACC_3,ANG_1,ANG_2,ANG_3} State_;
	State_ state_;
	boost::thread* get_data_;

	cTransferDevice* pTransDevice_;
	std::string port_;
	
	boost::mutex mu_gyro_v_;
	boost::mutex mu_gyro_ac_;
	boost::mutex mu_gyro_an_;

	U8 velocity_data_[3];
	U8 acc_data_[3];
	U8 angle_data_[3];

	U8 Gyro_velocity_symbol_;
	U8 Gyro_acc_symbol_;
	U8 Gyro_angle_symbol_;

	F32 Gyro_velocity_;
	F32 Gyro_acc_;
	F32 Gyro_angle_;
	F32 First_Gyro_angle_;

	U32 check_sum;
	U8 check_;

	bool b_run_;
	bool first_init_;

	std::map<F32,F32> m_k_para_;


	F32 min_angle_k_;
	F32 mid_angle_k_;
	F32 max_angle_k_;

	F32 min_angle_;
	F32 mid_angle_;
	F32 max_angle_;

	std::string Chassis_Type_;
	const char* k_cur_angle_para_;

	S8 calibration_time_;
};


#endif//_CHASSIS_T20_K_C_ANGLE_


