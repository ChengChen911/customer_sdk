#ifndef _ROBO_DRIVER_CHENG_H_
#define _ROBO_DRIVER_CHENG_H_

#include "TimerDiff.h"
#include "chassis/driver_base.h"
#include "linux_driver/gpio.h"

class cTransferDevice;

class robomodule_driver : Driver_base
{

public:
	robomodule_driver();
	~robomodule_driver();

protected:

private://virtual
	void setPara(std::string name ,std::string value);

	bool init_driver();
	void close_driver();
	void setRPM(int id,F32 v);
	//void setAngle(int id,F32 rad_angle);
	F32 GetDiffPos(int id );
	void setAngle(int id,F32 rad_angle);
	F32 getAngle( int id );
	bool open_transfer_device();

	F32 getSpeed(int id);
	F32 getDiffAngle(int id);
private:
	typedef enum{Null=0, Speed_NO_PID=1,Speed=2, pos=3, Speed_pos=4}Mode_;
	typedef struct
	{
		bool rec_speed;
		bool rec_ctl;
	}rec_;
	typedef struct
	{
		Mode_ mode;
		S32 data;
	}control_mode_;
	control_mode_ Angle_mode_;
	control_mode_ Speed_mode_;

	S16 Set_Speed_;
	S32 Set_Angle_;
	S16 Set_PWM_;

	gpio gpio_dio_ ;
	int scram_;

	std::string port_;
	std::string port1_;
	cTransferDevice* pTransDevice_;
	cTransferDevice* pTransDevice1_;

	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	S32 pos_buf_;
	S32 pos_;
	S32 last_pos_;
	S32 deta_pos_;

	S16 C_Speed_buf_;
	F32 C_Speed_;

	F32 Angle_;
	F32 C_Angle_;

	F32 qc_;		//counts of pi
	F32 reduction_steer_;
	F32 one_circle_counts_enc_;

	U8 Ctl_buf_;
	U8 Ctl_;
	S32 Ctl_pos_buf_;
	S32 Ctl_pos_;
	S32 Ctl_pos_last_;

	U8 speed_data_[2];
	U8 angle_pos_data_[4];
	U8 speed_pos_data_[4];

	S16 temp_pwm_;
	S16 temp_speed_;

	F32 Angle_zero_;

private:
public:
	boost::thread* th_get_CTL_;

	bool th_get_CTL_run_;
	bool th_get_speed_pos_run_;

	boost::mutex mu_ctl_;
	boost::mutex mu_ctl_pos_;
	boost::mutex mu_speed_;
	boost::mutex mu_pos_;

	typedef enum{HEAD_1,KEY1_1,KEY2_1,CTL0_1,CTL1_1,CRC0_1,CRC1_1,CRC2_1,CRC3_1,CRC4_1} State_ctl;
	State_ctl sta_ctl_;
	typedef enum{HEAD_2,KEY1_2,KEY2_2,CUR0_2,CUR1_2,SPEED0_2,SPEED1_2,POS0_2,POS1_2,POS2_2,CRC} State_ctl_pos;
	State_ctl_pos sta_ctl_pos_;
	typedef enum{HEAD_3,KEY1_3,KEY2_3,CUR0_3,CUR1_3,SPEED0_3,SPEED1_3,POS0_3,POS1_3,POS2_3,CRC3} State_speed_pos;
	State_speed_pos sta_speed_pos_;

private:
public:
	bool open_ttyS2_device();
	bool open_ttyS3_device();
	void Send_Driver(int id, U8* s_data,U16 len);
	void Read_Driver(int id, U8* r_data,int &len,const int &need);
	void SendData( U8* s_data,U16 len);
	void SendData1( U8* s_data,U16 len);
	void ReadData(  U8* r_data,int &len,const int &need);
	void ReadData1(  U8* r_data,int &len,const int &need);

	bool Steer_calibration(int id);
	void th_Get_CTL(void);
	void th_Get_Speed_pos(void);
	void state_mathine_CTL(U8 data);
	void state_mathine_pos(U8 data);
	void state_mathine_speed_pos(U8 data);


	bool Reset_driver(int id);
	bool Mode_driver(int id, Mode_ mode);
	bool Receive_Message(int id, rec_ rec_msg, U8 timeout);
	void Send_Speed_NO_PID(int id, S16 Speed);
	void Send_Speed(int id, S16 Speed, S16 PWM_max);
	void Send_Pos(int id, S32 Pos, S16 PWM_max);
	void Send_Speed_Pos(int id, S32 Pos, S16 PWM_max, S16 PWM_speed);

};

#endif /* _ROBO_DRIVER_CHENG_H_ */
